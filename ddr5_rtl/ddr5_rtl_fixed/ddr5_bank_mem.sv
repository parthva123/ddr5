// ============================================================================
// ddr5_bank_mem.sv
//
// Behavioral DDR5 SDRAM array model — one instance per DEVICE (all 32
// banks), meant to sit on the DFI-style pins driven by ddr5_channel_ctrl /
// ddr5_dfi_if and give the controller something real to talk to in
// simulation. This is a functional model, not a bit-accurate DRAM: it
// gets the command semantics and row-buffer behavior right (ACT opens a
// row into a per-bank buffer, RD/WR touch the open row, PRE writes the
// buffer back to a sparse backing array) but does not model AC timing,
// refresh charge leakage, or analog behavior. Pair it with
// ddr5_bank_tracker (which enforces timing) on the controller side; this
// module assumes commands arriving at its ports are already legal.
//
// Storage is a sparse associative array keyed by {bank, row}, each entry
// holding one full page (2^COL_W x DQ_W), so a 24Gb device does not need
// to be allocated up front — only rows actually touched consume memory.
//
// Command decode mirrors ddr5_types_pkg::ddr5_cmd_e and the bg/ba/row/col
// fields used throughout ddr5_top, so this model can be dropped in next
// to a_ca/a_dq or driven directly from a testbench without re-deriving
// CA-bus decode logic.
// ============================================================================
`ifndef DDR5_BANK_MEM_SV
`define DDR5_BANK_MEM_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"

module ddr5_bank_mem
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
#(
  parameter int DQ_W_LOCAL = DQ_W   // sub-channel data width (32)
)(
  input  logic       ck,
  input  logic       rst_n,

  // Decoded command interface (one command per issue, first CA half).
  // Drive these directly from ddr5_channel_ctrl's decoded cmd_* /
  // cmd_bg / cmd_ba outputs plus the row/col that were sent to the
  // encoder, or from a testbench driving ddr5_req_t fields directly.
  input  logic             cmd_valid,     // pulse: a command is issued this ck
  input  ddr5_cmd_e        cmd,
  input  logic [2:0]       bg,
  input  logic [1:0]       ba,
  input  logic [ROW_W-1:0] row,           // valid on ACT
  input  logic [COL_W-1:0] col,           // valid on RD/WR (burst start col)
  input  logic              ap,           // auto-precharge on RD/WR
  input  logic [4:0]        burst_len,    // beats for this RD/WR (8 or 16;
                                           // driven by the caller from bc8,
                                           // matching ddr5_data_engine's
                                           // burst_len = bc8 ? 8 : 16)

  // Write-data stream: one CK = one beat of DQ_W_LOCAL bits (model treats
  // this as one column beat per ck while wr_active is high; if you want
  // true DDR double-pumping, feed rising/falling phases on alternate
  // half-cycles from your testbench clock).
  input  logic                     wr_active,
  input  logic [DQ_W_LOCAL-1:0]    wr_data,
  input  logic [DQ_W_LOCAL/8-1:0]  wr_dm_n,     // active-low byte mask

  // Read-data stream, latency-free from this model's perspective — the
  // CL/CWL pipeline already lives in ddr5_read_data_engine on the
  // controller side, so this model just presents data the cycle after
  // the RD command's column is latched.
  output logic                     rd_valid,
  output logic [DQ_W_LOCAL-1:0]    rd_data,
  output logic                     rd_last,

  // Debug / scoreboard hook: total burst length in beats for the access
  // currently in flight, and how many beats have completed.
  output logic [5:0]               beat_cnt
);

  localparam int NUM_COLS = (1 << COL_W);

  // Sparse per-bank page storage: page_mem[bank] is an associative array
  // keyed by row, each value a full page of NUM_COLS x DQ_W_LOCAL bits.
  typedef logic [DQ_W_LOCAL-1:0] page_t [0:NUM_COLS-1];
  page_t                page_mem [NUM_BANKS] [logic [ROW_W-1:0]];

  // Row buffer state per bank.
  logic             row_open [NUM_BANKS];
  logic [ROW_W-1:0] open_row [NUM_BANKS];
  page_t            row_buf  [NUM_BANKS];

  logic [4:0] idx;
  assign idx = bank_idx(bg, ba);

  logic [COL_W-1:0] beat_col;  // combinational per-cycle beat address,
                                // computed unconditionally each ck below
                                // so it's a plain variable, not an
                                // `automatic` block-local (kept out for
                                // tool portability; only read while
                                // burst_active is high).

  // ------------------------------------------------------------------
  // In-flight burst tracking for RD/WR column bursts. Burst length is
  // driven by the same BL16/BC8/BL32 convention as the rest of the
  // design; this model defaults to BL16 (8 beats, matching TCCD_S)
  // unless a testbench overrides via burst_len below.
  // ------------------------------------------------------------------
  logic       burst_active, burst_is_write;
  logic [4:0] burst_bank_idx;
  logic [COL_W-1:0] burst_col_base;
  logic [4:0] burst_len_beats;   // 4 (BC8), 8 (BL16), or 16 (BL32)
  logic [4:0] burst_beat;

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      for (int b = 0; b < NUM_BANKS; b++) begin
        row_open[b] <= 1'b0;
        open_row[b] <= '0;
      end
      burst_active    <= 1'b0;
      burst_is_write  <= 1'b0;
      burst_bank_idx  <= '0;
      burst_col_base  <= '0;
      burst_len_beats <= '0;
      burst_beat      <= '0;
      rd_valid        <= 1'b0;
      rd_last         <= 1'b0;
      rd_data         <= '0;
      beat_cnt        <= '0;
    end else begin
      rd_valid <= 1'b0;
      rd_last  <= 1'b0;

      // ---------------- command decode ----------------
      if (cmd_valid) begin
        unique case (cmd)

          CMD_ACT: begin
            // Opening an already-open bank is a controller/tracker bug,
            // not something this model should silently paper over —
            // flag it and still proceed with the new row to keep sim
            // running.
            if (row_open[idx])
              $display("[ddr5_bank_mem] WARNING: ACT to already-open bank %0d at row %0d (was %0d)",
                        idx, row, open_row[idx]);
            row_open[idx] <= 1'b1;
            open_row[idx] <= row;
            if (page_mem[idx].exists(row)) row_buf[idx] <= page_mem[idx][row];
            else                           for (int c = 0; c < NUM_COLS; c++) row_buf[idx][c] <= '0;
          end

          CMD_RD: begin
            if (!row_open[idx])
              $display("[ddr5_bank_mem] WARNING: RD to closed bank %0d", idx);
            burst_active    <= 1'b1;
            burst_is_write  <= 1'b0;
            burst_bank_idx  <= idx;
            burst_col_base  <= col;
            burst_len_beats <= burst_len;
            burst_beat      <= '0;
            if (ap) begin
              // Auto-precharge: write the row buffer back and close the
              // bank once the burst finishes (handled at burst-end below
              // for simplicity, matching tRTP+tRP intent functionally).
            end
          end

          CMD_WR: begin
            if (!row_open[idx])
              $display("[ddr5_bank_mem] WARNING: WR to closed bank %0d", idx);
            burst_active    <= 1'b1;
            burst_is_write  <= 1'b1;
            burst_bank_idx  <= idx;
            burst_col_base  <= col;
            burst_len_beats <= burst_len;
            burst_beat      <= '0;
          end

          CMD_PRE: begin
            // Single-bank precharge (addressed by bg/ba), matching
            // ddr5_cmd_encoder's CMD_PRE / ddr5_top's a_cmd_pre.
            if (row_open[idx]) begin
              page_mem[idx][open_row[idx]] <= row_buf[idx];
              row_open[idx] <= 1'b0;
            end
          end

          CMD_PRE_ALL: begin
            // All-bank precharge, matching CMD_PRE_ALL / a_cmd_pre_all.
            // (This design has no separate per-bank-group precharge —
            // only single-bank PRE and all-bank PRE_ALL exist.)
            for (int b = 0; b < NUM_BANKS; b++) begin
              if (row_open[b]) begin
                page_mem[b][open_row[b]] <= row_buf[b];
                row_open[b] <= 1'b0;
              end
            end
          end

          default: ; // REF_AB, REF_PB, REF_SB, MRW, MRR, ZQ_START,
                      // ZQ_LATCH, NOP: no array-contents effect in this
                      // model. Refresh does not alter data; it is a
                      // charge-retention operation with no functional
                      // effect as long as the bank being refreshed is
                      // already closed (enforced by ddr5_bank_tracker
                      // on the controller side).

        endcase
      end

      // ---------------- burst execution ----------------
      beat_col = burst_col_base + {{(COL_W-5){1'b0}}, burst_beat};
      if (burst_active) begin
        if (burst_is_write) begin
          if (wr_active) begin
            for (int byte_i = 0; byte_i < DQ_W_LOCAL/8; byte_i++) begin
              if (!wr_dm_n[byte_i])
                row_buf[burst_bank_idx][beat_col][byte_i*8 +: 8] <=
                  wr_data[byte_i*8 +: 8];
            end
            burst_beat <= burst_beat + 1'b1;
            beat_cnt   <= burst_beat + 1'b1;
            if (burst_beat + 1'b1 == burst_len_beats) begin
              burst_active <= 1'b0;
              if (ap) begin
                page_mem[burst_bank_idx][open_row[burst_bank_idx]] <= row_buf[burst_bank_idx];
                row_open[burst_bank_idx] <= 1'b0;
              end
            end
          end
        end else begin
          rd_valid   <= 1'b1;
          rd_data    <= row_buf[burst_bank_idx][beat_col];
          burst_beat <= burst_beat + 1'b1;
          beat_cnt   <= burst_beat + 1'b1;
          if (burst_beat + 1'b1 == burst_len_beats) begin
            rd_last      <= 1'b1;
            burst_active <= 1'b0;
            if (ap) begin
              page_mem[burst_bank_idx][open_row[burst_bank_idx]] <= row_buf[burst_bank_idx];
              row_open[burst_bank_idx] <= 1'b0;
            end
          end
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // Backdoor task for testbenches: preload/inspect a location without
  // going through the command interface. Skips the row buffer entirely
  // so it is safe to call whether or not the target bank is open.
  // ------------------------------------------------------------------
  task automatic backdoor_write(input logic [4:0] bank,
                                 input logic [ROW_W-1:0] r,
                                 input logic [COL_W-1:0] c,
                                 input logic [DQ_W_LOCAL-1:0] data);
    if (!page_mem[bank].exists(r))
      for (int cc = 0; cc < NUM_COLS; cc++) page_mem[bank][r][cc] = '0;
    page_mem[bank][r][c] = data;
    if (row_open[bank] && open_row[bank] == r) row_buf[bank][c] = data;
  endtask

  function automatic logic [DQ_W_LOCAL-1:0] backdoor_read(
      input logic [4:0] bank, input logic [ROW_W-1:0] r, input logic [COL_W-1:0] c);
    if (row_open[bank] && open_row[bank] == r) return row_buf[bank][c];
    if (page_mem[bank].exists(r))              return page_mem[bank][r][c];
    return '0;
  endfunction

endmodule

`endif

