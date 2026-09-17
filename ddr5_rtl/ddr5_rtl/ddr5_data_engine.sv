// ============================================================================
// ddr5_data_engine.sv
// Write-data and read-data engines. Both stream BL (or BL/2 for
// burst-chop-8) 32-bit beats aligned to CWL / CL cycles after the
// WR / RD command's Cycle-1, launched from a one-hot "command issued"
// pulse shifted through a fixed-latency tapped delay line.
// ============================================================================
`ifndef DDR5_DATA_ENGINE_SV
`define DDR5_DATA_ENGINE_SV

`include "ddr5_pkg.sv"

// ----------------------------------------------------------------------
// Write-data engine: pulls beats from the write-data FIFO and drives
// them onto DQ/DM starting CWL cycles after a WR command is issued.
// ----------------------------------------------------------------------
module ddr5_write_data_engine
  import ddr5_pkg::*;
#(
  parameter int LAUNCH_TAP = CWL    // real DDR5-6400 CWL = 36 CK
)(
  input  logic         ck,
  input  logic         rst_n,

  input  logic         wr_issued_pulse,   // 1-cycle pulse when WR command issued
  input  logic         bc8,
  input  logic         dbi_en,            // MR3 write-DBI enable — see note below

  input  logic         wdq_valid,
  input  logic [31:0]  wdq_data,
  input  logic [3:0]   wdq_dm_n,
  output logic         wdq_ready,

  output logic [31:0]  dq_out,
  output logic         dq_oe,
  output logic [3:0]   dm_out            // carries DM when dbi_en=0, or per-byte
                                          // DBI_n (0=inverted) when dbi_en=1 —
                                          // DM and DBI share this same physical
                                          // pin group in real DDR5, selected by
                                          // the Mode Register, so the two
                                          // functions are mutually exclusive
                                          // here exactly as in silicon.
);

  logic [LAUNCH_TAP:0] pipe;
  logic       active;
  logic [4:0] beat_cnt;
  logic [4:0] burst_len;

  // Real per-byte DDR5 DBI: for each of the 4 bytes, invert it (and drive
  // its DBI_n low) when it has more than 4 set bits, else pass it through
  // unmodified with DBI_n high — this is the actual JEDEC DBI rule, not a
  // whole-word approximation.
  logic [31:0] dbi_data;
  logic [3:0]  dbi_n;
  genvar gi;
  generate
    for (gi = 0; gi < 4; gi++) begin : g_dbi_byte
      wire [7:0] byte_in = wdq_data[gi*8 +: 8];
      wire [3:0] popcnt  = byte_in[0]+byte_in[1]+byte_in[2]+byte_in[3]+
                            byte_in[4]+byte_in[5]+byte_in[6]+byte_in[7];
      wire       invert  = (popcnt > 4);
      assign dbi_data[gi*8 +: 8] = invert ? ~byte_in : byte_in;
      assign dbi_n[gi]           = !invert;
    end
  endgenerate

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      pipe <= '0; active <= 1'b0; beat_cnt <= '0; dq_oe <= 1'b0;
    end else begin
      pipe <= {pipe[LAUNCH_TAP-1:0], wr_issued_pulse};

      if (pipe[LAUNCH_TAP-1] && !active) begin
        active    <= 1'b1;
        burst_len <= bc8 ? 5'd8 : 5'd16;
        beat_cnt  <= '0;
      end

      dq_oe <= active && wdq_valid;

      if (active && wdq_valid) begin
        if (dbi_en) begin
          dq_out <= dbi_data;
          dm_out <= dbi_n;
        end else begin
          dq_out <= wdq_data;
          dm_out <= wdq_dm_n;
        end
        beat_cnt <= beat_cnt + 1'b1;
        if (beat_cnt + 1'b1 == burst_len) active <= 1'b0;
      end
    end
  end

  assign wdq_ready = active;

endmodule

// ----------------------------------------------------------------------
// Read-data engine: latches dq_in starting CL cycles after a RD command
// is issued and streams it into the read-data FIFO.
// ----------------------------------------------------------------------
module ddr5_read_data_engine
  import ddr5_pkg::*;
#(
  parameter int LAUNCH_TAP = CL     // real DDR5-6400 CL = 40 CK
)(
  input  logic         ck,
  input  logic         rst_n,

  input  logic         rd_issued_pulse,
  input  logic         bc8,

  input  logic [31:0]  dq_in,

  output logic         rdq_valid,
  output logic [31:0]  rdq_data,
  output logic         rdq_last,

  output logic         dq_valid,       // for functional-coverage / assertions
  output logic [4:0]   dq_beat_cnt
);

  logic [LAUNCH_TAP:0] pipe;
  logic       active;
  logic [4:0] beat_cnt;
  logic [4:0] burst_len;

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      pipe <= '0; active <= 1'b0; beat_cnt <= '0;
      rdq_valid <= 1'b0; rdq_last <= 1'b0; dq_valid <= 1'b0; dq_beat_cnt <= '0;
    end else begin
      pipe <= {pipe[LAUNCH_TAP-1:0], rd_issued_pulse};

      if (pipe[LAUNCH_TAP-1] && !active) begin
        active    <= 1'b1;
        burst_len <= bc8 ? 5'd8 : 5'd16;
        beat_cnt  <= '0;
      end

      rdq_valid <= 1'b0;
      rdq_last  <= 1'b0;
      dq_valid  <= 1'b0;

      if (active) begin
        rdq_valid   <= 1'b1;
        rdq_data    <= dq_in;
        beat_cnt    <= beat_cnt + 1'b1;
        dq_valid    <= 1'b1;
        dq_beat_cnt <= beat_cnt + 1'b1;
        if (beat_cnt + 1'b1 == burst_len) begin
          active   <= 1'b0;
          rdq_last <= 1'b1;
        end
      end
    end
  end

endmodule

`endif

