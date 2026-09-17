// ============================================================================
// ddr5_device_model.sv
//
// Sits on one channel's decoded command bus + data pins, as driven by
// ddr5_top (a_cmd_* / a_dbg_* / a_dq / a_dm_n / a_dq_in, or the b_
// equivalents), and drives ddr5_bank_mem so the controller has something
// real to talk to in simulation.
//
// Rev 2: earlier revision of this model decoded the raw CA bus itself,
// using a generic JEDEC-style command truth table. That did not match
// this project's actual ddr5_cmd_encoder.sv encoding (CA[13] as the ACT
// identifier, literal opcode bytes for everything else) and referenced
// ddr5_cmd_e values (CMD_PRE_PB/SB/AB, CMD_MPC, CMD_PDE, CMD_SRE) that
// don't exist in ddr5_types_pkg. This revision instead consumes the
// already-decoded per-channel command bus ddr5_channel_ctrl exposes
// (cmd_act/cmd_rd/cmd_wr/... + the dbg_row/dbg_col/dbg_ap/dbg_bc8 taps),
// so it can't drift out of sync with the real encoder, and only uses
// ddr5_cmd_e members that are actually defined.
//
// TIMING: ddr5_write_data_engine / ddr5_read_data_engine launch their
// DQ bursts CWL / CL cycles after the WR/RD command pulse (see
// ddr5_data_engine.sv), not on the command cycle itself. This model
// mirrors that with its own CWL/CL tapped delay lines so write data is
// sampled, and read data is presented, on the same cycles the real
// engines drive/expect them.
//
// ASSUMPTIONS (functional model, not pin-accurate):
//  * Single rank, one device per channel.
//  * Burst length follows dbg_bc8 exactly as ddr5_data_engine computes
//    it (bc8 ? 8 : 16 beats), matching CWL/CL launch timing.
//  * Assumes wdq_valid never stalls a write burst once launched (i.e.
//    write data is always ready by CWL) -- a reasonable simplification
//    for a functional/verification model, not a full DFI stall model.
//  * MR reads/writes are not modeled here (ddr5_mode_registers already
//    holds MR state on the controller side); this model only concerns
//    itself with the DRAM array (ACT/RD/WR/PRE/PRE_ALL).
// ============================================================================
`ifndef DDR5_DEVICE_MODEL_SV
`define DDR5_DEVICE_MODEL_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_bank_mem.sv"

module ddr5_device_model
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic              ck,
  input  logic               reset_n,      // from a_reset_n / b_reset_n

  // Decoded command bus, straight from ddr5_top's a_cmd_* / a_dbg_*
  // (or b_ equivalents) -- one pulse per issued command, valid the same
  // cycle as its matching cmd_bg/cmd_ba/dbg_row/dbg_col/dbg_ap/dbg_bc8.
  input  logic               cmd_act,
  input  logic               cmd_rd,
  input  logic               cmd_wr,
  input  logic               cmd_pre,
  input  logic               cmd_pre_all,
  input  logic [2:0]         cmd_bg,
  input  logic [1:0]         cmd_ba,
  input  logic [ROW_W-1:0]   dbg_row,
  input  logic [COL_W-1:0]   dbg_col,
  input  logic                dbg_ap,
  input  logic                dbg_bc8,

  inout  wire  [DQ_W-1:0]    dq,           // a_dq / b_dq (controller drives on WR)
  input  logic [3:0]         dm_n,         // a_dm_n / b_dm_n (active-low, matches
                                            // ddr5_channel_ctrl's dm_n output and
                                            // ddr5_bank_mem's wr_dm_n input)
  output logic [DQ_W-1:0]    dq_in         // -> a_dq_in / b_dq_in
);

  // ------------------------------------------------------------------
  // Immediate (undelayed) commands into ddr5_bank_mem: ACT / PRE /
  // PRE_ALL take effect on the same cycle the controller issues them --
  // no CWL/CL-style launch delay applies to these in the real protocol.
  // ------------------------------------------------------------------
  logic       imm_valid;
  ddr5_cmd_e  imm_cmd;

  always_comb begin
    imm_valid = cmd_act || cmd_pre || cmd_pre_all;
    unique case (1'b1)
      cmd_act:     imm_cmd = CMD_ACT;
      cmd_pre:     imm_cmd = CMD_PRE;
      cmd_pre_all: imm_cmd = CMD_PRE_ALL;
      default:     imm_cmd = CMD_NOP;
    endcase
  end

  // ------------------------------------------------------------------
  // Write launch: mirrors ddr5_write_data_engine's tapped delay line
  // (LAUNCH_TAP = CWL), driving burst_len consecutive beats of wr_active
  // into ddr5_bank_mem starting CWL cycles after cmd_wr.
  // ------------------------------------------------------------------
  logic [CWL:0] wr_pipe;
  logic         wr_active_q;
  logic [4:0]   wr_beat_cnt;
  logic [4:0]   wr_burst_len;
  logic [ROW_W-1:0] wr_row_q; logic [COL_W-1:0] wr_col_q;
  logic [2:0]  wr_bg_q; logic [1:0] wr_ba_q; logic wr_ap_q;

  always_ff @(posedge ck or negedge reset_n) begin
    if (!reset_n) begin
      wr_pipe <= '0; wr_active_q <= 1'b0; wr_beat_cnt <= '0;
    end else begin
      wr_pipe <= {wr_pipe[CWL-1:0], cmd_wr};

      if (cmd_wr) begin
        wr_bg_q <= cmd_bg; wr_ba_q <= cmd_ba;
        wr_row_q <= dbg_row; wr_col_q <= dbg_col; wr_ap_q <= dbg_ap;
        wr_burst_len <= dbg_bc8 ? 5'd8 : 5'd16;
      end

      if (wr_pipe[CWL-1] && !wr_active_q) begin
        wr_active_q <= 1'b1;
        wr_beat_cnt <= '0;
      end else if (wr_active_q) begin
        wr_beat_cnt <= wr_beat_cnt + 1'b1;
        if (wr_beat_cnt + 1'b1 == wr_burst_len) wr_active_q <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------
  // Read launch: mirrors ddr5_read_data_engine's tapped delay line
  // (LAUNCH_TAP = CL), presenting burst_len consecutive beats of read
  // data starting CL cycles after cmd_rd -- exactly when
  // ddr5_read_data_engine will be sampling a_dq_in / b_dq_in.
  // ------------------------------------------------------------------
  logic [CL:0] rd_pipe;
  logic        rd_active_q;
  logic [4:0]  rd_beat_cnt;
  logic [4:0]  rd_burst_len;
  logic [ROW_W-1:0] rd_row_q; logic [COL_W-1:0] rd_col_q;
  logic [2:0]  rd_bg_q; logic [1:0] rd_ba_q; logic rd_ap_q;

  always_ff @(posedge ck or negedge reset_n) begin
    if (!reset_n) begin
      rd_pipe <= '0; rd_active_q <= 1'b0; rd_beat_cnt <= '0;
    end else begin
      rd_pipe <= {rd_pipe[CL-1:0], cmd_rd};

      if (cmd_rd) begin
        rd_bg_q <= cmd_bg; rd_ba_q <= cmd_ba;
        rd_row_q <= dbg_row; rd_col_q <= dbg_col; rd_ap_q <= dbg_ap;
        rd_burst_len <= dbg_bc8 ? 5'd8 : 5'd16;
      end

      if (rd_pipe[CL-1] && !rd_active_q) begin
        rd_active_q <= 1'b1;
        rd_beat_cnt <= '0;
      end else if (rd_active_q) begin
        rd_beat_cnt <= rd_beat_cnt + 1'b1;
        if (rd_beat_cnt + 1'b1 == rd_burst_len) rd_active_q <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------
  // ddr5_bank_mem instance. RD and WR bursts never overlap in practice
  // (bank_tracker enforces tCCD/turnaround on the controller side), so
  // it's safe to mux the launched bg/ba/row/col/ap onto one command
  // port: RD's launch wins ties (WR only asserts cmd_valid via bank_mem
  // internally through the CMD_WR case, RD through CMD_RD).
  // ------------------------------------------------------------------
  logic       cmd_valid;
  ddr5_cmd_e  cmd_sel;
  logic [2:0] bg_sel; logic [1:0] ba_sel;
  logic [ROW_W-1:0] row_sel; logic [COL_W-1:0] col_sel; logic ap_sel;

  wire wr_launch = wr_pipe[CWL-1] && !wr_active_q;  // first beat this cycle
  wire rd_launch = rd_pipe[CL-1]  && !rd_active_q;  // first beat this cycle

  always_comb begin
    if (imm_valid) begin
      cmd_valid = 1'b1;  cmd_sel = imm_cmd;
      bg_sel = cmd_bg;   ba_sel = cmd_ba;
      row_sel = dbg_row; col_sel = dbg_col; ap_sel = dbg_ap;
    end else if (wr_launch) begin
      cmd_valid = 1'b1;  cmd_sel = CMD_WR;
      bg_sel = wr_bg_q;  ba_sel = wr_ba_q;
      row_sel = wr_row_q; col_sel = wr_col_q; ap_sel = wr_ap_q;
    end else if (rd_launch) begin
      cmd_valid = 1'b1;  cmd_sel = CMD_RD;
      bg_sel = rd_bg_q;  ba_sel = rd_ba_q;
      row_sel = rd_row_q; col_sel = rd_col_q; ap_sel = rd_ap_q;
    end else begin
      cmd_valid = 1'b0;  cmd_sel = CMD_NOP;
      bg_sel = '0; ba_sel = '0; row_sel = '0; col_sel = '0; ap_sel = '0;
    end
  end

  logic rd_valid, rd_last;
  logic [DQ_W-1:0] rd_data;
  logic [5:0] beat_cnt;

  logic [4:0] blen_sel;
  always_comb begin
    if (wr_launch)      blen_sel = wr_burst_len;
    else if (rd_launch) blen_sel = rd_burst_len;
    else                blen_sel = 5'd0;
  end

  ddr5_bank_mem #(.DQ_W_LOCAL(DQ_W)) u_bank_mem (
    .ck(ck), .rst_n(reset_n),
    .cmd_valid(cmd_valid), .cmd(cmd_sel),
    .bg(bg_sel), .ba(ba_sel), .row(row_sel), .col(col_sel), .ap(ap_sel),
    .burst_len(blen_sel),
    .wr_active(wr_active_q), .wr_data(dq), .wr_dm_n(dm_n),
    .rd_valid(rd_valid), .rd_data(rd_data), .rd_last(rd_last),
    .beat_cnt(beat_cnt)
  );

  // Device never drives the bidirectional a_dq/b_dq pin in this model
  // (the controller is the only driver, on writes); read data goes
  // straight to dq_in, matching how ddr5_top / ddr5_read_data_engine
  // consume it (dq_in is sampled unconditionally every active cycle,
  // so it must be valid for the full CL-delayed read window).
  assign dq = {DQ_W{1'bz}};

  always_ff @(posedge ck or negedge reset_n) begin
    if (!reset_n) dq_in <= '0;
    else if (rd_valid) dq_in <= rd_data;
  end

endmodule

`endif
