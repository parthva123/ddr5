// ============================================================================
// ddr5_channel_ctrl.sv
// One DDR5-6400 sub-channel's command_fsm, wired to:
//   ddr5_bank_tracker   — legality + timing state
//   ddr5_cmd_encoder    — 2-cycle CA field generation
//   ddr5_dfi_if         — pin-level register stage
//   ddr5_write/read_data_engine — DQ datapath
//
// A request arriving as CMD_ACT auto-expands into ACT followed by the
// matching RD or WR (chosen by req.is_write) once tRCD is satisfied,
// so the scheduler/AXI4 side only ever issues one descriptor per
// transaction. CMD_MRW / CMD_REF_AB / CMD_ZQ_* requests are single-shot.
// ============================================================================
`ifndef DDR5_CHANNEL_CTRL_SV
`define DDR5_CHANNEL_CTRL_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_bank_tracker.sv"
`include "ddr5_cmd_encoder.sv"
`include "ddr5_dfi_if.sv"
`include "ddr5_data_engine.sv"

module ddr5_channel_ctrl
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic         ck,
  input  logic         por_reset_n,
  input  logic         ctrl_ready,     // from ddr5_init_fsm
  input  logic         pwr_active,     // from ddr5_power_fsm — gates new request acceptance
  output logic         chan_idle,      // to ddr5_power_fsm — no in-flight/pending command

  // ---- request interface from ddr5_scheduler ----
  input  logic         req_valid,
  output logic         req_ready,
  input  ddr5_req_t    req,

  // ---- write-data pipe (from write-data request_fifo) ----
  input  logic         wdq_valid,
  input  logic [31:0]  wdq_data,
  input  logic [3:0]   wdq_dm_n,
  output logic         wdq_ready,

  // ---- read-data pipe (to read-data request_fifo) ----
  output logic         rdq_valid,
  output logic [31:0]  rdq_data,
  output logic         rdq_last,

  // ---- Mode-Register file interface ----
  output logic         mrw_pulse,
  output logic [7:0]   mr_addr,
  output logic [7:0]   mr_data,
  input  logic         dbi_wr_en,
  input  logic         dbi_rd_en,

  // ---- pin-level (through ddr5_dfi_if / ddr5_phy_wrapper) ----
  input  logic         reset_n_in,     // from ddr5_init_fsm
  input  logic         cke_in,         // from ddr5_init_fsm
  output logic         reset_n,
  output logic         cke,
  output logic         cs_n,
  output logic [13:0]  ca,
  inout  wire  [31:0]  dq,
  output logic [3:0]   dm_n,
  input  logic [31:0]  dq_in,

  // ---- decoded command bus (for SVA / functional coverage bind) ----
  output logic         cmd_act, cmd_rd, cmd_wr, cmd_pre, cmd_pre_all,
  output logic         cmd_ref_ab, cmd_ref_pb, cmd_mrw_pulse_dbg, cmd_mrr,
  output logic         cmd_zq_start, cmd_zq_latch,
  output logic [2:0]   cmd_bg,
  output logic [1:0]   cmd_ba,
  output logic         dq_valid,
  output logic [4:0]   dq_beat_cnt,

  // ---- debug taps (verification-only): row/col/ap/bc8 that were sent
  // to the encoder for the command currently pulsing on cmd_act/cmd_rd/
  // cmd_wr above. Valid on the same cycle as those pulses, held stable
  // from CYC_IDLE through CYC_1 so they line up with cmd_bg/cmd_ba too.
  // Added so a behavioral memory/device model can reconstruct a full
  // command without re-deriving ddr5_cmd_encoder's CA bit packing. ----
  output logic [ROW_W-1:0] dbg_row,
  output logic [COL_W-1:0] dbg_col,
  output logic              dbg_ap,
  output logic              dbg_bc8
);

  assign dbg_row = act_row;
  assign dbg_col = act_col;
  assign dbg_ap  = act_ap;
  assign dbg_bc8 = act_bc8;

  // ------------------------------------------------------------------
  // Bank tracker (legality + timing state)
  // ------------------------------------------------------------------
  logic       q_legal;
  ddr5_cmd_e  q_cmd_w;
  logic [2:0] q_bg_w;
  logic [1:0] q_ba_w;

  logic       commit_pulse;
  ddr5_cmd_e  c_cmd;
  logic [2:0] c_bg;
  logic [1:0] c_ba;
  logic [ROW_W-1:0] c_row;
  logic       c_ap;

  ddr5_bank_tracker u_bank_tracker (
    .ck(ck), .rst_n(por_reset_n), .ctrl_ready(ctrl_ready),
    .q_cmd(q_cmd_w), .q_bg(q_bg_w), .q_ba(q_ba_w), .legal(q_legal),
    .commit_pulse(commit_pulse), .c_cmd(c_cmd), .c_bg(c_bg), .c_ba(c_ba), .c_row(c_row), .c_ap(c_ap),
    .rfc_ab_busy()
  );

  // ------------------------------------------------------------------
  // Command encoder (combinational)
  // ------------------------------------------------------------------
  logic [13:0] enc_ca0, enc_ca1;
  ddr5_cmd_e   act_cmd;
  logic [2:0]  act_bg;
  logic [1:0]  act_ba;
  logic [ROW_W-1:0] act_row;
  logic [COL_W-1:0] act_col;
  logic        act_ap, act_bc8;
  logic [7:0]  act_mr_addr, act_mr_data;

  ddr5_cmd_encoder u_encoder (
    .cmd(act_cmd), .bg(act_bg), .ba(act_ba), .row(act_row), .col(act_col),
    .ap(act_ap), .bc8(act_bc8), .mr_addr(act_mr_addr), .mr_data(act_mr_data),
    .ca_cycle0(enc_ca0), .ca_cycle1(enc_ca1)
  );

  // ------------------------------------------------------------------
  // Pin-level DFI adapter
  // ------------------------------------------------------------------
  logic cyc0_en, cyc1_en, cs_active;

  ddr5_dfi_if u_dfi (
    .ck(ck), .rst_n(por_reset_n),
    .cyc0_en(cyc0_en), .ca0(enc_ca0), .cyc1_en(cyc1_en), .ca1(enc_ca1), .cs_active(cs_active),
    .reset_n_in(reset_n_in), .cke_in(cke_in),
    .reset_n(reset_n), .cke(cke), .cs_n(cs_n), .ca(ca)
  );

  // ------------------------------------------------------------------
  // Write / read data engines
  // ------------------------------------------------------------------
  logic wr_issued_pulse, rd_issued_pulse;
  logic [31:0] dq_out_eng; logic dq_oe_eng; logic [3:0] dm_out_eng;

  ddr5_write_data_engine u_wr_eng (
    .ck(ck), .rst_n(por_reset_n),
    .wr_issued_pulse(wr_issued_pulse), .bc8(act_bc8), .dbi_en(dbi_wr_en),
    .wdq_valid(wdq_valid), .wdq_data(wdq_data), .wdq_dm_n(wdq_dm_n), .wdq_ready(wdq_ready),
    .dq_out(dq_out_eng), .dq_oe(dq_oe_eng), .dm_out(dm_out_eng)
  );

  logic [31:0] rdq_data_eng;
  logic [31:0] dq_in_dbi;
  assign dq_in_dbi = dbi_rd_en ? ~dq_in : dq_in; // simplified DBI polarity

  ddr5_read_data_engine u_rd_eng (
    .ck(ck), .rst_n(por_reset_n),
    .rd_issued_pulse(rd_issued_pulse), .bc8(act_bc8),
    .dq_in(dq_in_dbi),
    .rdq_valid(rdq_valid), .rdq_data(rdq_data), .rdq_last(rdq_last),
    .dq_valid(dq_valid), .dq_beat_cnt(dq_beat_cnt)
  );

  assign dq   = dq_oe_eng ? dq_out_eng : 32'bz;
  assign dm_n = dq_oe_eng ? ~dm_out_eng : 4'b1111;

  // ------------------------------------------------------------------
  // command_fsm: 2-cycle dispatch sequencer + ACT->RD/WR auto follow-up
  // ------------------------------------------------------------------
  typedef enum logic [1:0] {CYC_IDLE, CYC_0, CYC_1} dispatch_e;
  dispatch_e dispatch_st;

  logic       pend_valid;
  logic       pend_is_write;
  logic [2:0] pend_bg; logic [1:0] pend_ba;
  logic [ROW_W-1:0] pend_row; logic [COL_W-1:0] pend_col;
  logic       pend_ap, pend_bc8;

  // What are we proposing to bank_tracker *this* cycle, for the legality
  // query, before we know whether it will actually be accepted?
  always_comb begin
    if (pend_valid) begin
      if (pend_is_write) q_cmd_w = CMD_WR;
      else                q_cmd_w = CMD_RD;
      q_bg_w  = pend_bg;
      q_ba_w  = pend_ba;
    end else begin
      q_cmd_w = req.cmd;
      q_bg_w  = req.bg;
      q_ba_w  = req.ba;
    end
  end

  assign req_ready = ctrl_ready && pwr_active && (dispatch_st == CYC_IDLE) && !pend_valid && req_valid && q_legal;

  wire pend_grant = ctrl_ready && pwr_active && (dispatch_st == CYC_IDLE) && pend_valid && q_legal;

  assign chan_idle = (dispatch_st == CYC_IDLE) && !pend_valid && !req_valid;

  // cyc0_en / cyc1_en / cs_active are combinational (Mealy) outputs of the
  // *current* dispatch_st: ddr5_dfi_if provides the actual register
  // stage onto the pins, so these must not be pre-registered here too
  // (doing so would shift CS#/CA by an extra cycle and break the
  // required 2-consecutive-cycle CS# window).
  assign cyc0_en   = (dispatch_st == CYC_0);
  assign cyc1_en   = (dispatch_st == CYC_1);
  assign cs_active = (dispatch_st == CYC_0) || (dispatch_st == CYC_1);

  always_ff @(posedge ck or negedge por_reset_n) begin
    if (!por_reset_n) begin
      dispatch_st <= CYC_IDLE;
      pend_valid  <= 1'b0;
      commit_pulse <= 1'b0;
      {cmd_act,cmd_rd,cmd_wr,cmd_pre,cmd_pre_all,cmd_ref_ab,cmd_ref_pb,
       cmd_mrw_pulse_dbg,cmd_mrr,cmd_zq_start,cmd_zq_latch,mrw_pulse,wr_issued_pulse,rd_issued_pulse} <= '0;
      cmd_bg <= '0; cmd_ba <= '0;
    end else begin
      commit_pulse <= 1'b0;
      {cmd_act,cmd_rd,cmd_wr,cmd_pre,cmd_pre_all,cmd_ref_ab,cmd_ref_pb,
       cmd_mrw_pulse_dbg,cmd_mrr,cmd_zq_start,cmd_zq_latch,mrw_pulse,wr_issued_pulse,rd_issued_pulse} <= '0;

      unique case (dispatch_st)
        CYC_IDLE: begin
          if (pend_grant) begin
            if (pend_is_write) act_cmd <= CMD_WR;
            else                act_cmd <= CMD_RD;
            act_bg  <= pend_bg; act_ba <= pend_ba;
            act_row <= pend_row; act_col <= pend_col;
            act_ap  <= pend_ap; act_bc8 <= pend_bc8;
            dispatch_st <= CYC_0;
          end else if (req_ready) begin
            act_cmd     <= req.cmd;
            act_bg      <= req.bg;  act_ba  <= req.ba;
            act_row     <= req.row; act_col <= req.col;
            act_ap      <= req.ap;  act_bc8 <= req.bc8;
            act_mr_addr <= req.mr_addr; act_mr_data <= req.mr_data;
            if (req.cmd == CMD_ACT) begin
              pend_valid    <= 1'b1;
              pend_is_write <= req.is_write;
              pend_bg <= req.bg; pend_ba <= req.ba;
              pend_row <= req.row; pend_col <= req.col;
              pend_ap  <= req.ap; pend_bc8 <= req.bc8;
            end
            dispatch_st <= CYC_0;
          end
        end

        CYC_0: begin
          dispatch_st <= CYC_1;
        end

        CYC_1: begin
          cmd_bg  <= act_bg;
          cmd_ba  <= act_ba;

          commit_pulse <= 1'b1;
          c_cmd <= act_cmd; c_bg <= act_bg; c_ba <= act_ba; c_row <= act_row; c_ap <= act_ap;

          unique case (act_cmd)
            CMD_ACT:     cmd_act     <= 1'b1;
            CMD_RD:      begin cmd_rd <= 1'b1; rd_issued_pulse <= 1'b1;
                                if (pend_valid && !pend_is_write) pend_valid <= 1'b0; end
            CMD_WR:      begin cmd_wr <= 1'b1; wr_issued_pulse <= 1'b1;
                                if (pend_valid && pend_is_write) pend_valid <= 1'b0; end
            CMD_PRE:     cmd_pre     <= 1'b1;
            CMD_PRE_ALL: cmd_pre_all <= 1'b1;
            CMD_REF_AB:  cmd_ref_ab  <= 1'b1;
            CMD_REF_PB:  cmd_ref_pb  <= 1'b1;
            CMD_MRW:     begin
                           cmd_mrw_pulse_dbg <= 1'b1;
                           mrw_pulse <= 1'b1;
                           mr_addr   <= act_mr_addr;
                           mr_data   <= act_mr_data;
                         end
            CMD_MRR:     cmd_mrr     <= 1'b1;
            CMD_ZQ_START: cmd_zq_start <= 1'b1;
            CMD_ZQ_LATCH: cmd_zq_latch <= 1'b1;
            default: ;
          endcase
          dispatch_st <= CYC_IDLE;
        end
        default: dispatch_st <= CYC_IDLE;
      endcase
    end
  end

endmodule

`endif

