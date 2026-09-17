// ============================================================================
// ddr5_bank_tracker.sv
// Per-channel 32-bank (8BG x 4BA) open/closed state and timing-parameter
// tracker. Combines the roles of a bank-state tracker and a timing
// checker: it both remembers which rows are open and enforces
// tRCD/tRP/tRAS/tCCD_L/tCCD_S/tRRD_L/tRRD_S/tRFC_AB/tRFC_PB/tMRD by
// gating `legal` for whatever command the command_fsm is proposing.
// ============================================================================
`ifndef DDR5_BANK_TRACKER_SV
`define DDR5_BANK_TRACKER_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"

module ddr5_bank_tracker
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic       ck,
  input  logic       rst_n,
  input  logic       ctrl_ready,

  // legality query — combinational, valid the same cycle as cmd/bg/ba
  input  ddr5_cmd_e  q_cmd,
  input  logic [2:0] q_bg,
  input  logic [1:0] q_ba,
  output logic       legal,

  // commit — pulsed by command_fsm on the cycle it actually issues the
  // command (Cycle-1 of the 2-cycle protocol)
  input  logic       commit_pulse,
  input  ddr5_cmd_e  c_cmd,
  input  logic [2:0] c_bg,
  input  logic [1:0] c_ba,
  input  logic [ROW_W-1:0] c_row,
  input  logic       c_ap,

  output logic       rfc_ab_busy   // all-bank refresh recovery in progress
);

  logic             bank_open   [NUM_BANKS];
  logic [ROW_W-1:0] bank_row    [NUM_BANKS];
  int               act_timer   [NUM_BANKS];
  int               pre_timer   [NUM_BANKS];
  int               rfc_pb_timer[NUM_BANKS];
  int               rc_timer    [NUM_BANKS];   // tRC: time since last ACT on this bank

  int ccd_l_timer, ccd_s_timer, rrd_l_timer, rrd_s_timer, rfc_ab_timer, mrd_timer;

  logic [4:0] q_idx;
  assign q_idx = bank_idx(q_bg, q_ba);

  logic [4:0] cidx;
  assign cidx = bank_idx(c_bg, c_ba);

  assign rfc_ab_busy = (rfc_ab_timer != 0);

  always_comb begin
    legal = 1'b0;
    if (ctrl_ready) begin
      unique case (q_cmd)
        CMD_ACT:     legal = !bank_open[q_idx] && (pre_timer[q_idx] == 0) &&
                              (rc_timer[q_idx] == 0) &&
                              (rrd_l_timer == 0) && (rrd_s_timer == 0) &&
                              (mrd_timer == 0) && (rfc_ab_timer == 0);
        CMD_RD, CMD_WR:
                     legal = bank_open[q_idx] &&
                              (act_timer[q_idx] <= (TRAS - TRCD)) &&
                              (ccd_l_timer == 0) && (ccd_s_timer == 0) &&
                              (mrd_timer == 0);
        CMD_PRE:     legal = bank_open[q_idx] && (act_timer[q_idx] == 0);
        CMD_PRE_ALL: legal = 1'b1;
        CMD_REF_AB:  legal = (rfc_ab_timer == 0);
        CMD_REF_PB:  legal = (rfc_pb_timer[q_idx] == 0) && !bank_open[q_idx];
        CMD_MRW:     legal = (mrd_timer == 0);
        CMD_MRR:     legal = (mrd_timer == 0);
        CMD_ZQ_START, CMD_ZQ_LATCH: legal = 1'b1;
        default:     legal = 1'b0;
      endcase
    end
  end

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      ccd_l_timer <= 0; ccd_s_timer <= 0; rrd_l_timer <= 0; rrd_s_timer <= 0;
      rfc_ab_timer <= 0; mrd_timer <= 0;
      for (int b = 0; b < NUM_BANKS; b++) begin
        bank_open[b] <= 1'b0; bank_row[b] <= '0;
        act_timer[b] <= 0; pre_timer[b] <= 0; rfc_pb_timer[b] <= 0; rc_timer[b] <= 0;
      end
    end else begin
      // decrement all outstanding timers
      if (ccd_l_timer  > 0) ccd_l_timer  <= ccd_l_timer  - 1;
      if (ccd_s_timer  > 0) ccd_s_timer  <= ccd_s_timer  - 1;
      if (rrd_l_timer  > 0) rrd_l_timer  <= rrd_l_timer  - 1;
      if (rrd_s_timer  > 0) rrd_s_timer  <= rrd_s_timer  - 1;
      if (rfc_ab_timer > 0) rfc_ab_timer <= rfc_ab_timer - 1;
      if (mrd_timer    > 0) mrd_timer    <= mrd_timer    - 1;
      for (int b = 0; b < NUM_BANKS; b++) begin
        if (act_timer[b] > 0) act_timer[b] <= act_timer[b] - 1;
        if (pre_timer[b] > 0) pre_timer[b] <= pre_timer[b] - 1;
        if (rfc_pb_timer[b] > 0) rfc_pb_timer[b] <= rfc_pb_timer[b] - 1;
        if (rc_timer[b] > 0) rc_timer[b] <= rc_timer[b] - 1;
      end

      if (commit_pulse) begin
        unique case (c_cmd)
          CMD_ACT: begin
            bank_open[cidx] <= 1'b1;
            bank_row[cidx]  <= c_row;
            act_timer[cidx] <= TRAS;
            rrd_l_timer <= TRRD_L;
            rrd_s_timer <= TRRD_S;
          end
          CMD_RD: begin
            ccd_l_timer <= TCCD_L; ccd_s_timer <= TCCD_S;
            if (c_ap) begin
              pre_timer[cidx] <= TRP;
              bank_open[cidx] <= 1'b0;
            end
          end
          CMD_WR: begin
            ccd_l_timer <= TCCD_L; ccd_s_timer <= TCCD_S;
            if (c_ap) begin
              pre_timer[cidx] <= TRP + TWR;
              bank_open[cidx] <= 1'b0;
            end
          end
          CMD_PRE: begin
            bank_open[cidx] <= 1'b0;
            pre_timer[cidx] <= TRP;
          end
          CMD_PRE_ALL: begin
            for (int b = 0; b < NUM_BANKS; b++) begin
              bank_open[b] <= 1'b0;
              pre_timer[b] <= TRP;
            end
          end
          CMD_REF_AB: rfc_ab_timer <= TRFC_AB;
          CMD_REF_PB: rfc_pb_timer[cidx] <= TRFC_PB;
          CMD_MRW:    mrd_timer <= TMRD;
          CMD_MRR:    mrd_timer <= TMRD;
          default: ;
        endcase
      end
    end
  end

endmodule

`endif

