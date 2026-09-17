// ============================================================================
// tb_ddr5_top.sv
//
// Minimal harness: instantiates ddr5_top and drops a ddr5_device_model on
// each of its two channels (A and B), driven off ddr5_top's decoded
// command bus + debug taps (a_cmd_*/a_dbg_*, b_cmd_*/b_dbg_*) rather than
// the raw CA pins, so a testbench can drive the AXI4 side and get real
// read data back. Fill in your own AXI driver / scoreboard around this --
// this file only shows the DUT<->model wiring.
//
// Rev 2: fixed to match ddr5_top's actual port list (single a_dq_in, the
// real a_cmd_pre/a_cmd_pre_all/a_cmd_ref_pb names, no PRE_PB/SB/AB or
// MPC/PDE/NOP outputs) and dropped a .P_TINIT1/3/4(...) parameter
// override -- ddr5_top takes no parameters.
// ============================================================================
`include "ddr5_top.sv"
`include "ddr5_device_model.sv"

module tb_ddr5_top;

  // ---- clocks / reset ----
  logic aclk = 0, ck_a_p = 0, ck_b_p = 0, ck_a_n, ck_b_n;
  logic aresetn = 0, por_reset_n = 0;

  always #1.5625  aclk   = ~aclk;    // AXI clock, arbitrary for the example
  always #0.15625 ck_a_p = ~ck_a_p;  // DDR5-6400: tCK(avg) = 0.3125 ns
  always #0.15625 ck_b_p = ~ck_b_p;
  assign ck_a_n = ~ck_a_p;
  assign ck_b_n = ~ck_b_p;

  initial begin
    por_reset_n = 0; aresetn = 0;
    #100 por_reset_n = 1;
    #10  aresetn     = 1;
  end

  // ---- AXI4 stimulus signals (drive these from your test sequence) ----
  logic [31:0] awaddr, araddr; logic [7:0] awlen, arlen; logic [2:0] awsize, arsize;
  logic [1:0]  awburst, arburst; logic [3:0] awid, arid;
  logic        awvalid, awready, arvalid, arready;
  logic [63:0] wdata; logic [7:0] wstrb; logic wlast, wvalid, wready;
  logic [3:0]  bid; logic [1:0] bresp; logic bvalid, bready;
  logic [63:0] rdata; logic [3:0] rid; logic [1:0] rresp; logic rlast, rvalid, rready;

  // ---- channel A pins / decoded command bus ----
  logic        a_reset_n, a_cke, a_cs_n; logic [13:0] a_ca;
  wire  [31:0] a_dq; logic [3:0] a_dm_n; logic [31:0] a_dq_in;
  logic a_cmd_act, a_cmd_rd, a_cmd_wr, a_cmd_pre, a_cmd_pre_all;
  logic a_cmd_ref_ab, a_cmd_ref_pb, a_cmd_mrw, a_cmd_mrr, a_cmd_zq_start, a_cmd_zq_latch;
  logic [2:0] a_cmd_bg; logic [1:0] a_cmd_ba;
  logic a_dq_valid; logic [4:0] a_dq_beat_cnt;
  logic [16:0] a_dbg_row; logic [9:0] a_dbg_col; logic a_dbg_ap, a_dbg_bc8;

  // ---- channel B pins / decoded command bus ----
  logic        b_reset_n, b_cke, b_cs_n; logic [13:0] b_ca;
  wire  [31:0] b_dq; logic [3:0] b_dm_n; logic [31:0] b_dq_in;
  logic b_cmd_act, b_cmd_rd, b_cmd_wr, b_cmd_pre, b_cmd_pre_all;
  logic b_cmd_ref_ab, b_cmd_ref_pb, b_cmd_mrw, b_cmd_mrr, b_cmd_zq_start, b_cmd_zq_latch;
  logic [2:0] b_cmd_bg; logic [1:0] b_cmd_ba;
  logic b_dq_valid; logic [4:0] b_dq_beat_cnt;
  logic [16:0] b_dbg_row; logic [9:0] b_dbg_col; logic b_dbg_ap, b_dbg_bc8;

  ddr5_top dut (
    .aclk(aclk), .aresetn(aresetn),
    .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
    .awid(awid), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
    .arid(arid), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rid(rid), .rresp(rresp), .rlast(rlast),
    .rvalid(rvalid), .rready(rready),

    .ck_a_p(ck_a_p), .ck_a_n(ck_a_n), .por_reset_n(por_reset_n),
    .a_reset_n(a_reset_n), .a_cke(a_cke), .a_cs_n(a_cs_n), .a_ca(a_ca),
    .a_dq(a_dq), .a_dm_n(a_dm_n), .a_dq_in(a_dq_in),
    .a_cmd_act(a_cmd_act), .a_cmd_rd(a_cmd_rd), .a_cmd_wr(a_cmd_wr),
    .a_cmd_pre(a_cmd_pre), .a_cmd_pre_all(a_cmd_pre_all),
    .a_cmd_ref_ab(a_cmd_ref_ab), .a_cmd_ref_pb(a_cmd_ref_pb),
    .a_cmd_mrw(a_cmd_mrw), .a_cmd_mrr(a_cmd_mrr),
    .a_cmd_zq_start(a_cmd_zq_start), .a_cmd_zq_latch(a_cmd_zq_latch),
    .a_cmd_bg(a_cmd_bg), .a_cmd_ba(a_cmd_ba),
    .a_dq_valid(a_dq_valid), .a_dq_beat_cnt(a_dq_beat_cnt),
    .a_dbg_row(a_dbg_row), .a_dbg_col(a_dbg_col), .a_dbg_ap(a_dbg_ap), .a_dbg_bc8(a_dbg_bc8),

    .ck_b_p(ck_b_p), .ck_b_n(ck_b_n),
    .b_reset_n(b_reset_n), .b_cke(b_cke), .b_cs_n(b_cs_n), .b_ca(b_ca),
    .b_dq(b_dq), .b_dm_n(b_dm_n), .b_dq_in(b_dq_in),
    .b_cmd_act(b_cmd_act), .b_cmd_rd(b_cmd_rd), .b_cmd_wr(b_cmd_wr),
    .b_cmd_pre(b_cmd_pre), .b_cmd_pre_all(b_cmd_pre_all),
    .b_cmd_ref_ab(b_cmd_ref_ab), .b_cmd_ref_pb(b_cmd_ref_pb),
    .b_cmd_mrw(b_cmd_mrw), .b_cmd_mrr(b_cmd_mrr),
    .b_cmd_zq_start(b_cmd_zq_start), .b_cmd_zq_latch(b_cmd_zq_latch),
    .b_cmd_bg(b_cmd_bg), .b_cmd_ba(b_cmd_ba),
    .b_dq_valid(b_dq_valid), .b_dq_beat_cnt(b_dq_beat_cnt),
    .b_dbg_row(b_dbg_row), .b_dbg_col(b_dbg_col), .b_dbg_ap(b_dbg_ap), .b_dbg_bc8(b_dbg_bc8)
  );

  // ---- device models: one per channel, driven off the decoded command
  //      bus + debug taps (not the raw CA pins) ----
  ddr5_device_model u_dev_a (
    .ck(ck_a_p), .reset_n(a_reset_n),
    .cmd_act(a_cmd_act), .cmd_rd(a_cmd_rd), .cmd_wr(a_cmd_wr),
    .cmd_pre(a_cmd_pre), .cmd_pre_all(a_cmd_pre_all),
    .cmd_bg(a_cmd_bg), .cmd_ba(a_cmd_ba),
    .dbg_row(a_dbg_row), .dbg_col(a_dbg_col), .dbg_ap(a_dbg_ap), .dbg_bc8(a_dbg_bc8),
    .dq(a_dq), .dm_n(a_dm_n), .dq_in(a_dq_in)
  );

  ddr5_device_model u_dev_b (
    .ck(ck_b_p), .reset_n(b_reset_n),
    .cmd_act(b_cmd_act), .cmd_rd(b_cmd_rd), .cmd_wr(b_cmd_wr),
    .cmd_pre(b_cmd_pre), .cmd_pre_all(b_cmd_pre_all),
    .cmd_bg(b_cmd_bg), .cmd_ba(b_cmd_ba),
    .dbg_row(b_dbg_row), .dbg_col(b_dbg_col), .dbg_ap(b_dbg_ap), .dbg_bc8(b_dbg_bc8),
    .dq(b_dq), .dm_n(b_dm_n), .dq_in(b_dq_in)
  );

  // Fill in your AXI4 write/read sequences here, e.g.:
  // initial begin
  //   wait (aresetn);
  //   // ... drive awaddr/awvalid, wdata/wvalid, then araddr/arvalid ...
  // end

endmodule
