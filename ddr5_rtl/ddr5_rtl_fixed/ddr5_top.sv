// ============================================================================
// ddr5_top.sv
// DDR5-6400 Dual-Channel / Single-Rank AXI4 Controller — top level.
//
//   ddr5_top
//   +-- axi4_slave (write_channel, read_channel, address_mapper x2, CDC FIFOs)
//   +-- per channel (A, B):
//         +-- scheduler        (init > refresh > axi priority arbiter)
//         +-- refresh_manager  (tREFI tracking, auto REF_AB)
//         +-- init_fsm         (JEDEC reset/init sequence, ctrl_ready)
//         +-- mode_registers   (MR0..MR37 file, DBI enables)
//         +-- channel_ctrl     (command_fsm + bank_tracker + cmd_encoder
//                                + dfi_if + write/read data engines)
// ============================================================================
`ifndef DDR5_TOP_SV
`define DDR5_TOP_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_axi4_slave.sv"
`include "ddr5_scheduler.sv"
`include "ddr5_refresh_manager.sv"
`include "ddr5_init_fsm.sv"
`include "ddr5_mode_registers.sv"
`include "ddr5_power_fsm.sv"
`include "ddr5_channel_ctrl.sv"

module ddr5_top
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic         aclk,
  input  logic         aresetn,

  input  logic [31:0]  awaddr,  input logic [7:0] awlen, input logic [2:0] awsize,
  input  logic [1:0]   awburst, input logic [3:0] awid,  input logic       awvalid, output logic awready,
  input  logic [63:0]  wdata,   input logic [7:0] wstrb, input logic       wlast,   input logic  wvalid,  output logic wready,
  output logic [3:0]   bid,     output logic [1:0] bresp, output logic     bvalid,  input logic  bready,
  input  logic [31:0]  araddr,  input logic [7:0] arlen, input logic [2:0] arsize,
  input  logic [1:0]   arburst, input logic [3:0] arid,  input logic       arvalid, output logic arready,
  output logic [63:0]  rdata,   output logic [3:0] rid,   output logic [1:0] rresp, output logic rlast, output logic rvalid, input logic rready,

  // Channel A pins
  input  logic         ck_a_p, input logic ck_a_n, input logic por_reset_n,
  output logic         a_reset_n, output logic a_cke, output logic a_cs_n, output logic [13:0] a_ca,
  inout  wire  [31:0]  a_dq, output logic [3:0] a_dm_n, input logic [31:0] a_dq_in,
  output logic a_cmd_act, output logic a_cmd_rd, output logic a_cmd_wr, output logic a_cmd_pre, output logic a_cmd_pre_all,
  output logic a_cmd_ref_ab, output logic a_cmd_ref_pb, output logic a_cmd_mrw, output logic a_cmd_mrr,
  output logic a_cmd_zq_start, output logic a_cmd_zq_latch,
  output logic [2:0] a_cmd_bg, output logic [1:0] a_cmd_ba, output logic a_dq_valid, output logic [4:0] a_dq_beat_cnt,
  // Debug taps for a behavioral device/memory model (verification-only,
  // no effect on the DUT's own functionality): the row/col/ap/bc8 that
  // accompany the a_cmd_act/a_cmd_rd/a_cmd_wr pulses above.
  output logic [ROW_W-1:0] a_dbg_row, output logic [COL_W-1:0] a_dbg_col,
  output logic              a_dbg_ap,  output logic              a_dbg_bc8,

  // Channel B pins
  input  logic         ck_b_p, input logic ck_b_n,
  output logic         b_reset_n, output logic b_cke, output logic b_cs_n, output logic [13:0] b_ca,
  inout  wire  [31:0]  b_dq, output logic [3:0] b_dm_n, input logic [31:0] b_dq_in,
  output logic b_cmd_act, output logic b_cmd_rd, output logic b_cmd_wr, output logic b_cmd_pre, output logic b_cmd_pre_all,
  output logic b_cmd_ref_ab, output logic b_cmd_ref_pb, output logic b_cmd_mrw, output logic b_cmd_mrr,
  output logic b_cmd_zq_start, output logic b_cmd_zq_latch,
  output logic [2:0] b_cmd_bg, output logic [1:0] b_cmd_ba, output logic b_dq_valid, output logic [4:0] b_dq_beat_cnt,
  output logic [ROW_W-1:0] b_dbg_row, output logic [COL_W-1:0] b_dbg_col,
  output logic              b_dbg_ap,  output logic              b_dbg_bc8
);

  // ------------------------------------------------------------------
  // AXI4 slave <-> per-channel command / write-data / read-data
  // ------------------------------------------------------------------
  logic a_req_valid, a_req_ready; ddr5_req_t a_axi_req;
  logic a_wdq_valid; logic [31:0] a_wdq_data; logic [3:0] a_wdq_dm_n; logic a_wdq_ready;
  logic a_rdq_valid; logic [31:0] a_rdq_data; logic a_rdq_last;

  logic b_req_valid, b_req_ready; ddr5_req_t b_axi_req;
  logic b_wdq_valid; logic [31:0] b_wdq_data; logic [3:0] b_wdq_dm_n; logic b_wdq_ready;
  logic b_rdq_valid; logic [31:0] b_rdq_data; logic b_rdq_last;

  logic [7:0] a_mr_array [0:NUM_MR-1];
  logic [7:0] b_mr_array [0:NUM_MR-1];

  ddr5_axi4_slave u_axi4_slave (
    .aclk(aclk), .aresetn(aresetn),
    .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst), .awid(awid), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst), .arid(arid), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rid(rid), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready),

    .ck_a(ck_a_p), .ck_a_rst_n(por_reset_n),
    .a_req_valid(a_req_valid), .a_req_ready(a_req_ready), .a_req(a_axi_req),
    .a_wdq_valid(a_wdq_valid), .a_wdq_data(a_wdq_data), .a_wdq_dm_n(a_wdq_dm_n), .a_wdq_ready(a_wdq_ready),
    .a_rdq_valid(a_rdq_valid), .a_rdq_data(a_rdq_data), .a_rdq_last(a_rdq_last),
    .a_mr_array(a_mr_array),

    .ck_b(ck_b_p), .ck_b_rst_n(por_reset_n),
    .b_req_valid(b_req_valid), .b_req_ready(b_req_ready), .b_req(b_axi_req),
    .b_wdq_valid(b_wdq_valid), .b_wdq_data(b_wdq_data), .b_wdq_dm_n(b_wdq_dm_n), .b_wdq_ready(b_wdq_ready),
    .b_rdq_valid(b_rdq_valid), .b_rdq_data(b_rdq_data), .b_rdq_last(b_rdq_last),
    .b_mr_array(b_mr_array)
  );

  // ------------------------------------------------------------------
  // Channel A: init_fsm + refresh_manager + scheduler + mode_registers + channel_ctrl
  // ------------------------------------------------------------------
  logic a_ctrl_ready;
  logic a_init_reset_n, a_init_cke;
  logic a_init_req_valid, a_init_req_ready; ddr5_req_t a_init_req;
  logic a_ref_req_valid, a_ref_req_ready;   ddr5_req_t a_ref_req;
  logic a_sched_req_valid, a_sched_req_ready; ddr5_req_t a_sched_req;
  logic a_mrw_pulse; logic [7:0] a_mr_addr, a_mr_data;
  logic a_dbi_wr_en, a_dbi_rd_en, a_pb_refresh_en;

  ddr5_init_fsm u_init_a (
    .ck(ck_a_p), .por_reset_n(por_reset_n),
    .reset_n(a_init_reset_n), .cke(a_init_cke),
    .req_valid(a_init_req_valid), .req(a_init_req), .req_ready(a_init_req_ready),
    .ctrl_ready(a_ctrl_ready)
  );

  ddr5_refresh_manager u_refresh_a (
    .ck(ck_a_p), .rst_n(por_reset_n), .enable(a_ctrl_ready), .pb_mode(a_pb_refresh_en),
    .req_valid(a_ref_req_valid), .req(a_ref_req), .req_ready(a_ref_req_ready)
  );

  ddr5_scheduler u_sched_a (
    .init_req_valid(a_init_req_valid), .init_req(a_init_req), .init_req_ready(a_init_req_ready),
    .ref_req_valid(a_ref_req_valid),   .ref_req(a_ref_req),   .ref_req_ready(a_ref_req_ready),
    .axi_req_valid(a_req_valid),       .axi_req(a_axi_req),   .axi_req_ready(a_req_ready),
    .req_valid(a_sched_req_valid), .req(a_sched_req), .req_ready(a_sched_req_ready)
  );

  ddr5_mode_registers u_mr_a (
    .ck(ck_a_p), .rst_n(por_reset_n),
    .mrw_pulse(a_mrw_pulse), .mr_addr(a_mr_addr), .mr_data(a_mr_data),
    .mr_array(a_mr_array), .dbi_wr_en(a_dbi_wr_en), .dbi_rd_en(a_dbi_rd_en), .pb_refresh_en(a_pb_refresh_en)
  );

  logic a_pwr_cke, a_pwr_active, a_chan_idle, a_cke_in;

  ddr5_power_fsm u_pwr_a (
    .ck(ck_a_p), .rst_n(por_reset_n), .ctrl_ready(a_ctrl_ready), .chan_idle(a_chan_idle),
    .cke(a_pwr_cke), .pwr_active(a_pwr_active)
  );

  // before init completes, CKE is owned by init_fsm; afterwards, by power_fsm
  assign a_cke_in = a_ctrl_ready ? a_pwr_cke : a_init_cke;

  ddr5_channel_ctrl u_ch_a (
    .ck(ck_a_p), .por_reset_n(por_reset_n), .ctrl_ready(a_ctrl_ready),
    .pwr_active(a_pwr_active), .chan_idle(a_chan_idle),
    .req_valid(a_sched_req_valid), .req_ready(a_sched_req_ready), .req(a_sched_req),
    .wdq_valid(a_wdq_valid), .wdq_data(a_wdq_data), .wdq_dm_n(a_wdq_dm_n), .wdq_ready(a_wdq_ready),
    .rdq_valid(a_rdq_valid), .rdq_data(a_rdq_data), .rdq_last(a_rdq_last),
    .mrw_pulse(a_mrw_pulse), .mr_addr(a_mr_addr), .mr_data(a_mr_data),
    .dbi_wr_en(a_dbi_wr_en), .dbi_rd_en(a_dbi_rd_en),
    .reset_n_in(a_init_reset_n), .cke_in(a_cke_in),
    .reset_n(a_reset_n), .cke(a_cke), .cs_n(a_cs_n), .ca(a_ca),
    .dq(a_dq), .dm_n(a_dm_n), .dq_in(a_dq_in),
    .cmd_act(a_cmd_act), .cmd_rd(a_cmd_rd), .cmd_wr(a_cmd_wr), .cmd_pre(a_cmd_pre), .cmd_pre_all(a_cmd_pre_all),
    .cmd_ref_ab(a_cmd_ref_ab), .cmd_ref_pb(a_cmd_ref_pb), .cmd_mrw_pulse_dbg(a_cmd_mrw), .cmd_mrr(a_cmd_mrr),
    .cmd_zq_start(a_cmd_zq_start), .cmd_zq_latch(a_cmd_zq_latch),
    .cmd_bg(a_cmd_bg), .cmd_ba(a_cmd_ba), .dq_valid(a_dq_valid), .dq_beat_cnt(a_dq_beat_cnt),
    .dbg_row(a_dbg_row), .dbg_col(a_dbg_col), .dbg_ap(a_dbg_ap), .dbg_bc8(a_dbg_bc8)
  );

  // ------------------------------------------------------------------
  // Channel B: identical structure
  // ------------------------------------------------------------------
  logic b_ctrl_ready;
  logic b_init_reset_n, b_init_cke;
  logic b_init_req_valid, b_init_req_ready; ddr5_req_t b_init_req;
  logic b_ref_req_valid, b_ref_req_ready;   ddr5_req_t b_ref_req;
  logic b_sched_req_valid, b_sched_req_ready; ddr5_req_t b_sched_req;
  logic b_mrw_pulse; logic [7:0] b_mr_addr, b_mr_data;
  logic b_dbi_wr_en, b_dbi_rd_en, b_pb_refresh_en;

  ddr5_init_fsm u_init_b (
    .ck(ck_b_p), .por_reset_n(por_reset_n),
    .reset_n(b_init_reset_n), .cke(b_init_cke),
    .req_valid(b_init_req_valid), .req(b_init_req), .req_ready(b_init_req_ready),
    .ctrl_ready(b_ctrl_ready)
  );

  ddr5_refresh_manager u_refresh_b (
    .ck(ck_b_p), .rst_n(por_reset_n), .enable(b_ctrl_ready), .pb_mode(b_pb_refresh_en),
    .req_valid(b_ref_req_valid), .req(b_ref_req), .req_ready(b_ref_req_ready)
  );

  ddr5_scheduler u_sched_b (
    .init_req_valid(b_init_req_valid), .init_req(b_init_req), .init_req_ready(b_init_req_ready),
    .ref_req_valid(b_ref_req_valid),   .ref_req(b_ref_req),   .ref_req_ready(b_ref_req_ready),
    .axi_req_valid(b_req_valid),       .axi_req(b_axi_req),   .axi_req_ready(b_req_ready),
    .req_valid(b_sched_req_valid), .req(b_sched_req), .req_ready(b_sched_req_ready)
  );

  ddr5_mode_registers u_mr_b (
    .ck(ck_b_p), .rst_n(por_reset_n),
    .mrw_pulse(b_mrw_pulse), .mr_addr(b_mr_addr), .mr_data(b_mr_data),
    .mr_array(b_mr_array), .dbi_wr_en(b_dbi_wr_en), .dbi_rd_en(b_dbi_rd_en), .pb_refresh_en(b_pb_refresh_en)
  );

  logic b_pwr_cke, b_pwr_active, b_chan_idle, b_cke_in;

  ddr5_power_fsm u_pwr_b (
    .ck(ck_b_p), .rst_n(por_reset_n), .ctrl_ready(b_ctrl_ready), .chan_idle(b_chan_idle),
    .cke(b_pwr_cke), .pwr_active(b_pwr_active)
  );

  assign b_cke_in = b_ctrl_ready ? b_pwr_cke : b_init_cke;

  ddr5_channel_ctrl u_ch_b (
    .ck(ck_b_p), .por_reset_n(por_reset_n), .ctrl_ready(b_ctrl_ready),
    .pwr_active(b_pwr_active), .chan_idle(b_chan_idle),
    .req_valid(b_sched_req_valid), .req_ready(b_sched_req_ready), .req(b_sched_req),
    .wdq_valid(b_wdq_valid), .wdq_data(b_wdq_data), .wdq_dm_n(b_wdq_dm_n), .wdq_ready(b_wdq_ready),
    .rdq_valid(b_rdq_valid), .rdq_data(b_rdq_data), .rdq_last(b_rdq_last),
    .mrw_pulse(b_mrw_pulse), .mr_addr(b_mr_addr), .mr_data(b_mr_data),
    .dbi_wr_en(b_dbi_wr_en), .dbi_rd_en(b_dbi_rd_en),
    .reset_n_in(b_init_reset_n), .cke_in(b_cke_in),
    .reset_n(b_reset_n), .cke(b_cke), .cs_n(b_cs_n), .ca(b_ca),
    .dq(b_dq), .dm_n(b_dm_n), .dq_in(b_dq_in),
    .cmd_act(b_cmd_act), .cmd_rd(b_cmd_rd), .cmd_wr(b_cmd_wr), .cmd_pre(b_cmd_pre), .cmd_pre_all(b_cmd_pre_all),
    .cmd_ref_ab(b_cmd_ref_ab), .cmd_ref_pb(b_cmd_ref_pb), .cmd_mrw_pulse_dbg(b_cmd_mrw), .cmd_mrr(b_cmd_mrr),
    .cmd_zq_start(b_cmd_zq_start), .cmd_zq_latch(b_cmd_zq_latch),
    .cmd_bg(b_cmd_bg), .cmd_ba(b_cmd_ba), .dq_valid(b_dq_valid), .dq_beat_cnt(b_dq_beat_cnt),
    .dbg_row(b_dbg_row), .dbg_col(b_dbg_col), .dbg_ap(b_dbg_ap), .dbg_bc8(b_dbg_bc8)
  );

endmodule

`endif

