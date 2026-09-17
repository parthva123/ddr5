// ============================================================================
// ddr5_axi4_slave.sv
// AXI4 target top wrapper. Instantiates the write channel and read
// channel FSMs, and the per-channel-A/B command / write-data / read-data
// CDC FIFOs that cross from aclk into each DDR5 channel's ck domain.
// ============================================================================
`ifndef DDR5_AXI4_SLAVE_SV
`define DDR5_AXI4_SLAVE_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_axi4_write_channel.sv"
`include "ddr5_axi4_read_channel.sv"
`include "ddr5_request_fifo.sv"
`include "ddr5_async_fifo.sv"

module ddr5_axi4_slave
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic         aclk,
  input  logic         aresetn,

  input  logic [31:0]  awaddr, input logic [7:0] awlen, input logic [2:0] awsize,
  input  logic [1:0]   awburst, input logic [3:0] awid, input logic awvalid, output logic awready,
  input  logic [63:0]  wdata, input logic [7:0] wstrb, input logic wlast, input logic wvalid, output logic wready,
  output logic [3:0]   bid, output logic [1:0] bresp, output logic bvalid, input logic bready,
  input  logic [31:0]  araddr, input logic [7:0] arlen, input logic [2:0] arsize,
  input  logic [1:0]   arburst, input logic [3:0] arid, input logic arvalid, output logic arready,
  output logic [63:0]  rdata, output logic [3:0] rid, output logic [1:0] rresp, output logic rlast, output logic rvalid, input logic rready,

  input  logic         ck_a, input logic ck_a_rst_n,
  output logic         a_req_valid, input logic a_req_ready, output ddr5_req_t a_req,
  output logic         a_wdq_valid, output logic [31:0] a_wdq_data, output logic [3:0] a_wdq_dm_n, input logic a_wdq_ready,
  input  logic         a_rdq_valid, input logic [31:0] a_rdq_data, input logic a_rdq_last,
  input  logic [7:0]   a_mr_array [0:NUM_MR-1],

  input  logic         ck_b, input logic ck_b_rst_n,
  output logic         b_req_valid, input logic b_req_ready, output ddr5_req_t b_req,
  output logic         b_wdq_valid, output logic [31:0] b_wdq_data, output logic [3:0] b_wdq_dm_n, input logic b_wdq_ready,
  input  logic         b_rdq_valid, input logic [31:0] b_rdq_data, input logic b_rdq_last,
  input  logic [7:0]   b_mr_array [0:NUM_MR-1]
);

  // --------------------------------------------------------------------
  // Command descriptor FIFOs — write side and read side each push into
  // the *same* per-channel command stream (a channel only ever has one
  // request in flight in this design, see ddr5_channel_ctrl).
  // --------------------------------------------------------------------
  logic wcmdA_en, wcmdA_full; ddr5_req_t wcmdA_req;
  logic wcmdB_en, wcmdB_full; ddr5_req_t wcmdB_req;
  logic rcmdA_en, rcmdA_full; ddr5_req_t rcmdA_req;
  logic rcmdB_en, rcmdB_full; ddr5_req_t rcmdB_req;

  // simple static-priority merge (write over read) into one push port
  // per channel command_fifo — acceptable because ddr5_channel_ctrl only
  // accepts a new request when fully idle, so back-pressure (aXX_full)
  // naturally throttles whichever side loses arbitration.
  logic cmdA_en; ddr5_req_t cmdA_req; logic cmdA_full;
  logic cmdB_en; ddr5_req_t cmdB_req; logic cmdB_full;

  assign cmdA_en  = wcmdA_en ? 1'b1 : rcmdA_en;
  assign cmdA_req = wcmdA_en ? wcmdA_req : rcmdA_req;
  assign wcmdA_full = cmdA_full;
  assign rcmdA_full = cmdA_full || wcmdA_en; // read side stalls if write side wins this cycle

  assign cmdB_en  = wcmdB_en ? 1'b1 : rcmdB_en;
  assign cmdB_req = wcmdB_en ? wcmdB_req : rcmdB_req;
  assign wcmdB_full = cmdB_full;
  assign rcmdB_full = cmdB_full || wcmdB_en;

  logic cmdA_rd_empty, cmdB_rd_empty;

  ddr5_request_fifo #(.DEPTH_LOG2(4)) u_cmd_fifo_a (
    .wr_clk(aclk), .wr_rst_n(aresetn), .wr_en(cmdA_en), .wr_req(cmdA_req), .wr_full(cmdA_full),
    .rd_clk(ck_a), .rd_rst_n(ck_a_rst_n), .rd_en(a_req_valid && a_req_ready), .rd_req(a_req), .rd_empty(cmdA_rd_empty)
  );
  ddr5_request_fifo #(.DEPTH_LOG2(4)) u_cmd_fifo_b (
    .wr_clk(aclk), .wr_rst_n(aresetn), .wr_en(cmdB_en), .wr_req(cmdB_req), .wr_full(cmdB_full),
    .rd_clk(ck_b), .rd_rst_n(ck_b_rst_n), .rd_en(b_req_valid && b_req_ready), .rd_req(b_req), .rd_empty(cmdB_rd_empty)
  );
  assign a_req_valid = !cmdA_rd_empty;
  assign b_req_valid = !cmdB_rd_empty;

  // --------------------------------------------------------------------
  // Write-data FIFOs (32-bit + 4-bit DM, packed as 36 bits)
  // --------------------------------------------------------------------
  logic wdA_en, wdA_full; logic [35:0] wdA_data;
  logic wdB_en, wdB_full; logic [35:0] wdB_data;
  logic [35:0] wdA_rd, wdB_rd;
  logic        wdA_rd_empty, wdB_rd_empty;

  ddr5_async_fifo #(.WIDTH(36), .DEPTH_LOG2(5)) u_wd_fifo_a (
    .wr_clk(aclk), .wr_rst_n(aresetn), .wr_en(wdA_en), .wr_data(wdA_data), .wr_full(wdA_full),
    .rd_clk(ck_a), .rd_rst_n(ck_a_rst_n), .rd_en(a_wdq_ready), .rd_data(wdA_rd), .rd_empty(wdA_rd_empty)
  );
  ddr5_async_fifo #(.WIDTH(36), .DEPTH_LOG2(5)) u_wd_fifo_b (
    .wr_clk(aclk), .wr_rst_n(aresetn), .wr_en(wdB_en), .wr_data(wdB_data), .wr_full(wdB_full),
    .rd_clk(ck_b), .rd_rst_n(ck_b_rst_n), .rd_en(b_wdq_ready), .rd_data(wdB_rd), .rd_empty(wdB_rd_empty)
  );
  assign a_wdq_data = wdA_rd[31:0]; assign a_wdq_dm_n = wdA_rd[35:32]; assign a_wdq_valid = !wdA_rd_empty;
  assign b_wdq_data = wdB_rd[31:0]; assign b_wdq_dm_n = wdB_rd[35:32]; assign b_wdq_valid = !wdB_rd_empty;

  // --------------------------------------------------------------------
  // Read-data FIFOs (32-bit)
  // --------------------------------------------------------------------
  logic rdA_full, rdB_full;
  logic rdA_pop, rdB_pop;
  logic [31:0] rdA_rd, rdB_rd;
  logic        rdA_rd_empty, rdB_rd_empty;

  ddr5_async_fifo #(.WIDTH(32), .DEPTH_LOG2(5)) u_rd_fifo_a (
    .wr_clk(ck_a), .wr_rst_n(ck_a_rst_n), .wr_en(a_rdq_valid), .wr_data(a_rdq_data), .wr_full(rdA_full),
    .rd_clk(aclk), .rd_rst_n(aresetn), .rd_en(rdA_pop), .rd_data(rdA_rd), .rd_empty(rdA_rd_empty)
  );
  ddr5_async_fifo #(.WIDTH(32), .DEPTH_LOG2(5)) u_rd_fifo_b (
    .wr_clk(ck_b), .wr_rst_n(ck_b_rst_n), .wr_en(b_rdq_valid), .wr_data(b_rdq_data), .wr_full(rdB_full),
    .rd_clk(aclk), .rd_rst_n(aresetn), .rd_en(rdB_pop), .rd_data(rdB_rd), .rd_empty(rdB_rd_empty)
  );

  ddr5_axi4_write_channel u_wch (
    .aclk(aclk), .aresetn(aresetn),
    .awaddr(awaddr), .awlen(awlen), .awid(awid), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .a_cmd_wr_en(wcmdA_en), .a_cmd_full(wcmdA_full), .a_cmd_req(wcmdA_req),
    .a_wd_wr_en(wdA_en), .a_wd_full(wdA_full), .a_wd_data(wdA_data),
    .b_cmd_wr_en(wcmdB_en), .b_cmd_full(wcmdB_full), .b_cmd_req(wcmdB_req),
    .b_wd_wr_en(wdB_en), .b_wd_full(wdB_full), .b_wd_data(wdB_data)
  );

  ddr5_axi4_read_channel u_rch (
    .aclk(aclk), .aresetn(aresetn),
    .araddr(araddr), .arlen(arlen), .arid(arid), .arvalid(arvalid), .arready(arready),
    .rdata(rdata), .rid(rid), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready),
    .a_cmd_wr_en(rcmdA_en), .a_cmd_full(rcmdA_full), .a_cmd_req(rcmdA_req),
    .a_rd_pop(rdA_pop), .a_rd_empty(rdA_rd_empty), .a_rd_data(rdA_rd), .a_mr_array(a_mr_array),
    .b_cmd_wr_en(rcmdB_en), .b_cmd_full(rcmdB_full), .b_cmd_req(rcmdB_req),
    .b_rd_pop(rdB_pop), .b_rd_empty(rdB_rd_empty), .b_rd_data(rdB_rd), .b_mr_array(b_mr_array)
  );

endmodule

`endif

