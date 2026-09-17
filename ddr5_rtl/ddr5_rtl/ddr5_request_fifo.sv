// ============================================================================
// ddr5_request_fifo.sv
// Clock-domain-crossing FIFO specialised for ddr5_req_t, built on top of
// the generic ddr5_async_fifo primitive. One instance per direction per
// channel is used to move request descriptors from the AXI4 (aclk)
// domain into a channel's DDR5 (ck) domain.
// ============================================================================
`ifndef DDR5_REQUEST_FIFO_SV
`define DDR5_REQUEST_FIFO_SV

`include "ddr5_types_pkg.sv"
`include "ddr5_async_fifo.sv"

module ddr5_request_fifo
  import ddr5_types_pkg::*;
#(
  parameter int DEPTH_LOG2 = 4
)(
  input  logic       wr_clk,
  input  logic       wr_rst_n,
  input  logic       wr_en,
  input  ddr5_req_t  wr_req,
  output logic       wr_full,

  input  logic       rd_clk,
  input  logic       rd_rst_n,
  input  logic       rd_en,
  output ddr5_req_t  rd_req,
  output logic       rd_empty
);

  localparam int W = $bits(ddr5_req_t);

  logic [W-1:0] rd_data;

  ddr5_async_fifo #(.WIDTH(W), .DEPTH_LOG2(DEPTH_LOG2)) u_fifo (
    .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(wr_en), .wr_data(wr_req), .wr_full(wr_full),
    .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .rd_en(rd_en), .rd_data(rd_data), .rd_empty(rd_empty)
  );

  assign rd_req = ddr5_req_t'(rd_data);

endmodule

`endif

