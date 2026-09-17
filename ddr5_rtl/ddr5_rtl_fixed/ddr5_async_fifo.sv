// ============================================================================
// ddr5_async_fifo.sv
// Generic dual-clock (Gray-code pointer) asynchronous FIFO.
//
// Used by the DDR5 controller to cross:
//   - command descriptors : aclk (AXI4)  -> ck (DDR5 channel)
//   - write data           : aclk (AXI4)  -> ck (DDR5 channel)
//   - read data             : ck (DDR5 channel) -> aclk (AXI4)
//
// Standard 2-flop synchronised Gray-pointer CDC FIFO, safe for one
// writer / one reader crossing independent, unrelated clocks.
// ============================================================================
`ifndef DDR5_ASYNC_FIFO_SV
`define DDR5_ASYNC_FIFO_SV

module ddr5_async_fifo #(
  parameter int WIDTH = 32,
  parameter int DEPTH_LOG2 = 4          // depth = 2**DEPTH_LOG2
)(
  // write side
  input  logic                  wr_clk,
  input  logic                  wr_rst_n,
  input  logic                  wr_en,
  input  logic [WIDTH-1:0]      wr_data,
  output logic                  wr_full,

  // read side
  input  logic                  rd_clk,
  input  logic                  rd_rst_n,
  input  logic                  rd_en,
  output logic [WIDTH-1:0]      rd_data,
  output logic                  rd_empty
);

  localparam int DEPTH = 1 << DEPTH_LOG2;

  logic [WIDTH-1:0] mem [0:DEPTH-1];

  // binary + gray pointers, one extra MSB for full/empty disambiguation
  logic [DEPTH_LOG2:0] wr_bin, wr_bin_next, wr_gray, wr_gray_next;
  logic [DEPTH_LOG2:0] rd_bin, rd_bin_next, rd_gray, rd_gray_next;

  logic [DEPTH_LOG2:0] wr_gray_rd_sync1, wr_gray_rd_sync2; // wr gray, synced into rd domain
  logic [DEPTH_LOG2:0] rd_gray_wr_sync1, rd_gray_wr_sync2; // rd gray, synced into wr domain

  function automatic logic [DEPTH_LOG2:0] bin2gray(input logic [DEPTH_LOG2:0] b);
    return (b >> 1) ^ b;
  endfunction

  // ---------------- write domain ----------------
  assign wr_bin_next  = wr_bin + (wr_en && !wr_full);
  assign wr_gray_next = bin2gray(wr_bin_next);

  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_bin  <= '0;
      wr_gray <= '0;
    end else begin
      wr_bin  <= wr_bin_next;
      wr_gray <= wr_gray_next;
    end
  end

  always_ff @(posedge wr_clk) begin
    if (wr_en && !wr_full) mem[wr_bin[DEPTH_LOG2-1:0]] <= wr_data;
  end

  assign wr_full = (wr_gray_next == {~rd_gray_wr_sync2[DEPTH_LOG2:DEPTH_LOG2-1],
                                       rd_gray_wr_sync2[DEPTH_LOG2-2:0]});

  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rd_gray_wr_sync1 <= '0;
      rd_gray_wr_sync2 <= '0;
    end else begin
      rd_gray_wr_sync1 <= rd_gray;
      rd_gray_wr_sync2 <= rd_gray_wr_sync1;
    end
  end

  // ---------------- read domain ----------------
  assign rd_bin_next  = rd_bin + (rd_en && !rd_empty);
  assign rd_gray_next = bin2gray(rd_bin_next);

  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_bin  <= '0;
      rd_gray <= '0;
    end else begin
      rd_bin  <= rd_bin_next;
      rd_gray <= rd_gray_next;
    end
  end

  assign rd_data  = mem[rd_bin[DEPTH_LOG2-1:0]];
  assign rd_empty = (rd_gray_next == wr_gray_rd_sync2);

  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wr_gray_rd_sync1 <= '0;
      wr_gray_rd_sync2 <= '0;
    end else begin
      wr_gray_rd_sync1 <= wr_gray;
      wr_gray_rd_sync2 <= wr_gray_rd_sync1;
    end
  end

endmodule

`endif

