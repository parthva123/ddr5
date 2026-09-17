// ============================================================================
// ddr5_scheduler.sv
// Per-channel fixed-priority arbiter feeding a single request stream
// into that channel's command_fsm:
//   priority 1 (highest): ddr5_init_fsm  — RESET/ZQCAL/default-MR steps
//   priority 2:            ddr5_refresh_manager — auto REF_AB
//   priority 3 (lowest):   AXI4-originated traffic (request_fifo)
// ============================================================================
`ifndef DDR5_SCHEDULER_SV
`define DDR5_SCHEDULER_SV

`include "ddr5_types_pkg.sv"

module ddr5_scheduler
  import ddr5_types_pkg::*;
(
  input  logic       init_req_valid,
  input  ddr5_req_t  init_req,
  output logic       init_req_ready,

  input  logic       ref_req_valid,
  input  ddr5_req_t  ref_req,
  output logic       ref_req_ready,

  input  logic       axi_req_valid,
  input  ddr5_req_t  axi_req,
  output logic       axi_req_ready,

  output logic       req_valid,
  output ddr5_req_t  req,
  input  logic       req_ready
);

  always_comb begin
    init_req_ready = 1'b0;
    ref_req_ready  = 1'b0;
    axi_req_ready  = 1'b0;
    req_valid      = 1'b0;
    req            = '0;

    if (init_req_valid) begin
      req_valid       = 1'b1;
      req             = init_req;
      init_req_ready  = req_ready;
    end else if (ref_req_valid) begin
      req_valid      = 1'b1;
      req            = ref_req;
      ref_req_ready  = req_ready;
    end else if (axi_req_valid) begin
      req_valid      = 1'b1;
      req            = axi_req;
      axi_req_ready  = req_ready;
    end
  end

endmodule

`endif

