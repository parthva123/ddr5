// ============================================================================
// ddr5_dfi_if.sv
// DFI-style adapter: registers the command_fsm's Cycle-0/Cycle-1 CA
// words and cs_active window onto the actual DDR5 pins, and passes
// reset_n/cke through from the init FSM. This is the boundary between
// "controller logic" (clocked on the abstract command timeline) and
// "pin-level" (what ddr5_if in the testbench observes).
//
// ddr5_phy_wrapper is an intentionally trivial passthrough — a real
// DFI/PHY would add write/read training, DQ (de)serialisation and DQS
// generation, all of which are analog/SERDES concerns outside the scope
// of this digital controller RTL.
// ============================================================================
`ifndef DDR5_DFI_IF_SV
`define DDR5_DFI_IF_SV

module ddr5_dfi_if (
  input  logic         ck,
  input  logic         rst_n,

  input  logic         cyc0_en,
  input  logic [13:0]  ca0,
  input  logic         cyc1_en,
  input  logic [13:0]  ca1,
  input  logic         cs_active,   // held for both cycles of an active command

  input  logic         reset_n_in,
  input  logic         cke_in,

  output logic         reset_n,
  output logic         cke,
  output logic         cs_n,
  output logic [13:0]  ca
);

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      cs_n    <= 1'b1;
      ca      <= '0;
      reset_n <= 1'b0;
      cke     <= 1'b0;
    end else begin
      reset_n <= reset_n_in;
      cke     <= cke_in;
      cs_n    <= !cs_active;
      if (cyc0_en)      ca <= ca0;
      else if (cyc1_en) ca <= ca1;
    end
  end

endmodule

// ----------------------------------------------------------------------
// optional_phy_wrapper — passthrough stub for a future analog PHY.
// ----------------------------------------------------------------------
module ddr5_phy_wrapper (
  input  wire  [31:0] ctrl_dq_out,
  input  logic         ctrl_dq_oe,
  output wire  [31:0] pin_dq,

  input  wire  [31:0] pin_dq_sample,
  output logic [31:0] ctrl_dq_in
);

  assign pin_dq     = ctrl_dq_oe ? ctrl_dq_out : 32'bz;
  assign ctrl_dq_in = pin_dq_sample;

endmodule

`endif

