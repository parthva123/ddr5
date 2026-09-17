// ============================================================================
// ddr5_mode_registers.sv
// Per-channel Mode Register file (MR0..MR37). Captured whenever the
// command_fsm commits an MRW (mrw_pulse pulses for one cycle, carrying
// the address/data that were on the CA bus). Also decodes the few MR
// bits the datapath needs directly (write/read DBI enables from MR3).
// ============================================================================
`ifndef DDR5_MODE_REGISTERS_SV
`define DDR5_MODE_REGISTERS_SV

`include "ddr5_pkg.sv"

module ddr5_mode_registers
  import ddr5_pkg::*;
(
  input  logic       ck,
  input  logic       rst_n,

  input  logic        mrw_pulse,
  input  logic [7:0]  mr_addr,
  input  logic [7:0]  mr_data,

  output logic [7:0]  mr_array [0:NUM_MR-1],
  output logic         dbi_wr_en,
  output logic         dbi_rd_en,
  output logic         pb_refresh_en    // MR2[0]: 1 = distributed per-bank refresh
);

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < NUM_MR; i++) mr_array[i] <= 8'h00;
    end else if (mrw_pulse && (mr_addr < NUM_MR)) begin
      mr_array[mr_addr] <= mr_data;
    end
  end

  assign dbi_wr_en     = mr_array[3][6];
  assign dbi_rd_en     = mr_array[3][5];
  assign pb_refresh_en = mr_array[2][0];

endmodule

`endif

