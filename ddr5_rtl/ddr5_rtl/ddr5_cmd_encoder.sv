// ============================================================================
// ddr5_cmd_encoder.sv
// Pure-combinational 2-cycle CA field encoder, following the "2-Cycle
// Command Protocol" table in the course spec (ACT / RD / WR / PRE /
// PRE_ALL / REF_AB / REF_PB / REF_SB / MRW / ZQCAL_START / ZQCAL_LATCH).
//
// CA[13] doubles as the ACT identifier bit on Cycle-0 (1 = ACT, 0 =
// everything else), matching the spec table's "Identifier" column.
// ============================================================================
`ifndef DDR5_CMD_ENCODER_SV
`define DDR5_CMD_ENCODER_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"

module ddr5_cmd_encoder
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  ddr5_cmd_e         cmd,
  input  logic [2:0]        bg,
  input  logic [1:0]        ba,
  input  logic [ROW_W-1:0]  row,
  input  logic [COL_W-1:0]  col,
  input  logic              ap,
  input  logic              bc8,
  input  logic [7:0]        mr_addr,
  input  logic [7:0]        mr_data,
  output logic [13:0]       ca_cycle0,
  output logic [13:0]       ca_cycle1
);

  always_comb begin
    ca_cycle0 = '0;
    ca_cycle1 = '0;
    unique case (cmd)
      CMD_ACT: begin
        ca_cycle0 = {1'b1, row[ROW_W-1:ROW_W-13]};      // CA13=1, upper RA
        ca_cycle1 = {5'b0, row[3:0], bg, ba};            // lower RA + BG/BA
      end
      CMD_RD: begin
        ca_cycle0 = {1'b0, 2'b00, ap, bc8, col};          // col + A10/A12
        ca_cycle1 = {5'b0, bg, ba, 4'h2};                 // BG/BA + opcode=RD
      end
      CMD_WR: begin
        ca_cycle0 = {1'b0, 2'b00, ap, bc8, col};
        ca_cycle1 = {5'b0, bg, ba, 4'h3};                 // opcode=WR
      end
      CMD_PRE: begin
        ca_cycle0 = {1'b0, 3'b000, 1'b1, 8'h04};          // single-bank PRE
        ca_cycle1 = {8'h00, bg, ba};
      end
      CMD_PRE_ALL: begin
        ca_cycle0 = {1'b0, 3'b000, 1'b1, 8'h05};          // all-bank PRE, A10=1
        ca_cycle1 = '0;
      end
      CMD_REF_AB: begin
        ca_cycle0 = {1'b0, 5'b0, 8'h06};
        ca_cycle1 = '0;
      end
      CMD_REF_PB: begin
        ca_cycle0 = {1'b0, 5'b0, 8'h07};
        ca_cycle1 = {8'h00, bg, ba};
      end
      CMD_REF_SB: begin
        ca_cycle0 = {1'b0, 5'b0, 8'h08};
        ca_cycle1 = {8'h00, bg, ba};
      end
      CMD_MRW: begin
        ca_cycle0 = {1'b0, 5'b0, mr_addr};                // MR address
        ca_cycle1 = {6'b0, mr_data};                      // MR data
      end
      CMD_MRR: begin
        ca_cycle0 = {1'b0, 5'b0, mr_addr};                // MR address (read)
        ca_cycle1 = '0;
      end
      CMD_ZQ_START: begin
        ca_cycle0 = {1'b0, 5'b0, 8'h0B};
        ca_cycle1 = '0;
      end
      CMD_ZQ_LATCH: begin
        ca_cycle0 = {1'b0, 5'b0, 8'h0C};
        ca_cycle1 = '0;
      end
      default: begin
        ca_cycle0 = '0;
        ca_cycle1 = '0;
      end
    endcase
  end

endmodule

`endif

