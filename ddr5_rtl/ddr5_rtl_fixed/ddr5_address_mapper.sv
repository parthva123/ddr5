// ============================================================================
// ddr5_address_mapper.sv
// Pure-combinational AXI address -> DDR5 field decode.
// Used once per AXI channel (write side and read side each instantiate
// their own copy) inside ddr5_axi4_slave.
// ============================================================================
`ifndef DDR5_ADDRESS_MAPPER_SV
`define DDR5_ADDRESS_MAPPER_SV

`include "ddr5_pkg.sv"

module ddr5_address_mapper
  import ddr5_pkg::*;
(
  input  logic [31:0]      addr,
  output logic [2:0]       bg,
  output logic [1:0]       ba,
  output logic [ROW_W-1:0] row,
  output logic [COL_W-1:0] col,
  output logic             ch_sel,     // 0 = Channel A, 1 = Channel B
  output logic             is_mr,      // Mode-Register access (MRW on write side, MRR on read side)
  output logic [7:0]       mr_index,
  output logic             is_ctrl_cmd,// explicit PRE / PRE_ALL (write side only)
  output logic             ctrl_is_pre_all,
  output logic             ap,         // auto-precharge strap
  output logic             bc8,        // burst-chop-8 strap
  output logic             addr_error  // out of range (DECERR)
);

  assign bg       = addr_to_bg(addr);
  assign ba       = addr_to_ba(addr);
  assign col      = addr_to_col(addr);
  assign row      = addr_to_row(addr);
  assign ch_sel   = addr[CH_SEL_BIT];
  assign is_mr    = addr[MR_SEL_BIT];
  assign mr_index = addr[7:0];
  assign ap       = addr[AP_BIT];
  assign bc8      = addr[BC8_BIT];

  assign is_ctrl_cmd     = addr[CMD_SEL_BIT] && !addr[MR_SEL_BIT];
  assign ctrl_is_pre_all = addr[OP_LSB+1];   // addr[26]: 0=PRE, 1=PRE_ALL

  // MR-space accesses are always legal (they don't touch the DRAM
  // array); ordinary array accesses must fall within the addressable
  // per-channel range.
  assign addr_error = !is_mr && !is_ctrl_cmd && (addr > MAX_DDR5_ADDRESS);

endmodule

`endif

