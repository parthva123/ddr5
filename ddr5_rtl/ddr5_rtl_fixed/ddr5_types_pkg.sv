// ============================================================================
// ddr5_types_pkg.sv
// Shared enums / structs for the DDR5-6400 controller RTL.
// ============================================================================
`ifndef DDR5_TYPES_PKG_SV
`define DDR5_TYPES_PKG_SV

package ddr5_types_pkg;

  // 2-cycle CA command opcodes (internal encoding, see ddr5_cmd_encoder)
  typedef enum logic [3:0] {
    CMD_NOP      = 4'h0,
    CMD_ACT      = 4'h1,
    CMD_RD       = 4'h2,
    CMD_WR       = 4'h3,
    CMD_PRE      = 4'h4,
    CMD_PRE_ALL  = 4'h5,
    CMD_REF_AB   = 4'h6,
    CMD_REF_PB   = 4'h7,
    CMD_REF_SB   = 4'h8,
    CMD_MRW      = 4'h9,
    CMD_MRR      = 4'hA,
    CMD_ZQ_START = 4'hB,
    CMD_ZQ_LATCH = 4'hC
  } ddr5_cmd_e;

  // Request descriptor: the single currency passed from the AXI4 side,
  // the refresh manager, and the init FSM, through the scheduler, into a
  // channel controller.
  typedef struct packed {
    ddr5_cmd_e   cmd;
    logic [2:0]  bg;
    logic [1:0]  ba;
    logic [16:0] row;      // ROW_W = 17 (kept as a literal here so this
    logic [9:0]  col;      // package has no dependency on ddr5_pkg)
    logic        ap;       // auto-precharge (A10)
    logic        bc8;      // burst-chop-8   (A12)
    logic [7:0]  mr_addr;
    logic [7:0]  mr_data;
    logic [3:0]  tag;      // AXI ID passthrough
    logic        is_write; // 1 = write data follows, 0 = read
  } ddr5_req_t;

  // Per-bank timing/state snapshot exposed by ddr5_bank_tracker for
  // debug/coverage visibility (not required for functional correctness).
  typedef struct packed {
    logic        open;
    logic [16:0] row;
  } ddr5_bank_state_t;

endpackage

`endif

