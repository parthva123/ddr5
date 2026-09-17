// ============================================================================
// ddr5_pkg.sv
// DDR5-6400 timing parameters, array geometry, and the address-map
// functions used by ddr5_address_mapper.
// ============================================================================
`ifndef DDR5_PKG_SV
`define DDR5_PKG_SV

package ddr5_pkg;

  // Timing (CK cycles @ 3200 MHz, tCK = 312.5 ps) — from the DDR5-6400
  // timing table in the course spec.
  parameter int TRCD      = 44;
  parameter int TRP       = 44;
  parameter int TRAS      = 102;
  parameter int TRC       = 146;
  parameter int TCCD_L    = 5;
  parameter int TCCD_S    = 2;
  parameter int TRRD_L    = 5;
  parameter int TRRD_S    = 2;
  parameter int TWR       = 96;
  parameter int TRFC_AB   = 944;
  parameter int TRFC_PB   = 192;
  parameter int TREFI     = 12480;
  parameter int TCKE      = 3;
  parameter int TXP       = 7;
  parameter int TCKESR    = 3;
  parameter int TXSR      = 640;
  parameter int TMRD      = 16;
  parameter int TZQCAL    = 3200;
  parameter int TXPR      = 512;   // internal: CKE-low hold after RESET# deassert
  parameter int CWL       = 36;
  parameter int CL        = 40;
  parameter int BL        = 16;

  parameter int NUM_BG    = 8;
  parameter int NUM_BA    = 4;
  parameter int NUM_BANKS = NUM_BG * NUM_BA;   // 32
  parameter int ROW_W     = 17;
  parameter int COL_W     = 10;
  parameter int NUM_MR    = 38;                // MR0..MR37

  // --------------------------------------------------------------------
  // Address map (this design's authoritative, self-consistent decode —
  // the course PDF's own mapping text is internally inconsistent, e.g.
  // it lists BA as 3 bits which can't be right for 4 banks/group):
  //
  //   addr[2:0]    -> BG   (8 bank groups)
  //   addr[4:3]    -> BA   (4 banks per group)
  //   addr[14:5]   -> COL  (10-bit column / word index)
  //   addr[31:15]  -> ROW  (17-bit row)
  //   addr[27]     -> channel select (0 = Ch-A, 1 = Ch-B)
  //   addr[28]     -> explicit control command (PRE/PRE_ALL), ignored if
  //                    addr[31]=1; addr[26:25] then selects: 00=PRE
  //                    (single bank, uses BG/BA fields), 01=PRE_ALL
  //   addr[29]     -> burst-chop-8 strap
  //   addr[30]     -> auto-precharge strap
  //   addr[31]     -> Mode-Register access: on the WRITE channel this is
  //                    an MRW (WDATA[7:0] = MR data); on the READ
  //                    channel this is an MRR (RDATA[7:0] = MR value)
  //
  // Note: the ROW field (addr[31:15]) mathematically overlaps the strap
  // bits above (25-31 all fall inside it) — by design, for a
  // verification-oriented controller this is harmless (those bits just
  // ride along as part of the stored row value), but it does mean the
  // *usable* independent row range is addr[24:15] (10 bits).
  // --------------------------------------------------------------------
  parameter int CH_SEL_BIT  = 27;
  parameter int CMD_SEL_BIT = 28;
  parameter int OP_LSB      = 25;   // addr[26:25]: 00=PRE, 01=PRE_ALL
  parameter int BC8_BIT     = 29;
  parameter int AP_BIT      = 30;
  parameter int MR_SEL_BIT  = 31;
  parameter logic [31:0] MAX_DDR5_ADDRESS = 32'h07FF_FFFF;

  function automatic logic [2:0]  addr_to_bg (input logic [31:0] a); return a[2:0];   endfunction
  function automatic logic [1:0]  addr_to_ba (input logic [31:0] a); return a[4:3];   endfunction
  function automatic logic [COL_W-1:0] addr_to_col(input logic [31:0] a); return a[14:5]; endfunction
  function automatic logic [ROW_W-1:0] addr_to_row(input logic [31:0] a); return a[31:15]; endfunction
  function automatic logic [4:0]  bank_idx(input logic [2:0] bg, input logic [1:0] ba); return {bg, ba}; endfunction

endpackage

`endif

