// ============================================================================
// ddr5_refresh_manager.sv
// Tracks the refresh interval for one channel and raises a refresh
// request when due. Two modes, selected by `pb_mode` (wired from a
// Mode-Register bit — see ddr5_mode_registers):
//   pb_mode = 0 (default): classic all-bank refresh, one REF_AB every
//                            tREFI.
//   pb_mode = 1:            distributed per-bank refresh — one REF_PB
//                            is issued every tREFI/NUM_BANKS cycles,
//                            round-robining through all 32 banks so the
//                            whole array is still refreshed once per
//                            tREFI window overall. (Real DDR5 per-bank
//                            refresh timing (tREFIpb) is a separate,
//                            tighter JEDEC parameter than a flat
//                            tREFI/32 split — this is a documented
//                            simplification, not the exact JEDEC value.)
// ============================================================================
`ifndef DDR5_REFRESH_MANAGER_SV
`define DDR5_REFRESH_MANAGER_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"

module ddr5_refresh_manager
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic       ck,
  input  logic       rst_n,
  input  logic       enable,        // = channel ctrl_ready
  input  logic       pb_mode,       // 0 = all-bank REF_AB, 1 = per-bank REF_PB round-robin

  output logic       req_valid,
  output ddr5_req_t  req,
  input  logic       req_ready
);

  int          refi_timer;
  logic [4:0]  rr_ptr;

  localparam int PB_INTERVAL = TREFI / NUM_BANKS;

  wire [31:0] due_threshold = pb_mode ? (PB_INTERVAL - 8) : (TREFI - 32);

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      refi_timer <= 0;
      rr_ptr     <= '0;
    end else if (enable) begin
      if (req_valid && req_ready) begin
        refi_timer <= 0;
        if (pb_mode) rr_ptr <= rr_ptr + 1'b1;
      end else begin
        refi_timer <= refi_timer + 1;
      end
    end
  end

  // fire a little early so the pending refresh has time to be scheduled
  // and legal before the hard deadline
  assign req_valid = enable && (refi_timer >= due_threshold);

  always_comb begin
    req         = '0;
    req.is_write = 1'b0;
    if (pb_mode) begin
      req.cmd = CMD_REF_PB;
      req.bg  = rr_ptr[4:2];
      req.ba  = rr_ptr[1:0];
    end else begin
      req.cmd = CMD_REF_AB;
      req.bg  = '0;
      req.ba  = '0;
    end
  end

endmodule

`endif

