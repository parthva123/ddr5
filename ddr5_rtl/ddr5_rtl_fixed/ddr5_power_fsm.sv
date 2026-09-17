// ============================================================================
// ddr5_power_fsm.sv
// Post-init CKE (power-down) management for one channel.
//
// After ddr5_init_fsm brings the channel up (ctrl_ready), this block
// takes over CKE: if the channel goes idle (no in-flight command, no
// pending request) for IDLE_THRESHOLD cycles, it enters power-down
// (CKE low, held for tCKE), stays down for a fixed dwell, then exits
// (CKE high, wait tXP) before signalling `pwr_active` so
// ddr5_channel_ctrl resumes accepting new requests.
//
// Simplification: entry/exit are self-timed (a fixed idle threshold and
// a fixed power-down dwell) rather than event-driven wake-on-next-
// request. A fully traffic-aware wake would need power_fsm to see the
// scheduler's incoming request the instant it arrives, which would
// require deeper coupling into ddr5_scheduler; the self-timed version
// still exercises real tCKE/tXP legality and the CKE toggle safely,
// which is what CLK-TC-02 (CKE and CK Gating Verification) checks.
// ============================================================================
`ifndef DDR5_POWER_FSM_SV
`define DDR5_POWER_FSM_SV

`include "ddr5_pkg.sv"

module ddr5_power_fsm
  import ddr5_pkg::*;
#(
  parameter int IDLE_THRESHOLD = 256,  // idle cycles before entering power-down
  parameter int PD_DWELL       = 512   // cycles spent in power-down before auto-exit
)(
  input  logic ck,
  input  logic rst_n,
  input  logic ctrl_ready,   // from ddr5_init_fsm — power management only runs once true
  input  logic chan_idle,    // from ddr5_channel_ctrl — no in-flight / pending command

  output logic cke,
  output logic pwr_active    // 1 = fully active, gates ddr5_channel_ctrl.req_ready
);

  typedef enum logic [1:0] {PWR_ACTIVE, PWR_ENTER, PWR_DOWN, PWR_EXIT} state_e;
  state_e st;
  int     cnt;
  int     idle_cnt;

  always_ff @(posedge ck or negedge rst_n) begin
    if (!rst_n) begin
      st <= PWR_ACTIVE; cnt <= 0; idle_cnt <= 0; cke <= 1'b0; pwr_active <= 1'b0;
    end else if (!ctrl_ready) begin
      st <= PWR_ACTIVE; cnt <= 0; idle_cnt <= 0; cke <= 1'b0; pwr_active <= 1'b0;
    end else begin
      unique case (st)
        PWR_ACTIVE: begin
          cke        <= 1'b1;
          pwr_active <= 1'b1;
          if (chan_idle) begin
            if (idle_cnt < IDLE_THRESHOLD) idle_cnt <= idle_cnt + 1;
            else begin
              idle_cnt   <= 0;
              cnt        <= 0;
              pwr_active <= 1'b0;   // stop granting new requests before CKE drops
              st         <= PWR_ENTER;
            end
          end else begin
            idle_cnt <= 0;
          end
        end

        PWR_ENTER: begin          // CKE deasserted, hold for tCKE
          cke <= 1'b0;
          if (cnt < TCKE) cnt <= cnt + 1;
          else begin cnt <= 0; st <= PWR_DOWN; end
        end

        PWR_DOWN: begin           // powered down for a fixed dwell
          cke <= 1'b0;
          if (cnt < PD_DWELL) cnt <= cnt + 1;
          else begin cnt <= 0; st <= PWR_EXIT; end
        end

        PWR_EXIT: begin           // CKE re-asserted, hold tXP before new commands
          cke <= 1'b1;
          if (cnt < TXP) cnt <= cnt + 1;
          else begin cnt <= 0; pwr_active <= 1'b1; st <= PWR_ACTIVE; end
        end
        default: st <= PWR_ACTIVE;
      endcase
    end
  end

endmodule

`endif

