// ============================================================================
// ddr5_init_fsm.sv
// Per-channel JEDEC DDR5 power-up sequence:
//   RESET# low -> RESET# high (CKE still low) -> CKE high -> ZQCAL Start
//   -> wait tZQCAL -> ZQCAL Latch -> default MR programming -> ready.
//
// Drives reset_n/cke pin-level outputs directly (they are not part of
// the 2-cycle CA command stream) and issues its ZQCAL/MRW steps as
// ddr5_req_t requests through the scheduler's highest-priority input, so
// they share the same command_fsm/bank_tracker/cmd_encoder datapath as
// normal traffic.
// ============================================================================
`ifndef DDR5_INIT_FSM_SV
`define DDR5_INIT_FSM_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"

module ddr5_init_fsm
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic       ck,
  input  logic       por_reset_n,

  output logic       reset_n,
  output logic       cke,

  output logic       req_valid,
  output ddr5_req_t  req,
  input  logic       req_ready,

  output logic       ctrl_ready
);

  typedef enum logic [2:0] {
    S_RESET, S_CKE_LOW, S_ZQ_START, S_ZQ_WAIT, S_ZQ_LATCH, S_MR_INIT, S_DONE
  } state_e;

  state_e      st;
  int          cnt;
  logic [3:0]  mr_idx;

  localparam int NUM_DEFAULT_MR = 8;

  // simple default MR value table programmed automatically at init
  // (index i -> MRi = default_mr_val[i]); DBI enables live in MR3.
  function automatic logic [7:0] default_mr_val(input logic [3:0] i);
    unique case (i)
      4'd3:    return 8'b0110_0000; // MR3: WR-DBI + RD-DBI enabled
      default: return 8'h00;
    endcase
  endfunction

  assign req_valid = (st == S_ZQ_START) || (st == S_ZQ_LATCH) || (st == S_MR_INIT);

  always_comb begin
    req = '0;
    unique case (st)
      S_ZQ_START: req.cmd = CMD_ZQ_START;
      S_ZQ_LATCH: req.cmd = CMD_ZQ_LATCH;
      S_MR_INIT: begin
        req.cmd     = CMD_MRW;
        req.mr_addr = {4'b0, mr_idx};
        req.mr_data = default_mr_val(mr_idx);
      end
      default: req.cmd = CMD_NOP;
    endcase
  end

  always_ff @(posedge ck or negedge por_reset_n) begin
    if (!por_reset_n) begin
      st         <= S_RESET;
      cnt        <= 0;
      mr_idx     <= '0;
      reset_n    <= 1'b0;
      cke        <= 1'b0;
      ctrl_ready <= 1'b0;
    end else begin
      unique case (st)
        S_RESET: begin
          reset_n <= 1'b0;
          cke     <= 1'b0;
          if (cnt < 1000) cnt <= cnt + 1;
          else begin reset_n <= 1'b1; cnt <= 0; st <= S_CKE_LOW; end
        end

        S_CKE_LOW: begin
          if (cnt < TXPR) cnt <= cnt + 1;
          else begin cke <= 1'b1; cnt <= 0; st <= S_ZQ_START; end
        end

        S_ZQ_START: if (req_valid && req_ready) begin
          cnt <= 0;
          st  <= S_ZQ_WAIT;
        end

        S_ZQ_WAIT: begin
          if (cnt < TZQCAL) cnt <= cnt + 1;
          else st <= S_ZQ_LATCH;
        end

        S_ZQ_LATCH: if (req_valid && req_ready) begin
          mr_idx <= '0;
          st     <= S_MR_INIT;
        end

        S_MR_INIT: if (req_valid && req_ready) begin
          if (mr_idx == NUM_DEFAULT_MR - 1) st <= S_DONE;
          else mr_idx <= mr_idx + 1'b1;
        end

        S_DONE: ctrl_ready <= 1'b1;
        default: st <= S_RESET;
      endcase
    end
  end

endmodule

`endif

