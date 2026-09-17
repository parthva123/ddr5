// ============================================================================
// ddr5_axi4_read_channel.sv
// AXI4 AR / R handling. Decodes ARADDR via ddr5_address_mapper, pushes
// one ddr5_req_t descriptor into the addressed channel's command
// request_fifo, then reassembles pairs of 32-bit DDR5 read-data beats
// (popped from that channel's read-data FIFO) into 64-bit RDATA words.
// ============================================================================
`ifndef DDR5_AXI4_READ_CHANNEL_SV
`define DDR5_AXI4_READ_CHANNEL_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_address_mapper.sv"

module ddr5_axi4_read_channel
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic         aclk,
  input  logic         aresetn,

  input  logic [31:0]  araddr,
  input  logic [7:0]   arlen,
  input  logic [3:0]   arid,
  input  logic         arvalid,
  output logic         arready,

  output logic [63:0]  rdata,
  output logic [3:0]   rid,
  output logic [1:0]   rresp,
  output logic         rlast,
  output logic         rvalid,
  input  logic         rready,

  // channel-A command sink / read-data source
  output logic         a_cmd_wr_en, input logic a_cmd_full, output ddr5_req_t a_cmd_req,
  output logic         a_rd_pop,    input logic a_rd_empty, input logic [31:0] a_rd_data,
  input  logic [7:0]   a_mr_array [0:NUM_MR-1],

  // channel-B command sink / read-data source
  output logic         b_cmd_wr_en, input logic b_cmd_full, output ddr5_req_t b_cmd_req,
  output logic         b_rd_pop,    input logic b_rd_empty, input logic [31:0] b_rd_data,
  input  logic [7:0]   b_mr_array [0:NUM_MR-1]
);

  localparam logic [1:0] RESP_OKAY   = 2'b00;
  localparam logic [1:0] RESP_DECERR = 2'b11;

  logic [2:0]       m_bg;
  logic [1:0]       m_ba;
  logic [ROW_W-1:0] m_row;
  logic [COL_W-1:0] m_col;
  logic             m_ch_sel, m_is_mr, m_ap, m_bc8, m_addr_err;
  logic             m_is_ctrl_cmd, m_ctrl_is_pre_all;
  logic [7:0]       m_mr_index;

  ddr5_address_mapper u_map (
    .addr(araddr), .bg(m_bg), .ba(m_ba), .row(m_row), .col(m_col),
    .ch_sel(m_ch_sel), .is_mr(m_is_mr), .mr_index(m_mr_index),
    .is_ctrl_cmd(m_is_ctrl_cmd), .ctrl_is_pre_all(m_ctrl_is_pre_all),
    .ap(m_ap), .bc8(m_bc8), .addr_error(m_addr_err)
  );

  typedef enum logic [2:0] {R_IDLE, R_ISSUE, R_LO, R_HI, R_MR_RESP} r_state_e;
  r_state_e   r_st;
  logic [7:0] beats_left;
  logic       sel_ch_r, is_mr_r;
  logic [7:0] mr_index_r;
  logic [3:0] id_r;
  logic [1:0] resp_r;
  logic [31:0] lo_r;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      r_st <= R_IDLE; arready <= 1'b0; rvalid <= 1'b0;
      a_cmd_wr_en <= 1'b0; b_cmd_wr_en <= 1'b0; a_rd_pop <= 1'b0; b_rd_pop <= 1'b0;
    end else begin
      arready <= 1'b0; a_cmd_wr_en <= 1'b0; b_cmd_wr_en <= 1'b0;
      a_rd_pop <= 1'b0; b_rd_pop <= 1'b0;
      if (rvalid && rready) rvalid <= 1'b0;

      unique case (r_st)
        R_IDLE: if (arvalid) begin
          id_r <= arid;
          if (m_addr_err) begin
            resp_r  <= RESP_DECERR;
            arready <= 1'b1;
            rdata   <= '0;
            rid     <= arid;
            rresp   <= RESP_DECERR;
            rlast   <= 1'b1;
            rvalid  <= 1'b1;
            r_st    <= R_IDLE;
          end else begin
            sel_ch_r   <= m_ch_sel;
            is_mr_r    <= m_is_mr;
            mr_index_r <= m_mr_index;
            beats_left <= arlen + 8'd1;
            resp_r     <= RESP_OKAY;
            arready    <= 1'b1;
            r_st       <= R_ISSUE;
          end
        end

        R_ISSUE: begin
          ddr5_req_t r;
          r = '0;
          r.is_write = 1'b0;
          r.tag      = id_r;
          if (is_mr_r) begin
            r.cmd     = CMD_MRR;      // issued for protocol/coverage; the
            r.mr_addr = mr_index_r;   // actual value is read back directly
          end else begin
            r.cmd = CMD_ACT;          // channel controller expands ACT->RD internally
            r.bg = m_bg; r.ba = m_ba; r.row = m_row; r.col = m_col;
            r.ap = m_ap; r.bc8 = m_bc8;
          end

          if (!sel_ch_r && !a_cmd_full) begin
            a_cmd_req <= r; a_cmd_wr_en <= 1'b1;
            if (is_mr_r) r_st <= R_MR_RESP; else r_st <= R_LO;
          end else if (sel_ch_r && !b_cmd_full) begin
            b_cmd_req <= r; b_cmd_wr_en <= 1'b1;
            if (is_mr_r) r_st <= R_MR_RESP; else r_st <= R_LO;
          end
        end

        R_MR_RESP: begin
          // Mode-Register read-back: single-beat, sourced directly from
          // the per-channel MR file (a slow-changing config register, so
          // a direct cross-domain combinational read is an accepted
          // simplification here rather than a full synchroniser).
          if (!rvalid) begin
            rdata  <= {56'b0, (sel_ch_r ? b_mr_array[mr_index_r] : a_mr_array[mr_index_r])};
            rid    <= id_r;
            rresp  <= resp_r;
            rlast  <= 1'b1;
            rvalid <= 1'b1;
            r_st   <= R_IDLE;
          end
        end

        R_LO: begin
          if (!sel_ch_r && !a_rd_empty) begin
            lo_r <= a_rd_data; a_rd_pop <= 1'b1; r_st <= R_HI;
          end else if (sel_ch_r && !b_rd_empty) begin
            lo_r <= b_rd_data; b_rd_pop <= 1'b1; r_st <= R_HI;
          end
        end

        R_HI: begin
          if (!rvalid) begin
            if (!sel_ch_r && !a_rd_empty) begin
              rdata      <= {a_rd_data, lo_r};
              a_rd_pop   <= 1'b1;
              rid        <= id_r;
              rresp      <= resp_r;
              beats_left <= beats_left - 8'd1;
              rlast      <= (beats_left == 8'd1);
              rvalid     <= 1'b1;
              r_st       <= (beats_left == 8'd1) ? R_IDLE : R_LO;
            end else if (sel_ch_r && !b_rd_empty) begin
              rdata      <= {b_rd_data, lo_r};
              b_rd_pop   <= 1'b1;
              rid        <= id_r;
              rresp      <= resp_r;
              beats_left <= beats_left - 8'd1;
              rlast      <= (beats_left == 8'd1);
              rvalid     <= 1'b1;
              r_st       <= (beats_left == 8'd1) ? R_IDLE : R_LO;
            end
          end
        end
        default: r_st <= R_IDLE;
      endcase
    end
  end

endmodule

`endif

