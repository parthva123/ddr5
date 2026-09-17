// ============================================================================
// ddr5_axi4_write_channel.sv
// AXI4 AW / W / B handling. Decodes AWADDR via ddr5_address_mapper,
// pushes one ddr5_req_t descriptor per burst into the addressed
// channel's command request_fifo, and streams WDATA (split 64-bit AXI
// beats into two 32-bit DDR5 beats) into that channel's write-data
// FIFO. Mode-Register writes (addr[31]=1) carry their 8-bit MR data in
// WDATA[7:0] and consume no DDR5 data beats.
// ============================================================================
`ifndef DDR5_AXI4_WRITE_CHANNEL_SV
`define DDR5_AXI4_WRITE_CHANNEL_SV

`include "ddr5_pkg.sv"
`include "ddr5_types_pkg.sv"
`include "ddr5_address_mapper.sv"

module ddr5_axi4_write_channel
  import ddr5_pkg::*;
  import ddr5_types_pkg::*;
(
  input  logic         aclk,
  input  logic         aresetn,

  input  logic [31:0]  awaddr,
  input  logic [7:0]   awlen,
  input  logic [3:0]   awid,
  input  logic         awvalid,
  output logic         awready,

  input  logic [63:0]  wdata,
  input  logic [7:0]   wstrb,
  input  logic         wlast,
  input  logic         wvalid,
  output logic         wready,

  output logic [3:0]   bid,
  output logic [1:0]   bresp,
  output logic         bvalid,
  input  logic         bready,

  // channel-A command/data sink
  output logic         a_cmd_wr_en,  input logic a_cmd_full,  output ddr5_req_t a_cmd_req,
  output logic         a_wd_wr_en,   input logic a_wd_full,   output logic [35:0] a_wd_data, // {dm_n[3:0], data[31:0]}

  // channel-B command/data sink
  output logic         b_cmd_wr_en,  input logic b_cmd_full,  output ddr5_req_t b_cmd_req,
  output logic         b_wd_wr_en,   input logic b_wd_full,   output logic [35:0] b_wd_data
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
    .addr(awaddr), .bg(m_bg), .ba(m_ba), .row(m_row), .col(m_col),
    .ch_sel(m_ch_sel), .is_mr(m_is_mr), .mr_index(m_mr_index),
    .is_ctrl_cmd(m_is_ctrl_cmd), .ctrl_is_pre_all(m_ctrl_is_pre_all),
    .ap(m_ap), .bc8(m_bc8), .addr_error(m_addr_err)
  );

  typedef enum logic [2:0] {W_IDLE, W_ISSUE, W_DATA, W_NODATA, W_RESP} w_state_e;
  w_state_e   w_st;
  logic [7:0] beats_left;
  logic       sel_ch_r, is_mr_r, is_ctrl_r, ctrl_pre_all_r;
  logic [3:0] id_r;
  logic [1:0] resp_r;

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      w_st <= W_IDLE; awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0;
      a_cmd_wr_en <= 1'b0; b_cmd_wr_en <= 1'b0; a_wd_wr_en <= 1'b0; b_wd_wr_en <= 1'b0;
    end else begin
      a_cmd_wr_en <= 1'b0; b_cmd_wr_en <= 1'b0; a_wd_wr_en <= 1'b0; b_wd_wr_en <= 1'b0;
      awready <= 1'b0; wready <= 1'b0;

      unique case (w_st)
        W_IDLE: if (awvalid) begin
          id_r <= awid;
          if (m_addr_err) begin
            resp_r  <= RESP_DECERR;
            awready <= 1'b1;
            w_st    <= W_RESP;
          end else begin
            sel_ch_r       <= m_ch_sel;
            is_mr_r        <= m_is_mr;
            is_ctrl_r      <= m_is_ctrl_cmd;
            ctrl_pre_all_r <= m_ctrl_is_pre_all;
            beats_left     <= awlen + 8'd1;
            resp_r         <= RESP_OKAY;
            awready        <= 1'b1;
            w_st           <= W_ISSUE;
          end
        end

        W_ISSUE: begin
          ddr5_req_t r;
          r = '0;
          r.is_write = 1'b1;
          r.tag      = id_r;
          if (is_mr_r) begin
            r.cmd     = CMD_MRW;
            r.mr_addr = m_mr_index;
            r.mr_data = wdata[7:0];   // MR data rides on the first WDATA beat
          end else begin
            r.cmd = CMD_ACT;          // channel controller expands ACT->WR internally
            r.bg  = m_bg; r.ba = m_ba; r.row = m_row; r.col = m_col;
            r.ap  = m_ap; r.bc8 = m_bc8;
          end
          if (is_ctrl_r) begin
            if (ctrl_pre_all_r) r.cmd = CMD_PRE_ALL;
            else                 r.cmd = CMD_PRE;
            r.bg = m_bg; r.ba = m_ba;
          end

          if (!sel_ch_r && !a_cmd_full) begin
            a_cmd_req <= r; a_cmd_wr_en <= 1'b1;
            if (is_mr_r || is_ctrl_r) w_st <= W_NODATA;
            else                       w_st <= W_DATA;
          end else if (sel_ch_r && !b_cmd_full) begin
            b_cmd_req <= r; b_cmd_wr_en <= 1'b1;
            if (is_mr_r || is_ctrl_r) w_st <= W_NODATA;
            else                       w_st <= W_DATA;
          end
        end

        W_DATA: begin
          wready <= 1'b1;
          if (wvalid && wready) begin
            if (!sel_ch_r && !a_wd_full) begin
              a_wd_data  <= {~wstrb[3:0], wdata[31:0]};
              a_wd_wr_en <= 1'b1;
            end else if (sel_ch_r && !b_wd_full) begin
              b_wd_data  <= {~wstrb[3:0], wdata[31:0]};
              b_wd_wr_en <= 1'b1;
            end
            beats_left <= beats_left - 8'd1;
            if (wlast || beats_left == 8'd1) w_st <= W_RESP;
          end
        end

        W_NODATA: begin
          w_st <= W_RESP;   // MRW/PRE/PRE_ALL carry no AXI write-data beat
        end

        W_RESP: begin
          bvalid <= 1'b1;
          if (bvalid && bready) begin
            bvalid <= 1'b0;
            w_st   <= W_IDLE;
          end
        end
        default: w_st <= W_IDLE;
      endcase
    end
  end

  assign bid   = id_r;
  assign bresp = resp_r;

endmodule

`endif

