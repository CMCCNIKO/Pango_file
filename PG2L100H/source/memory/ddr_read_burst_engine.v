//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: HMIC_S v1.12 读突发适配器，生成 AR 请求并统计无 RREADY 的连续返回。
// Module Name: ddr_read_burst_engine
// File Name: source/memory/ddr_read_burst_engine.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// HMIC_S v1.12 读突发适配器，生成 AR 请求并统计无 RREADY 的连续返回。
// 发出请求前预留完整 burst 空间，先缓存整笔返回再按下游 ready/valid 交付；旧事务排空后才允许复用缓冲。
//
// Dependencies: 外部 HMIC 读端口；本模块内部突发缓冲寄存器。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Module: ddr_read_burst_engine
// HMIC_S v1.12 read adapter: no RREADY; buffer one complete burst before output.
//////////////////////////////////////////////////////////////////////////////////
module ddr_read_burst_engine #(
  parameter integer C_ADDR_WIDTH = 28,
  parameter integer C_MAX_BEATS = 16,
  parameter integer C_WRITE_ADDR_HALF_CYCLE = 0
)(
  input  i_clk,
  input  i_rst_n,
  input  i_abort,
  input  i_burst_req,
  input  [C_ADDR_WIDTH-1:0] i_start_addr,
  input  [3:0] i_burst_len,
  input  [15:0] i_output_capacity,
  output reg o_busy,
  output reg o_burst_done,
  output reg o_burst_failed,
  output reg [C_ADDR_WIDTH-1:0] o_axi_araddr,
  output reg [3:0] o_axi_arlen,
  output reg o_axi_arvalid,
  input  i_axi_arready,
  input [255:0] i_axi_rdata,
  input i_axi_rvalid,
  output o_word_valid,
  output [255:0] o_word_data,
  input i_word_ready
);

  localparam [2:0] ST_IDLE = 3'd0;
  localparam [2:0] ST_AR   = 3'd1;
  localparam [2:0] ST_R    = 3'd2;
  localparam [2:0] ST_PUSH = 3'd3;
  reg [2:0] state;
  reg [4:0] beats_total;
  reg [4:0] r_count;
  reg [4:0] push_count;
  reg abort_pending;
  reg [255:0] burst_data [0:C_MAX_BEATS-1];
  wire [4:0] burst_write_addr;

  // Stage only the RAM write address; the return data and write cycle stay unchanged.
  generate
    if (C_WRITE_ADDR_HALF_CYCLE != 0) begin : gen_half_cycle_write_addr
      reg [4:0] reg_write_addr;
      always @(negedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n)
          reg_write_addr <= 5'd0;
        else
          reg_write_addr <= r_count;
      end
      assign burst_write_addr = reg_write_addr;
    end else begin : gen_direct_write_addr
      assign burst_write_addr = r_count;
    end
  endgenerate

  assign o_word_valid = (state == ST_PUSH);
  assign o_word_data = (state == ST_PUSH) ? burst_data[push_count] : 256'd0;

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      state          <= ST_IDLE;
      beats_total    <= 5'd0;
      r_count        <= 5'd0;
      push_count     <= 5'd0;
      o_busy         <= 1'b0;
      o_burst_done   <= 1'b0;
      o_burst_failed <= 1'b0;
      o_axi_araddr   <= {C_ADDR_WIDTH{1'b0}};
      o_axi_arlen    <= 4'd0;
      o_axi_arvalid  <= 1'b0;
      abort_pending  <= 1'b0;
    end else begin
      o_burst_done   <= 1'b0;
      o_burst_failed <= 1'b0;
      // ARVALID 已提出后不允许撤销。即使 abort 与 ARREADY 同拍，
      // 仍须记录接收，并排空全部无 RREADY 的返回 beat。
      if (i_abort && o_busy && (state != ST_R) &&
          !((state == ST_AR) && o_axi_arvalid)) begin
        state         <= ST_IDLE;
        o_busy        <= 1'b0;
        o_axi_arvalid <= 1'b0;
        o_burst_failed<= 1'b1;
        abort_pending <= 1'b0;
      end else begin
        case (state)
          ST_IDLE: begin
            o_busy <= 1'b0;
            o_axi_arvalid <= 1'b0;
            if (i_burst_req && !i_abort &&
                (i_output_capacity >= ({12'd0,i_burst_len} + 1'b1))) begin
              beats_total   <= {1'b0,i_burst_len} + 1'b1;
              o_axi_araddr  <= i_start_addr;
              o_axi_arlen   <= i_burst_len;
              o_busy        <= 1'b1;
              abort_pending <= 1'b0;
              state         <= ST_AR;
            end
          end
          ST_AR: begin
            o_busy        <= 1'b1;
            o_axi_arvalid <= 1'b1;
            if (i_abort) abort_pending <= 1'b1;
            if (o_axi_arvalid && i_axi_arready) begin
              o_axi_arvalid <= 1'b0;
              r_count       <= 5'd0;
              state         <= ST_R;
            end
          end
          ST_R: begin
            o_busy <= 1'b1;
            // HMIC_S has no RREADY. Once AR is accepted, an abort must keep
            // sampling every RVALID beat to release the outstanding burst;
            // received words are discarded while abort_pending is set.
            if (i_abort)
              abort_pending <= 1'b1;
            if (i_axi_rvalid) begin
              if (!abort_pending && !i_abort)
                burst_data[burst_write_addr] <= i_axi_rdata;
              if (r_count == beats_total-1) begin
                if (abort_pending || i_abort) begin
                  o_busy         <= 1'b0;
                  o_burst_failed <= 1'b1;
                  abort_pending  <= 1'b0;
                  state          <= ST_IDLE;
                end else begin
                  push_count <= 5'd0;
                  state <= ST_PUSH;
                end
              end else begin
                r_count <= r_count + 1'b1;
              end
            end
          end
          ST_PUSH: begin
            o_busy <= 1'b1;
            if (i_word_ready) begin
              if (push_count == beats_total-1) begin
                o_busy       <= 1'b0;
                o_burst_done <= 1'b1;
                state        <= ST_IDLE;
                abort_pending <= 1'b0;
              end else begin
                push_count <= push_count + 1'b1;
              end
            end
          end
          default: begin
            state <= ST_IDLE;
            o_busy <= 1'b0;
            o_axi_arvalid <= 1'b0;
          end
        endcase
      end
    end
  end
endmodule
