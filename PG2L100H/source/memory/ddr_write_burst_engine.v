//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 从 show-ahead FIFO 取出参数化 C_BURST_BEATS 个 256-bit 字，保持地址/数据并适配 HMIC AW/W 背压握手。
// Module Name: ddr_write_burst_engine
// File Name: source/memory/ddr_write_burst_engine.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 从 show-ahead FIFO 取出参数化 C_BURST_BEATS 个 256-bit 字，保持地址/数据并适配 HMIC AW/W 背压握手。
// 支持预缓存多拍与 C_BURST_BEATS=1 单拍兼容路径，仅实际接受写数据才计数完成；当前顶层选择单拍。
//
// Dependencies: 外部 256-bit FIFO、HMIC_S v1.12 写端口。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Module: ddr_write_burst_engine
// HMIC_S v1.12 write burst adapter: explicit AWVALID/WVALID handshake.
//////////////////////////////////////////////////////////////////////////////////
module ddr_write_burst_engine #(
  parameter integer C_ADDR_WIDTH = 28,
  parameter integer C_BURST_BEATS = 16
)(
  input  i_clk,
  input  i_rst_n,
  input  i_abort,
  input  i_burst_req,
  input  [1:0] i_source_id,
  input  [C_ADDR_WIDTH-1:0] i_start_addr,
  input  [15:0] i_fifo_word_count,
  input  i_fifo_empty,
  input  [255:0] i_fifo_data,
  output reg o_fifo_pop,
  output reg o_busy,
  output reg o_burst_done,
  output reg o_burst_failed,
  output reg [1:0] o_active_source,
  output reg [C_ADDR_WIDTH-1:0] o_active_addr,
  output reg [4:0] o_beat_index,
  output reg [C_ADDR_WIDTH-1:0] o_axi_awaddr,
  output reg [3:0] o_axi_awlen,
  output reg o_axi_awvalid,
  input  i_axi_awready,
  output reg [255:0] o_axi_wdata,
  output reg o_axi_wvalid,
  output reg [31:0] o_axi_wstrb,
  input  i_axi_wready
);

  localparam [3:0] ST_IDLE       = 4'd0;
  localparam [3:0] ST_LOAD_WAIT1 = 4'd1;
  localparam [3:0] ST_LOAD_WAIT2 = 4'd2;
  localparam [3:0] ST_LOAD       = 4'd3;
  localparam [3:0] ST_AW         = 4'd4;
  localparam [3:0] ST_W          = 4'd5;
  reg [3:0] state;
  reg [4:0] load_count;
  reg abort_pending;
  reg [255:0] burst_data [0:C_BURST_BEATS-1];

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      state            <= ST_IDLE;
      o_fifo_pop       <= 1'b0;
      o_busy           <= 1'b0;
      o_burst_done     <= 1'b0;
      o_burst_failed   <= 1'b0;
      o_active_source  <= 2'd0;
      o_active_addr    <= {C_ADDR_WIDTH{1'b0}};
      o_beat_index     <= 5'd0;
      o_axi_awaddr     <= {C_ADDR_WIDTH{1'b0}};
      o_axi_awlen      <= 4'd15;
      o_axi_awvalid    <= 1'b0;
      o_axi_wdata      <= 256'd0;
      o_axi_wvalid     <= 1'b0;
      o_axi_wstrb      <= 32'hffff_ffff;
      abort_pending    <= 1'b0;
    end else begin
      o_fifo_pop     <= 1'b0;
      o_burst_done   <= 1'b0;
      o_burst_failed <= 1'b0;
      o_axi_wvalid   <= 1'b0;

      // In the board-verified single-beat mode, continuously register the
      // show-ahead FIFO head while idle.  The value is then held throughout
      // AW/W backpressure.  Besides matching the HMIC AWLEN=0 contract, this
      // keeps source eligibility/discard logic out of the 256-bit WDATA CE.
      if ((C_BURST_BEATS == 1) && (state == ST_IDLE))
        o_axi_wdata <= i_fifo_data;

      // 只允许取消尚未提出 AW 的预取。AW 一旦提出必须保持直到握手，
      // 此后完整发送缓存中的 16 字，最后再报告 failed。
      if (i_abort && o_busy && (state != ST_AW) && (state != ST_W)) begin
        state         <= ST_IDLE;
        o_busy        <= 1'b0;
        o_axi_awvalid <= 1'b0;
        o_burst_failed<= 1'b1;
        abort_pending <= 1'b0;
      end else begin
        case (state)
          ST_IDLE: begin
            o_busy        <= 1'b0;
            o_axi_awvalid <= 1'b0;
            if (i_burst_req && !i_abort && !i_fifo_empty &&
                (({1'b0,i_fifo_word_count} + (!i_fifo_empty ? 1'b1 : 1'b0)) >= C_BURST_BEATS)) begin
              o_active_source <= i_source_id;
              o_active_addr   <= i_start_addr;
              o_axi_awaddr   <= i_start_addr;
              o_axi_awlen    <= C_BURST_BEATS - 1;
              burst_data[0]  <= i_fifo_data;
              load_count     <= 5'd1;
              o_axi_wstrb    <= 32'hffff_ffff;
              o_beat_index   <= 5'd0;
              o_fifo_pop     <= 1'b1;
              o_busy         <= 1'b1;
              abort_pending  <= 1'b0;
              // HMIC also supports AWLEN=0.  In that compatibility mode the
              // visible FIFO head is the complete transaction, so bypass the
              // preload loop (which otherwise indexes burst_data[1]).
              if (C_BURST_BEATS == 1) begin
                state <= ST_AW;
              end else begin
                state <= ST_LOAD_WAIT1;
              end
            end else if (i_burst_req) begin
              // 仲裁决定到引擎接受之间，源可能 abort 并清 FIFO。
              // 返回失败给仲裁器，不能让 WAIT_DONE 永久等待。
              o_active_source <= i_source_id;
              o_burst_failed <= 1'b1;
            end
          end
          ST_LOAD_WAIT1: begin
            // C6 wrapper clears its one-word show-ahead on pop; vendor FIFO
            // refill becomes visible on the following cycle. Keep a second
            // wait so capture never samples the just-consumed head again.
            o_busy <= 1'b1;
            state <= ST_LOAD_WAIT2;
          end
          ST_LOAD_WAIT2: begin
            o_busy <= 1'b1;
            state <= ST_LOAD;
          end
          ST_LOAD: begin
            o_busy <= 1'b1;
            if (!i_fifo_empty) begin
              burst_data[load_count] <= i_fifo_data;
              if (load_count == C_BURST_BEATS-1) begin
                o_fifo_pop <= 1'b1;
                o_axi_wdata <= burst_data[0];
                state <= ST_AW;
              end else begin
                load_count <= load_count + 1'b1;
                o_fifo_pop <= 1'b1;
                state <= ST_LOAD_WAIT1;
              end
            end
          end
          ST_AW: begin
            o_busy <= 1'b1;
            o_axi_awvalid <= 1'b1;
            if (i_abort)
              abort_pending <= 1'b1;
            if (o_axi_awvalid && i_axi_awready) begin
              o_axi_awvalid <= 1'b0;
              if (C_BURST_BEATS != 1)
                o_axi_wdata <= burst_data[0];
              // Present the first W beat in the cycle immediately following
              // AW handshake; subsequent beats advance only on W handshake.
              o_axi_wvalid <= 1'b1;
              state <= ST_W;
            end
          end
          ST_W: begin
            o_busy <= 1'b1;
            o_axi_wvalid <= 1'b1;
            if (i_abort)
              abort_pending <= 1'b1;
            if (i_axi_wready) begin
              if (o_beat_index == C_BURST_BEATS-1) begin
                o_beat_index <= C_BURST_BEATS;
                o_busy       <= 1'b0;
                o_axi_wvalid <= 1'b0;
                if (abort_pending || i_abort)
                  o_burst_failed <= 1'b1;
                else
                  o_burst_done <= 1'b1;
                abort_pending <= 1'b0;
                state        <= ST_IDLE;
              end else begin
                o_beat_index <= o_beat_index + 1'b1;
                o_axi_wdata  <= burst_data[o_beat_index + 1'b1];
              end
            end
          end
          default: begin
            state <= ST_IDLE;
            o_busy <= 1'b0;
            o_axi_awvalid <= 1'b0;
          end
        endcase
      end
    end
  end
endmodule
