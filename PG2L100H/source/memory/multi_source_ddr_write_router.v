//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四路 256-bit 源 FIFO 到共享 HMIC 写突发的路由器，组合仲裁、固定源数据选择、每源槽内偏移和写完成回报。
// Module Name: multi_source_ddr_write_router
// File Name: source/memory/multi_source_ddr_write_router.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四路 256-bit 源 FIFO 到共享 HMIC 写突发的路由器，组合仲裁、固定源数据选择、每源槽内偏移和写完成回报。
// 整个事务保持同一源及 FIFO，按 C_BURST_BEATS*8 推进 HMIC 地址；支持参数化 burst，当前生产顶层传入 C_BURST_BEATS=1。
//
// Dependencies: multi_source_ddr_write_arbiter、ddr_write_burst_engine。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 原文件历史 Engineer 为 Codex；本次维护头不得把原始作者改造成 VinnLee。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: Codex
// Create Date: 2026-09-14
// Design Name: Quad Video Mosaic
// Module Name: multi_source_ddr_write_router
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 四路 256-bit FIFO 到 HMIC_S 16-beat 写 burst 的统一路由器。
//              仲裁器固定 burst 源，路由器在整个 burst 期间保持对应 FIFO。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module multi_source_ddr_write_router #(
  parameter integer C_ADDR_WIDTH = 28,
  parameter integer C_BURST_BEATS = 16
)(
  input  i_clk,
  input  i_rst_n,
  input  [3:0] i_frame_prepare,
  input  [3:0] i_frame_error,
  input  [3:0] i_request,
  input  [3:0] i_almost_full,
  input  i_read_urgent,
  input  i_engine_ready,
  input  [63:0] i_fifo_word_count,
  input  [3:0] i_fifo_empty,
  input  [1023:0] i_fifo_data,
  output [3:0] o_word_pop,
  input  [111:0] i_source_base,
  output [C_ADDR_WIDTH-1:0] o_axi_awaddr,
  output [3:0] o_axi_awlen,
  output o_axi_awvalid,
  input i_axi_awready,
  output [255:0] o_axi_wdata,
  output o_axi_wvalid,
  input i_axi_wready,
  output o_burst_done,
  output o_burst_failed,
  output o_busy,
  output [1:0] o_active_source
);

  wire arb_start;
  wire [1:0] arb_source;
  wire arb_busy;
  wire [127:0] arb_burst_count;
  wire engine_busy;
  wire engine_done;
  wire engine_failed;
  wire engine_pop;
  wire [1:0] engine_source;
  wire [C_ADDR_WIDTH-1:0] engine_addr;
  wire [4:0] engine_beat;
  wire [C_ADDR_WIDTH-1:0] engine_awaddr;
  wire [3:0] engine_awlen;
  wire engine_awvalid;
  wire [255:0] engine_wdata;
  wire engine_wvalid;
  wire [31:0] engine_wstrb;

  reg [1:0] source_sel;
  reg [15:0] fifo_count_sel;
  reg fifo_empty_sel;
  reg [255:0] fifo_data_sel;
  reg [C_ADDR_WIDTH-1:0] source_base_sel;
  reg [C_ADDR_WIDTH-1:0] source_offset0;
  reg [C_ADDR_WIDTH-1:0] source_offset1;
  reg [C_ADDR_WIDTH-1:0] source_offset2;
  reg [C_ADDR_WIDTH-1:0] source_offset3;
  localparam [C_ADDR_WIDTH-1:0] C_BURST_ADDR_SPAN = C_BURST_BEATS * 8;

  // Each producer owns its own linear address cursor. A new frame resets only
  // that producer; successful HMIC bursts advance only their source cursor.
  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      source_offset0 <= {C_ADDR_WIDTH{1'b0}};
      source_offset1 <= {C_ADDR_WIDTH{1'b0}};
      source_offset2 <= {C_ADDR_WIDTH{1'b0}};
      source_offset3 <= {C_ADDR_WIDTH{1'b0}};
    end else begin
      if (i_frame_prepare[0]) source_offset0 <= {C_ADDR_WIDTH{1'b0}};
      else if (engine_done && engine_source == 2'd0)
        source_offset0 <= source_offset0 + C_BURST_ADDR_SPAN;
      if (i_frame_prepare[1]) source_offset1 <= {C_ADDR_WIDTH{1'b0}};
      else if (engine_done && engine_source == 2'd1)
        source_offset1 <= source_offset1 + C_BURST_ADDR_SPAN;
      if (i_frame_prepare[2]) source_offset2 <= {C_ADDR_WIDTH{1'b0}};
      else if (engine_done && engine_source == 2'd2)
        source_offset2 <= source_offset2 + C_BURST_ADDR_SPAN;
      if (i_frame_prepare[3]) source_offset3 <= {C_ADDR_WIDTH{1'b0}};
      else if (engine_done && engine_source == 2'd3)
        source_offset3 <= source_offset3 + C_BURST_ADDR_SPAN;
    end
  end

  // The arbiter source is used until the engine accepts the request; the
  // engine's latched source is used thereafter, so FIFO routing cannot move.
  always @(*) begin
    source_sel = engine_busy ? engine_source : arb_source;
    case (source_sel)
      2'd0: begin
        fifo_count_sel = i_fifo_word_count[15:0];
        fifo_empty_sel = i_fifo_empty[0];
        fifo_data_sel = i_fifo_data[255:0];
        source_base_sel = i_source_base[27:0] + source_offset0;
      end
      2'd1: begin
        fifo_count_sel = i_fifo_word_count[31:16];
        fifo_empty_sel = i_fifo_empty[1];
        fifo_data_sel = i_fifo_data[511:256];
        source_base_sel = i_source_base[55:28] + source_offset1;
      end
      2'd2: begin
        fifo_count_sel = i_fifo_word_count[47:32];
        fifo_empty_sel = i_fifo_empty[2];
        fifo_data_sel = i_fifo_data[767:512];
        source_base_sel = i_source_base[83:56] + source_offset2;
      end
      default: begin
        fifo_count_sel = i_fifo_word_count[63:48];
        fifo_empty_sel = i_fifo_empty[3];
        fifo_data_sel = i_fifo_data[1023:768];
        source_base_sel = i_source_base[111:84] + source_offset3;
      end
    endcase
  end

  multi_source_ddr_write_arbiter u_arbiter(
    .i_clk(i_clk), .i_rstn(i_rst_n), .i_request(i_request),
    .i_almost_full(i_almost_full), .i_read_urgent(i_read_urgent),
    .i_engine_ready(i_engine_ready && !engine_busy),
    .i_burst_done(engine_done), .i_burst_failed(engine_failed),
    .o_start(arb_start), .o_source_id(arb_source), .o_busy(arb_busy),
    .o_burst_count(arb_burst_count)
  );

  ddr_write_burst_engine #(
    .C_ADDR_WIDTH(C_ADDR_WIDTH), .C_BURST_BEATS(C_BURST_BEATS)
  ) u_engine(
    .i_clk(i_clk), .i_rst_n(i_rst_n),
    .i_abort(i_frame_error[engine_busy ? engine_source : arb_source]),
    .i_burst_req(arb_start), .i_source_id(arb_source),
    .i_start_addr(source_base_sel), .i_fifo_word_count(fifo_count_sel),
    .i_fifo_empty(fifo_empty_sel), .i_fifo_data(fifo_data_sel),
    .o_fifo_pop(engine_pop), .o_busy(engine_busy),
    .o_burst_done(engine_done), .o_burst_failed(engine_failed),
    .o_active_source(engine_source), .o_active_addr(engine_addr),
    .o_beat_index(engine_beat), .o_axi_awaddr(engine_awaddr),
    .o_axi_awlen(engine_awlen), .o_axi_awvalid(engine_awvalid),
    .i_axi_awready(i_axi_awready), .o_axi_wdata(engine_wdata),
    .o_axi_wvalid(engine_wvalid), .o_axi_wstrb(engine_wstrb),
    .i_axi_wready(i_axi_wready)
  );

  assign o_word_pop = engine_pop ? (4'b0001 << engine_source) : 4'b0000;
  assign o_axi_awaddr = engine_awaddr;
  assign o_axi_awlen = engine_awlen;
  assign o_axi_awvalid = engine_awvalid;
  assign o_axi_wdata = engine_wdata;
  assign o_axi_wvalid = engine_wvalid;
  assign o_burst_done = engine_done;
  assign o_burst_failed = engine_failed;
  assign o_busy = engine_busy | arb_busy;
  assign o_active_source = engine_source;
endmodule
