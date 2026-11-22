//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-15
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四源写入流水边界，集中连接各源 frame ownership、槽基址、实际写握手计数、burst 仲裁和共享 HMIC 写路由。
// Module Name: quad_source_ddr_write_pipeline
// File Name: source/memory/quad_source_ddr_write_pipeline.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四源写入流水边界，集中连接各源 frame ownership、槽基址、实际写握手计数、burst 仲裁和共享 HMIC 写路由。
// 输入 FIFO 已在 DDR core_clk 域；槽完成以实际 W 接受和 terminal/drained 为依据，并处理 show-ahead 尾批与重复 prepare。
//
// Dependencies: quad_frame_slot_manager、multi_source_ddr_write_router、quad_frame_address_map.vh；外部真实 HMIC IP。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-09-15
// Design Name: Quad Video Mosaic
// Module Name: quad_source_ddr_write_pipeline
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4 / ModelSim 10.6e
// Description: 四源 frame ownership、槽位基址、写仲裁和 HMIC burst 路由边界。
//              输入 FIFO 均已处于 DDR core_clk 域；本模块不实例化或替换
//              厂商 HMIC IP，调用方负责连接 AWUSER/WSTRB 等固定端口。
// Dependencies: quad_frame_slot_manager、multi_source_ddr_write_router
// Revision: 0.02 - 修正 show-ahead 尾批、真实 W handshake 计数和重复 prepare 保护
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_source_ddr_write_pipeline #(
  parameter integer C_ADDR_WIDTH = 28,
  parameter integer C_BURST_BEATS = 16,
  parameter integer C_FRAME_WORDS = 12800,
  parameter integer C_EXTERNAL_LEASE_ENABLE = 0
)(
  input i_clk,
  input i_rst_n,
  input [3:0] i_frame_prepare,
  input [3:0] i_frame_end,
  input [3:0] i_frame_error,
  input [3:0] i_frame_drained,
  input i_hdmi_frame_boundary,
  input [3:0] i_retire_done,
  input [11:0] i_external_slot_held,
  output [63:0] o_display_epoch,
  input [63:0] i_fifo_word_count,
  input [3:0] i_fifo_empty,
  input [1023:0] i_fifo_data,
  input [3:0] i_fifo_almost_full,
  input i_read_urgent,
  input i_engine_ready,
  input i_axi_awready,
  input i_axi_wready,
  output [3:0] o_word_pop,
  output [27:0] o_axi_awaddr,
  output [3:0] o_axi_awlen,
  output o_axi_awvalid,
  output [255:0] o_axi_wdata,
  output o_axi_wvalid,
  output o_burst_done,
  output o_burst_failed,
  output o_busy,
  output [1:0] o_active_source,
  output [3:0] o_grant_valid,
  output [7:0] o_grant_slot,
  output [3:0] o_frame_ready,
  output [3:0] o_frame_aborted,
  output [7:0] o_display_slot,
  output [3:0] o_display_valid,
  output [3:0] o_display_switch,
  output [127:0] o_valid_frame_count,
  output [127:0] o_dropped_frame_count,
  output [127:0] o_display_commit_count,
  output [35:0] o_slot_state,
  output [63:0] o_written_words
);
  `include "quad_frame_address_map.vh"

  wire [3:0] receiving_active;
  reg [3:0] receiving_active_r;
  reg [7:0] receiving_slot_r;
  wire [3:0] manager_hmic_beat;
  wire [3:0] manager_error;
  wire [111:0] source_base;
  wire [3:0] router_request;
  wire [3:0] router_frame_prepare;
  wire [3:0] router_almost_full;
  wire [3:0] router_word_pop;
  reg [3:0] frame_error_seen;
  wire [3:0] within_frame_limit;
  genvar source_index;

  // A source remains eligible after the one-cycle manager grant until its
  // frame is committed or aborted.  This is the ownership fence used by the
  // write router; it cannot change in the middle of a 16-beat burst.
  assign receiving_active = receiving_active_r;
  // The FIFO wrapper exposes a show-ahead head word separately from its
  // internal read-level counter.  Include that visible word when deciding
  // whether a complete burst is available; otherwise an exact 16-word tail
  // would be reported as only 15 words and never launched.
  assign router_request = receiving_active & ~i_fifo_empty & ~manager_error & within_frame_limit &
                          {((i_fifo_word_count[63:48] + 16'd1) >= C_BURST_BEATS),
                           ((i_fifo_word_count[47:32] + 16'd1) >= C_BURST_BEATS),
                           ((i_fifo_word_count[31:16] + 16'd1) >= C_BURST_BEATS),
                           ((i_fifo_word_count[15:0]  + 16'd1) >= C_BURST_BEATS)};
  assign router_almost_full = i_fifo_almost_full & receiving_active;
  // A prepare arriving while a source still owns a RECEIVING slot is a
  // duplicate/early request.  The slot manager will reject it; prevent that
  // rejected pulse from resetting the router's address cursor underneath an
  // in-flight frame.
  assign router_frame_prepare = i_frame_prepare & ~receiving_active;
  // Slot ownership advances only for beats accepted by HMIC WREADY.  The
  // router_word_pop signal is an internal prefetch event and must not count
  // words before AW/W handshakes have actually consumed them.
  assign manager_hmic_beat = (o_axi_wvalid && i_axi_wready) ?
                             (4'b0001 << o_active_source) : 4'd0;
  // 源域 abort 可在寄存 pop 发出后一拍清空 FIFO，空时撤销无效 pop。
  assign o_word_pop = router_word_pop & ~i_fifo_empty;
  assign manager_error = i_frame_error | frame_error_seen |
                         (o_burst_failed ? (4'b0001 << o_active_source) : 4'd0);

  // 错误直到下一次预约才清除；源清空 FIFO 后不会重新发起坏帧 burst。
  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      frame_error_seen <= 4'd0;
    end else begin
      frame_error_seen <= (frame_error_seen | i_frame_error |
                          (o_burst_failed ? (4'b0001 << o_active_source) : 4'd0)) &
                          ~router_frame_prepare;
    end
  end

  generate
    for (source_index = 0; source_index < 4; source_index = source_index + 1) begin: gen_frame_limit
      // 写地址必须止于本帧尺寸，超长输入由 ingress 丢弃并标错。
      assign within_frame_limit[source_index] =
        ({1'b0, o_written_words[source_index * 16 +: 16]} + C_BURST_BEATS <= C_FRAME_WORDS);
    end
  endgenerate

  // Slot bases are fixed by the common address map and selected per source by
  // the slot manager grant.  The BR source remains reserved for source2.
  function [27:0] slot_base;
    input [1:0] source;
    input [1:0] slot;
    begin
      case ({source,slot})
        4'b0000: slot_base = `QUAD_SRC0_SLOT0_BASE;
        4'b0001: slot_base = `QUAD_SRC0_SLOT1_BASE;
        4'b0010: slot_base = `QUAD_SRC0_SLOT2_BASE;
        4'b0100: slot_base = `QUAD_SRC1_SLOT0_BASE;
        4'b0101: slot_base = `QUAD_SRC1_SLOT1_BASE;
        4'b0110: slot_base = `QUAD_SRC1_SLOT2_BASE;
        4'b1000: slot_base = `QUAD_SRC2_SLOT0_BASE;
        4'b1001: slot_base = `QUAD_SRC2_SLOT1_BASE;
        4'b1010: slot_base = `QUAD_SRC2_SLOT2_BASE;
        4'b1100: slot_base = `QUAD_SRC3_SLOT0_BASE;
        4'b1101: slot_base = `QUAD_SRC3_SLOT1_BASE;
        default: slot_base = `QUAD_SRC3_SLOT2_BASE;
      endcase
    end
  endfunction

  assign source_base = {
    slot_base(2'd3, receiving_slot_r[7:6]),
    slot_base(2'd2, receiving_slot_r[5:4]),
    slot_base(2'd1, receiving_slot_r[3:2]),
    slot_base(2'd0, receiving_slot_r[1:0])
  };

  always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
      receiving_active_r <= 4'd0;
      receiving_slot_r <= 8'd0;
    end else begin
      receiving_active_r <= receiving_active_r & ~(o_frame_ready | o_frame_aborted);
      if (|o_grant_valid) begin
        if (o_grant_valid[0]) begin receiving_active_r[0] <= 1'b1; receiving_slot_r[1:0] <= o_grant_slot[1:0]; end
        if (o_grant_valid[1]) begin receiving_active_r[1] <= 1'b1; receiving_slot_r[3:2] <= o_grant_slot[3:2]; end
        if (o_grant_valid[2]) begin receiving_active_r[2] <= 1'b1; receiving_slot_r[5:4] <= o_grant_slot[5:4]; end
        if (o_grant_valid[3]) begin receiving_active_r[3] <= 1'b1; receiving_slot_r[7:6] <= o_grant_slot[7:6]; end
      end
    end
  end

  quad_frame_slot_manager #(.C_FRAME_WORDS(C_FRAME_WORDS), .C_EXTERNAL_LEASE_ENABLE(C_EXTERNAL_LEASE_ENABLE)) u_slot_manager(
    .i_clk(i_clk), .i_rst_n(i_rst_n), .i_frame_prepare(i_frame_prepare),
    .o_grant_valid(o_grant_valid), .o_grant_slot(o_grant_slot), .o_grant_toggle(),
    .i_hmic_write_beat(manager_hmic_beat), .i_frame_end(i_frame_end),
    .i_write_stream_drained(i_frame_drained), .i_overflow(manager_error),
    .i_error(4'd0), .i_hdmi_frame_boundary(i_hdmi_frame_boundary),
    .i_retire_done(i_retire_done), .o_frame_ready(o_frame_ready),
    .i_external_slot_held(i_external_slot_held), .o_display_epoch(o_display_epoch),
    .o_frame_aborted(o_frame_aborted), .o_display_slot(o_display_slot),
    .o_display_valid(o_display_valid), .o_display_switch(o_display_switch),
    .o_valid_frame_count(o_valid_frame_count),
    .o_dropped_frame_count(o_dropped_frame_count),
    .o_display_commit_count(o_display_commit_count), .o_slot_state(o_slot_state),
    .o_receiving_slot(), .o_written_words(o_written_words)
  );

  multi_source_ddr_write_router #(
    .C_ADDR_WIDTH(C_ADDR_WIDTH), .C_BURST_BEATS(C_BURST_BEATS)
  ) u_router(
    .i_clk(i_clk), .i_rst_n(i_rst_n), .i_frame_prepare(router_frame_prepare),
    .i_frame_error(manager_error),
    .i_request(router_request), .i_almost_full(router_almost_full),
    .i_read_urgent(i_read_urgent), .i_engine_ready(i_engine_ready),
    .i_fifo_word_count(i_fifo_word_count), .i_fifo_empty(i_fifo_empty),
    .i_fifo_data(i_fifo_data), .i_source_base(source_base),
    .o_word_pop(router_word_pop), .o_axi_awaddr(o_axi_awaddr),
    .o_axi_awlen(o_axi_awlen), .o_axi_awvalid(o_axi_awvalid),
    .i_axi_awready(i_axi_awready), .o_axi_wdata(o_axi_wdata),
    .o_axi_wvalid(o_axi_wvalid), .i_axi_wready(i_axi_wready),
    .o_burst_done(o_burst_done), .o_burst_failed(o_burst_failed),
    .o_busy(o_busy), .o_active_source(o_active_source)
  );
endmodule
