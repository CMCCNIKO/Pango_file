//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 统一封装 HDMI-IN/MS7200、OV5640 CMOS1 和 SFP1/HSST CH2 视频输入；HDMI-IN 由 1920x1080 裁剪/抽样为 640x320 RGB565。
// Module Name: video_sources_top
// File Name: source/subsystems/video_sources_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 统一封装 HDMI-IN/MS7200、OV5640 CMOS1 和 SFP1/HSST CH2 视频输入；HDMI-IN 由 1920x1080 裁剪/抽样为 640x320 RGB565。
// 各输入保持原采集时钟，通过帧授权与官方异步 FIFO 输出 DDR 域 256-bit word、帧结束/错误事件；同时解析帧间 LANE-META-V1 车道属性并形成独立 27MHz 系统快照。
//
// Dependencies: hdmi_rx_frontend、hdmi_source3_word_cdc、ov5640_source1_word_cdc、ov5640_sccb_init、fiber_hsst_rx_adapter、fiber_word_align、fiber_video_rx_assembler、fiber_source2_word_cdc、fiber_lane_meta_rx、fiber_lane_snapshot_cdc。
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
//
// Create Date: 2026-10-16
// Design Name: Quad Video Mosaic
// Module Name: video_sources_top
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 视频源输入子系统统一封装。
//              保留 HDMI-IN、OV5640 CMOS1、SFP/HSST 光纤接收三条原有
//              数据通路，以及它们到 DDR 域的原有帧授权和 word-pop 接口。
//              本文件只整理层次，不改变时钟、复位、参数或数据行为。
// Dependencies: hdmi_rx_frontend、hdmi_source3_word_cdc、
//               ov5640_source1_word_cdc、ov5640_sccb_init、
//               fiber_hsst_rx_adapter、fiber_word_align、
//               fiber_video_rx_assembler、fiber_source2_word_cdc。
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments: 厂商 HSST/FIFO IP 仍由原工程登记和提供，本 wrapper
//                      不复制或替代任何厂商生成文件。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module video_sources_top
#(
	parameter integer C_Q8_MULTI_SOURCE = 1,
	parameter integer C_HDMI_SOURCE3_ENABLE = 1,
	parameter integer C_CMOS1_ENABLE = 1,
	parameter integer C_FIBER_SOURCE2_ENABLE = 1
)
(
	// 系统、外部视频源和板级复位
	input i_sys_clk,
	input i_button_rst_n,
	input i_hdmi_rx_pclk,
	input i_hdmi_rx_vs,
	input i_hdmi_rx_hs,
	input i_hdmi_rx_de,
	input [23:0] i_hdmi_rx_rgb,
	input i_cmos1_pclk,
	input i_cmos1_vsync,
	input i_cmos1_href,
	input [7:0] i_cmos1_data,
	inout io_cmos1_scl,
	inout io_cmos1_sda,
	output o_cmos1_reset,

	// SFP0 旧端口名保留；实际 HSST RX 通路仍对应原工程的 Channel 2。
	input i_p_refckn_1,
	input i_p_refckp_1,
	input i_sfp0_rxn,
	input i_sfp0_rxp,
	input i_sfp0_los,

	// DDR 域读出授权和时钟。父 top 可将 i_ddr_rst_n 接原来的
	// i_button_rst_n，以完全保持当前 top_design 行为。
	input [3:0] i_quad_grant_toggle,
	input [3:0] i_quad_word_pop,
	input i_ddr_clk,
	input i_ddr_rst_n,

	// HDMI-IN 规范化流和监测状态
	output o_hdmi_rx_tile_valid,
	output [15:0] o_hdmi_rx_tile_rgb565,
	output o_hdmi_rx_tile_frame_start,
	output o_hdmi_rx_tile_frame_end,
	output o_hdmi_rx_tile_frame_error,
	output o_hdmi_rx_vs_active,
	output o_hdmi_rx_hs_active,
	output o_hdmi_rx_de_active,
	output [255:0] o_hdmi_source3_word_data,
	output o_hdmi_source3_word_empty,
	output [15:0] o_hdmi_source3_wr_level,
	output [15:0] o_hdmi_source3_rd_level,
	output o_hdmi_source3_prepare_ddr,
	output o_hdmi_source3_complete_ddr,
	output o_hdmi_source3_abort_ddr,

	// OV5640 CMOS1 规范化流、帧事件和 SCCB 状态
	output [255:0] o_cmos1_word_data,
	output o_cmos1_word_empty,
	output [15:0] o_cmos1_wr_level,
	output [15:0] o_cmos1_rd_level,
	output o_cmos1_prepare_ddr,
	output o_cmos1_complete_ddr,
	output o_cmos1_abort_ddr,
	output o_cmos1_sccb_scl,
	output o_cmos1_sccb_sda_drive_low,
	output o_cmos1_sccb_done,
	output o_cmos1_sccb_error,

	// 光纤 HSST、字节对齐、协议组帧和 source2 DDR CDC
	output o_fiber_rx_clk,
	output o_fiber_pll_ready,
	output o_fiber_rx_lane_done,
	output o_fiber_rx_lane_ready,
	output [31:0] o_fiber_rxd,
	output [3:0] o_fiber_rxk,
	output [3:0] o_fiber_decode_error,
	output [3:0] o_fiber_disparity_error,
	output o_fiber_signal_detect,
	output o_fiber_cdr_align,
	output [1:0] o_fiber_watchdog_status,
	output o_fiber_aligned_valid,
	output [31:0] o_fiber_aligned_rxd,
	output [3:0] o_fiber_aligned_rxk,
	output o_fiber_align_lock,
	output o_fiber_align_error,
	output o_fiber_frame_start,
	output o_fiber_frame_end,
	output o_fiber_frame_error,
	output o_fiber_word256_valid,
	output [255:0] o_fiber_word256_data,
	output [9:0] o_fiber_pixel_x,
	output [8:0] o_fiber_pixel_y,
	output [31:0] o_fiber_good_frame_count,
	output [31:0] o_fiber_bad_frame_count,
	output [255:0] o_fiber_source2_word_data,
	output o_fiber_source2_word_empty,
	output [15:0] o_fiber_source2_wr_level,
	output [15:0] o_fiber_source2_rd_level,
	output o_fiber_source2_prepare_ddr,
	output o_fiber_source2_complete_ddr,
	output o_fiber_source2_abort_ddr,
	// 独立27MHz域车道快照，不依赖DDR grant或PCIe会话。
	output [63:0] o_fiber_lane_snapshot,
	output o_fiber_lane_valid,
	output o_fiber_lane_stale
);
	wire fiber_aligned_phy_error;
	wire fiber_resync;
	wire fiber_video_frame_active;
	wire fiber_lane_snapshot_valid_rx;
	wire [63:0] fiber_lane_snapshot_rx;
	reg [2:0] fiber_meta_reset_sync;
	// 新增控制逻辑的复位在恢复时钟域同步释放。
	always@(posedge o_fiber_rx_clk or negedge i_button_rst_n)begin
		if(!i_button_rst_n)begin
			fiber_meta_reset_sync <= 3'd0;
		end else begin
			fiber_meta_reset_sync <= {fiber_meta_reset_sync[1:0],1'b1};
		end
	end

	// HDMI-IN 1920x1080 -> crop/decimate -> 640x320 RGB565。
	hdmi_rx_frontend #(
		.C_SOURCE_WIDTH(1920),
		.C_SOURCE_HEIGHT(1080),
		.C_CROP_TOP(60),
		.C_TILE_WIDTH(640),
		.C_TILE_HEIGHT(320),
		.C_H_DECIM(3),
		.C_V_DECIM(3)
	) u_hdmi_rx_frontend(
		.i_pclk(i_hdmi_rx_pclk),
		.i_rst_n(i_button_rst_n),
		.i_vs(i_hdmi_rx_vs),
		.i_hs(i_hdmi_rx_hs),
		.i_de(i_hdmi_rx_de),
		.i_rgb888(i_hdmi_rx_rgb),
		.o_pixel_valid(o_hdmi_rx_tile_valid),
		.o_pixel_rgb565(o_hdmi_rx_tile_rgb565),
		.o_frame_start(o_hdmi_rx_tile_frame_start),
		.o_frame_end(o_hdmi_rx_tile_frame_end),
		.o_frame_error(o_hdmi_rx_tile_frame_error),
		.o_rx_vs_active(o_hdmi_rx_vs_active),
		.o_rx_hs_active(o_hdmi_rx_hs_active),
		.o_rx_de_active(o_hdmi_rx_de_active)
	);

	// 保留原顶层的 i_button_rst_n 作为 source3 DDR reset 的接入语义；
	// 父 top 通过 i_ddr_rst_n 接入该相同信号即可，不使用 quad_ddr_rst_n。
	hdmi_source3_word_cdc #(
		.C_USE_GRANT_TOKEN(1)
	) u_hdmi_source3_word_cdc(
		.i_rx_clk(i_hdmi_rx_pclk),
		.i_rx_rst_n(i_button_rst_n),
		.i_frame_accept((C_Q8_MULTI_SOURCE != 0) && (C_HDMI_SOURCE3_ENABLE != 0)),
		.i_grant_toggle(i_quad_grant_toggle[3]),
		.i_frame_start(o_hdmi_rx_tile_frame_start),
		.i_frame_end(o_hdmi_rx_tile_frame_end),
		.i_frame_error(o_hdmi_rx_tile_frame_error),
		.i_pixel_valid(o_hdmi_rx_tile_valid),
		.i_pixel_rgb565(o_hdmi_rx_tile_rgb565),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rst_n),
		.i_word_pop(i_quad_word_pop[3]),
		.o_word_data(o_hdmi_source3_word_data),
		.o_word_empty(o_hdmi_source3_word_empty),
		.o_wr_water_level(o_hdmi_source3_wr_level),
		.o_rd_water_level(o_hdmi_source3_rd_level),
		.o_frame_prepare_ddr(o_hdmi_source3_prepare_ddr),
		.o_frame_complete_ddr(o_hdmi_source3_complete_ddr),
		.o_frame_abort_ddr(o_hdmi_source3_abort_ddr)
	);

	// CMOS1 DVP -> RGB565 tile -> source1 word CDC。
	ov5640_source1_word_cdc #(
		.C_FRAME_ACCEPT((C_Q8_MULTI_SOURCE != 0) && (C_CMOS1_ENABLE != 0)),
		.C_USE_GRANT_TOKEN(1)
	) u_ov5640_source1_word_cdc(
		.i_grant_toggle(i_quad_grant_toggle[1]),
		.i_cmos_pclk(i_cmos1_pclk),
		.i_cmos_rst_n(i_button_rst_n),
		.i_cmos_vsync(i_cmos1_vsync),
		.i_cmos_href(i_cmos1_href),
		.i_cmos_data(i_cmos1_data),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rst_n),
		.i_word_pop(i_quad_word_pop[1]),
		.o_word_data(o_cmos1_word_data),
		.o_word_empty(o_cmos1_word_empty),
		.o_wr_water_level(o_cmos1_wr_level),
		.o_rd_water_level(o_cmos1_rd_level),
		.o_frame_prepare_ddr(o_cmos1_prepare_ddr),
		.o_frame_complete_ddr(o_cmos1_complete_ddr),
		.o_frame_abort_ddr(o_cmos1_abort_ddr)
	);

	// SCCB 保留原三态边界；SDA 仅在现有 master 请求时驱动低电平。
	ov5640_sccb_init u_ov5640_sccb_init(
		.i_clk(i_sys_clk),
		.i_rst_n(i_button_rst_n),
		.i_sda(io_cmos1_sda),
		.o_scl(o_cmos1_sccb_scl),
		.o_sda_drive_low(o_cmos1_sccb_sda_drive_low),
		.o_busy(),
		.o_done(o_cmos1_sccb_done),
		.o_error(o_cmos1_sccb_error)
	);
	assign io_cmos1_scl = o_cmos1_sccb_scl;
	assign io_cmos1_sda = o_cmos1_sccb_sda_drive_low ? 1'b0 : 1'bz;
	assign o_cmos1_reset = i_button_rst_n;

	// HSST 通道、恢复时钟域字节对齐和视频协议组帧。
	fiber_hsst_rx_adapter u_fiber_hsst_rx_adapter(
		.i_free_clk(i_sys_clk),
		.i_rst_n(i_button_rst_n),
		.i_refckn(i_p_refckn_1),
		.i_refckp(i_p_refckp_1),
		.i_rxn(i_sfp0_rxn),
		.i_rxp(i_sfp0_rxp),
		.i_sfp_los(i_sfp0_los),
		.o_rx_clk(o_fiber_rx_clk),
		.o_pll_ready(o_fiber_pll_ready),
		.o_rx_lane_done(o_fiber_rx_lane_done),
		.o_rx_lane_ready(o_fiber_rx_lane_ready),
		.o_rxd(o_fiber_rxd),
		.o_rxk(o_fiber_rxk),
		.o_decode_error(o_fiber_decode_error),
		.o_disparity_error(o_fiber_disparity_error),
		.o_signal_detect(o_fiber_signal_detect),
		.o_cdr_align(o_fiber_cdr_align),
		.o_watchdog_status(o_fiber_watchdog_status)
	);

	fiber_word_align u_fiber_word_align(
		.i_clk(o_fiber_rx_clk),
		.i_rst_n(i_button_rst_n),
		.i_rx_valid(o_fiber_rx_lane_ready),
		.i_rxd(o_fiber_rxd),
		.i_rxk(o_fiber_rxk),
		.i_decode_error(o_fiber_decode_error),
		.i_disparity_error(o_fiber_disparity_error),
		.o_aligned_valid(o_fiber_aligned_valid),
		.o_aligned_rxd(o_fiber_aligned_rxd),
		.o_aligned_rxk(o_fiber_aligned_rxk),
		.o_lock(o_fiber_align_lock),
		.o_align_error(o_fiber_align_error),
		.o_aligned_phy_error(fiber_aligned_phy_error),
		.o_resync(fiber_resync)
	);

	fiber_video_rx_assembler u_fiber_video_rx_assembler(
		.i_clk(o_fiber_rx_clk),
		.i_rst_n(i_button_rst_n),
		.i_link_ready(o_fiber_rx_lane_ready),
		.i_phy_error(fiber_aligned_phy_error),
		.i_resync(fiber_resync),
		.i_word_valid(o_fiber_aligned_valid),
		.i_rxd(o_fiber_aligned_rxd),
		.i_rxk(o_fiber_aligned_rxk),
		.o_frame_start(o_fiber_frame_start),
		.o_frame_end(o_fiber_frame_end),
		.o_frame_error(o_fiber_frame_error),
		.o_word256_valid(o_fiber_word256_valid),
		.o_word256_data(o_fiber_word256_data),
		.o_x(o_fiber_pixel_x),
		.o_y(o_fiber_pixel_y),
		.o_good_frame_count(o_fiber_good_frame_count),
		.o_bad_frame_count(o_fiber_bad_frame_count),
		.o_frame_active(fiber_video_frame_active)
	);

	// 帧间四字包旁路监听；不会回压像素流，也不向视频FIFO写属性字。
	fiber_lane_meta_rx u_fiber_lane_meta_rx(
		.i_clk(o_fiber_rx_clk),
		.i_rst_n(fiber_meta_reset_sync[2]),
		.i_link_ready(o_fiber_rx_lane_ready),
		.i_stream_error(fiber_aligned_phy_error),
		.i_resync(fiber_resync),
		.i_video_frame_active(fiber_video_frame_active),
		.i_word_valid(o_fiber_aligned_valid),
		.i_rxd(o_fiber_aligned_rxd),
		.i_rxk(o_fiber_aligned_rxk),
		.o_snapshot_valid(fiber_lane_snapshot_valid_rx),
		.o_snapshot(fiber_lane_snapshot_rx)
	);
	fiber_lane_snapshot_cdc u_fiber_lane_snapshot_cdc(
		.i_rx_clk(o_fiber_rx_clk),
		.i_rx_rst_n(i_button_rst_n),
		.i_rx_link_ready(o_fiber_rx_lane_ready),
		.i_snapshot_valid(fiber_lane_snapshot_valid_rx),
		.i_snapshot(fiber_lane_snapshot_rx),
		.i_sys_clk(i_sys_clk),
		.i_sys_rst_n(i_button_rst_n),
		.o_update(),
		.o_snapshot(o_fiber_lane_snapshot),
		.o_valid(o_fiber_lane_valid),
		.o_stale(o_fiber_lane_stale)
	);

	fiber_source2_word_cdc u_fiber_source2_word_cdc(
		.i_rx_clk(o_fiber_rx_clk),
		.i_rx_rst_n(i_button_rst_n),
		.i_frame_accept((C_Q8_MULTI_SOURCE != 0) && (C_FIBER_SOURCE2_ENABLE != 0)),
		.i_grant_toggle(i_quad_grant_toggle[2]),
		.i_frame_start(o_fiber_frame_start),
		.i_frame_end(o_fiber_frame_end),
		.i_frame_error(o_fiber_frame_error),
		.i_word_valid(o_fiber_word256_valid),
		.i_word_data(o_fiber_word256_data),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rst_n),
		.i_word_pop(i_quad_word_pop[2]),
		.o_word_data(o_fiber_source2_word_data),
		.o_word_empty(o_fiber_source2_word_empty),
		.o_wr_water_level(o_fiber_source2_wr_level),
		.o_rd_water_level(o_fiber_source2_rd_level),
		.o_frame_prepare_ddr(o_fiber_source2_prepare_ddr),
		.o_frame_complete_ddr(o_fiber_source2_complete_ddr),
		.o_frame_abort_ddr(o_fiber_source2_abort_ddr)
	);

endmodule
