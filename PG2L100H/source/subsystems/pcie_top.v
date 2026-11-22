//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: PCIe 视频采集与主机交互层次封装，连接独立 PCIe x4 物理接口、参考时钟、PERST#、场景事件和行为快照。
// Module Name: pcie_top
// File Name: source/subsystems/pcie_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// PCIe 视频采集与主机交互层次封装，连接独立 PCIe x4 物理接口、参考时钟、PERST#、场景事件和行为快照。
// 采集共享算法层 A 的去雾输出，RGB 与 VS/HS/DE/标签同拍；PCIe 传输数据不含 HDMI 专用算法后级或 HUD 叠加。
//
// Dependencies: pcie_dma_subsystem 及其登记的 PCIe/DMA IP。
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
// Create Date: 2026-10-16
// Design Name: AVM PCIe hierarchy refactor
// Module Name: pcie_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Structural extraction; original clocks, handshakes and latency retained.
// Dependencies: Original project modules instantiated below.
// Revision: 0.01 - Extracted from top_design.
// Revision: 0.02 - 2026-10-21 Shared scene events and reference-timed behavior snapshot.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_top(
	input i_ref_clk,
	input i_image_control_clk,
	input i_image_control_rst_n,
	output o_isp_ext_valid,
	output o_isp_ext_query,
	output [519:0] o_isp_ext_config,
	input i_isp_ext_ready,
	input i_isp_ext_done,
	input [7:0] i_isp_ext_status,
	input [519:0] i_isp_ext_snapshot,
	input [15:0] i_isp_ext_epoch,
	input [1:0] i_isp_ext_source,
	output o_algo_ext_valid,
	output o_algo_ext_query,
	output [63:0] o_algo_ext_config,
	input i_algo_ext_ready,
	input i_algo_ext_done,
	input [7:0] i_algo_ext_status,
	input [63:0] i_algo_ext_snapshot,
	input [15:0] i_algo_ext_epoch,
	input [1:0] i_algo_ext_source,
	input [63:0] i_fiber_lane_snapshot,
	input i_fiber_lane_valid,
	input i_fiber_lane_stale,
	input i_control_rst_n,
	input i_scene_commit_valid,
	input [111:0] i_scene_commit_payload,
	output [175:0] o_display_snapshot,
	input i_hdmi_pixel_clk,
	input i_hdmi_pixel_rst_n,
	input [23:0] i_dehaze_algo_rgb,
	input [159:0] i_dehaze_algo_tag,
	output o_hud_wr_en,
	output [11:0] o_hud_wr_addr,
	output [31:0] o_hud_wr_data,
	output [3:0] o_hud_wr_strb,
	output [11:0] o_hud_rd_addr,
	input [127:0] i_hud_rd_window,
	output o_pcie_pclk_div2,
	output o_pcie_core_rst_n,
	output o_pcie_link_up,
	input i_button_rst_n,
	input i_pcie_ref_clk_p,
	input i_pcie_ref_clk_n,
	input i_pcie_perst_n,
	input [3:0] i_pcie_rxp,
	input [3:0] i_pcie_rxn,
	input i_action_clk,
	input i_action_valid,
	input [15:0] i_action_sequence,
	input [2:0] i_action_type,
	input i_action_request,
	output o_action_ready,
	output o_action_online,
	output [3:0] o_pcie_txp,
	output [3:0] o_pcie_txn,
	output [31:0] o_behavior_result,
	output [31:0] o_behavior_effective
);
	wire hdmi_pixel_clk;
	wire hdmi_pixel_rst_n;
	wire [23:0] dehaze_algo_rgb;
	wire [159:0] dehaze_algo_tag;
	wire hud_wr_en;
	wire [11:0] hud_wr_addr;
	wire [31:0] hud_wr_data;
	wire [3:0] hud_wr_strb;
	wire [11:0] hud_rd_addr;
	wire [127:0] hud_rd_window;
	wire pcie_pclk;
	wire pcie_pclk_div2;
	wire pcie_core_rst_n;
	wire pcie_smlh_link_up;
	wire pcie_rdlh_link_up;
	wire [4:0] pcie_ltssm_state;
	wire pcie_link_up;
	wire [31:0] behavior_result;
	wire [31:0] behavior_effective;

	assign hdmi_pixel_clk = i_hdmi_pixel_clk;
	assign hdmi_pixel_rst_n = i_hdmi_pixel_rst_n;
	assign dehaze_algo_rgb = i_dehaze_algo_rgb;
	assign dehaze_algo_tag = i_dehaze_algo_tag;
	assign o_hud_wr_en = hud_wr_en;
	assign o_hud_wr_addr = hud_wr_addr;
	assign o_hud_wr_data = hud_wr_data;
	assign o_hud_wr_strb = hud_wr_strb;
	assign o_hud_rd_addr = hud_rd_addr;
	assign hud_rd_window = i_hud_rd_window;
	assign o_pcie_pclk_div2 = pcie_pclk_div2;
	assign o_pcie_core_rst_n = pcie_core_rst_n;
	assign o_pcie_link_up = pcie_link_up;
	assign o_behavior_result = behavior_result;
	assign o_behavior_effective = behavior_effective;

	// quad_base_* 仍保留为四宫格原始取样点和旁路观察点；PCIe 发送端接
	// 算法层 A 的去雾延迟同步流。dehaze_algo_tag 的 VS/HS/DE 与 RGB 同拍，
	// 不接算法层 B 或 HDMI HUD，确保 PCIe 只获得去雾结果。
	pcie_dma_subsystem u_pcie_dma_subsystem(
		.i_ref_clk(i_ref_clk),
		.i_image_control_clk(i_image_control_clk),
		.i_image_control_rst_n(i_image_control_rst_n),
		.o_isp_ext_valid(o_isp_ext_valid),
		.o_isp_ext_query(o_isp_ext_query),
		.o_isp_ext_config(o_isp_ext_config),
		.i_isp_ext_ready(i_isp_ext_ready),
		.i_isp_ext_done(i_isp_ext_done),
		.i_isp_ext_status(i_isp_ext_status),
		.i_isp_ext_snapshot(i_isp_ext_snapshot),
		.i_isp_ext_epoch(i_isp_ext_epoch),
		.i_isp_ext_source(i_isp_ext_source),
		.o_algo_ext_valid(o_algo_ext_valid),
		.o_algo_ext_query(o_algo_ext_query),
		.o_algo_ext_config(o_algo_ext_config),
		.i_algo_ext_ready(i_algo_ext_ready),
		.i_algo_ext_done(i_algo_ext_done),
		.i_algo_ext_status(i_algo_ext_status),
		.i_algo_ext_snapshot(i_algo_ext_snapshot),
		.i_algo_ext_epoch(i_algo_ext_epoch),
		.i_algo_ext_source(i_algo_ext_source),
		.i_fiber_lane_snapshot(i_fiber_lane_snapshot),
		.i_fiber_lane_valid(i_fiber_lane_valid),
		.i_fiber_lane_stale(i_fiber_lane_stale),
		.i_control_rst_n(i_control_rst_n),
		.i_scene_commit_valid(i_scene_commit_valid),
		.i_scene_commit_payload(i_scene_commit_payload),
		.o_display_snapshot(o_display_snapshot),
		.i_video_pixel_clk(hdmi_pixel_clk),
		.i_video_pixel_rst_n(hdmi_pixel_rst_n),
		.i_video_vs(dehaze_algo_tag[159]),
		.i_video_hs(dehaze_algo_tag[158]),
		.i_video_de(dehaze_algo_tag[157]),
		.i_video_rgb(dehaze_algo_rgb),
		.i_pcie_ref_clk_p(i_pcie_ref_clk_p),
		.i_pcie_ref_clk_n(i_pcie_ref_clk_n),
		.i_pcie_button_rst_n(i_button_rst_n),
		.i_pcie_perst_n(i_pcie_perst_n),
		.i_pcie_rxp(i_pcie_rxp),
		.i_pcie_rxn(i_pcie_rxn),
		.i_action_clk(i_action_clk),
		.i_action_valid(i_action_valid),
		.i_action_sequence(i_action_sequence),
		.i_action_type(i_action_type),
		.i_action_request(i_action_request),
		.o_action_ready(o_action_ready),
		.o_action_online(o_action_online),
		.o_pcie_txp(o_pcie_txp),
		.o_pcie_txn(o_pcie_txn),
		.o_pcie_pclk(pcie_pclk),
		.o_pcie_pclk_div2(pcie_pclk_div2),
		.o_pcie_core_rst_n(pcie_core_rst_n),
		.o_pcie_smlh_link_up(pcie_smlh_link_up),
		.o_pcie_rdlh_link_up(pcie_rdlh_link_up),
		.o_pcie_ltssm_state(pcie_ltssm_state),
		.o_pcie_link_up(pcie_link_up),
		.o_hud_wr_en(hud_wr_en),
		.o_hud_wr_addr(hud_wr_addr),
		.o_hud_wr_data(hud_wr_data),
		.o_hud_wr_strb(hud_wr_strb),
		.o_hud_rd_addr(hud_rd_addr),
		.i_hud_rd_window(hud_rd_window),
		.o_behavior_result(behavior_result),
		.o_behavior_effective(behavior_effective)
	);


endmodule
