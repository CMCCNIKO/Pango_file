//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: AVM 与双路监视总封装，接收 UDP 路由控制、管理各读者的帧槽 lease，独立读取监视源并将左侧 AVM 与右侧监视画面合成输出。
// Module Name: avm_monitor_top
// File Name: source/subsystems/avm_monitor_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// AVM 与双路监视总封装，接收 UDP 路由控制、管理各读者的帧槽 lease，独立读取监视源并将左侧 AVM 与右侧监视画面合成输出。
// 按 C_LUT_SOURCE 互斥选择 UDP、用户 SPI0 Flash 或 SD LUT 装载，装载完成才允许 AVM 读取整表；当前默认使用 SD。
// LUT 角色 [左,前,右,后] 对应物理源 [HDMI,HSST,Ethernet,OV5640]；统一仲裁 HMIC 读请求及 quad/LUT/AVM 写事务。
//
// Dependencies: udp_control_magic_dispatch、monitor_route_control、udp_control_ack_arbiter、video_source_slot_leases、monitor_display_pipeline、hdmi_ddr_read_arbiter、avm_monitor_compositor、avm_lut_flash_reader_gtp、avm_lut_sd_loader、avm_lut_udp_loader、avm_single_beat_writer、ddr_write_owner_arbiter、avm_video_subsystem。
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
// Module Name: avm_monitor_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Structural extraction; original clocks, handshakes and latency retained.
// Dependencies: Original project modules instantiated below.
// Revision: 0.01 - Extracted from top_design.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_monitor_top #(
	parameter integer C_FLASH_ENABLE = 1,
	parameter [23:0] C_FLASH_LUT_BASE_ADDR = 24'h800000,
	parameter integer C_FLASH_SELECT = 0,
	parameter integer C_LUT_SOURCE = 2
)(
	input i_rgmii_clk,
	input i_udp_ctrl_valid,
	input [7:0] i_udp_ctrl_data,
	input [15:0] i_udp_ctrl_length,
	input i_udp_ctrl_start,
	input i_udp_ctrl_end,
	input i_udp_ctrl_error,
	input i_algo_common_reset_n,
	input i_algo_net_rst_n,
	input i_algo_pixel_rst_n,
	input i_algo_ack_valid,
	input [143:0] i_algo_ack_payload,
	input [15:0] i_algo_ack_length,
	output o_algo_ack_ready,
	input i_isp_ack_valid,
	input [143:0] i_isp_ack_payload,
	input [15:0] i_isp_ack_length,
	output o_isp_ack_ready,
	output o_isp_start,
	output o_isp_valid,
	output [7:0] o_isp_data,
	output o_isp_end,
	output [15:0] o_isp_length,
	output o_isp_error,
	input i_hdmi_pixel_clk,
	input [159:0] i_algo_output_tag,
	input [23:0] i_hdmi_algo_rgb,
	input [8:0] i_hud_state,
	input i_c5_ddr_clk,
	input i_c5_ddr_init_done,
	output [27:0] o_c5_awaddr,
	output [3:0] o_c5_awlen,
	input i_c5_awready,
	output o_c5_awvalid,
	output [255:0] o_c5_wdata,
	output [31:0] o_c5_wstrb,
	input i_c5_wready,
	output [27:0] o_c5_araddr,
	output [3:0] o_c5_arlen,
	input i_c5_arready,
	output o_c5_arvalid,
	input [255:0] i_c5_rdata,
	input i_c5_rvalid,
	input [7:0] i_quad_display_slot,
	input [3:0] i_quad_display_valid,
	input [3:0] i_quad_retire_done,
	input i_quad_snapshot,
	input i_quad_read_urgent,
	input i_quad_ddr_rst_n,
	output o_avm_render_busy,
	output o_avm_frame_toggle_dbg,
	output o_avm_renderer_ready_dbg,
	output o_avm_free_available_dbg,
	output o_avm_pixel_alive_dbg,
	output o_avm_sd_busy_dbg,
	output o_avm_sd_ready_dbg,
	output [7:0] o_avm_sd_error_dbg,
	output [31:0] o_avm_sd_committed_beats_dbg,
	output [31:0] o_avm_sd_loaded_bytes_dbg,
	input [27:0] i_avm_quad_awaddr,
	input [3:0] i_avm_quad_awlen,
	input i_avm_quad_awvalid,
	input [255:0] i_avm_quad_wdata,
	input [31:0] i_avm_quad_wstrb,
	output [2:0] o_avm_write_client_awready,
	output [2:0] o_avm_write_client_wready,
	output o_avm_pixel_valid,
	output o_monitor_algo_start,
	output o_monitor_algo_valid,
	output o_monitor_algo_end,
	output o_monitor_algo_error,
	output [7:0] o_monitor_algo_data,
	output [15:0] o_monitor_algo_length,
	output o_monitor_mux_ack_valid,
	input i_monitor_mux_ack_ready,
	output [143:0] o_monitor_mux_ack_payload,
	output [15:0] o_monitor_mux_ack_length,
	input i_behavior_ack_valid,
	input [143:0] i_behavior_ack_payload,
	input [15:0] i_behavior_ack_length,
	output o_behavior_ack_ready,
	output [159:0] o_monitor_composite_tag,
	output [23:0] o_monitor_composite_rgb,
	output [8:0] o_monitor_composite_hud,
	input [27:0] i_monitor_quad_araddr,
	input [3:0] i_monitor_quad_arlen,
	input i_monitor_quad_arvalid,
	output o_monitor_quad_arready,
	output o_monitor_quad_rvalid,
	output [11:0] o_monitor_slot_held,
	output [11:0] o_monitor_lease_ready,
	output o_avm_pixel_frame_ready,
	input [63:0] i_monitor_display_epoch,
	input i_sys_clk,
	input i_button_rst_n,
	output o_flash_cs_n,
	output o_flash1_cs_n,
	inout [3:0] io_flash_dq,
	output o_sd_clk,
	output o_sd_mosi,
	input i_sd_miso,
	output o_sd_cs
);
	wire rgmii_clk;
	wire udp_ctrl_valid;
	wire [7:0] udp_ctrl_data;
	wire [15:0] udp_ctrl_length;
	wire udp_ctrl_start;
	wire udp_ctrl_end;
	wire udp_ctrl_error;
	wire algo_common_reset_n;
	wire algo_net_rst_n;
	wire algo_pixel_rst_n;
	wire algo_ack_valid;
	wire [143:0] algo_ack_payload;
	wire [15:0] algo_ack_length;
	wire algo_ack_ready;
	wire behavior_ack_ready;
	wire hdmi_pixel_clk;
	wire [159:0] algo_output_tag;
	wire [23:0] hdmi_algo_rgb;
	wire [8:0] hud_state;
	wire c5_ddr_clk;
	wire c5_ddr_init_done;
	wire [27:0] c5_awaddr;
	wire [3:0] c5_awlen;
	wire c5_awready;
	wire c5_awvalid;
	wire [255:0] c5_wdata;
	wire [31:0] c5_wstrb;
	wire c5_wready;
	wire [27:0] c5_araddr;
	wire [3:0] c5_arlen;
	wire c5_arready;
	wire c5_arvalid;
	wire [255:0] c5_rdata;
	wire c5_rvalid;
	wire [7:0] quad_display_slot;
	wire [3:0] quad_display_valid;
	wire [3:0] quad_retire_done;
	wire quad_snapshot;
	wire quad_read_urgent;
	wire quad_ddr_rst_n;
	reg [1:0] avm_lut_net_reset_sync;
	wire avm_lut_net_rstn;
	wire avm_lut_start;
	wire avm_lut_valid;
	wire avm_lut_end;
	wire avm_lut_error;
	wire [7:0] avm_lut_data;
	wire [15:0] avm_lut_length;
	wire avm_lut_ack_valid;
	wire avm_lut_ack_ready;
	wire [143:0] avm_lut_ack_payload;
	wire [15:0] avm_lut_ack_length;
	wire avm_lut_ready;
	wire avm_lut_update_pending;
	wire avm_render_busy;
	wire avm_frame_toggle_dbg;
	wire avm_renderer_ready_dbg;
	wire avm_free_available_dbg;
	wire avm_pixel_alive_dbg;
	wire avm_sd_busy_dbg;
	wire avm_sd_ready_dbg;
	wire [7:0] avm_sd_error_dbg;
	wire [31:0] avm_sd_committed_beats_dbg;
	wire [31:0] avm_sd_loaded_bytes_dbg;
	wire [27:0] avm_quad_awaddr;
	wire [27:0] avm_lut_awaddr;
	wire [27:0] avm_render_awaddr;
	wire [3:0] avm_quad_awlen;
	wire [3:0] avm_lut_awlen;
	wire [3:0] avm_render_awlen;
	wire avm_quad_awvalid;
	wire avm_lut_awvalid;
	wire avm_render_awvalid;
	wire [255:0] avm_quad_wdata;
	wire [255:0] avm_lut_wdata;
	wire [255:0] avm_render_wdata;
	wire [31:0] avm_quad_wstrb;
	wire [31:0] avm_lut_wstrb;
	wire [31:0] avm_render_wstrb;
	wire avm_lut_wvalid;
	wire avm_render_wvalid;
	wire [2:0] avm_write_client_awready;
	wire [2:0] avm_write_client_wready;
	wire avm_write_valid;
	wire avm_write_ready;
	wire avm_write_drained;
	wire [27:0] avm_write_address;
	wire [27:0] avm_reader_araddr;
	wire [255:0] avm_write_data;
	wire [3:0] avm_reader_arlen;
	wire avm_reader_arvalid;
	wire avm_read_urgent;
	wire avm_pixel_pop;
	wire avm_pixel_valid;
	wire [15:0] avm_pixel_rgb565;
	wire [3:0] avm_lease_acquire;
	wire [3:0] avm_lease_release;
	wire [7:0] avm_lease_slot;
	wire [63:0] avm_lease_epoch;
	wire avm_render_budget_ok;
	wire monitor_read_urgent;
	wire flash_lut_ready_i;
	wire flash_lut_busy_i;
	wire flash_lut_error_i;
	wire flash_jedec_ok_i;
	wire [31:0] flash_bytes_read_i;
	wire monitor_algo_start;
	wire monitor_algo_valid;
	wire monitor_algo_end;
	wire monitor_algo_error;
	wire [7:0] monitor_algo_data;
	wire [15:0] monitor_algo_length;
	wire monitor_ctrl_start;
	wire monitor_ctrl_valid;
	wire monitor_ctrl_end;
	wire monitor_ctrl_error;
	wire [7:0] monitor_ctrl_data;
	wire [15:0] monitor_ctrl_length;
	wire monitor_route_ack_valid;
	wire monitor_route_ack_ready;
	wire [143:0] monitor_route_ack_payload;
	wire [15:0] monitor_route_ack_length;
	wire monitor_mux_ack_valid;
	wire monitor_mux_ack_ready;
	wire [143:0] monitor_mux_ack_payload;
	wire [15:0] monitor_mux_ack_length;
	wire monitor_request_valid;
	wire monitor_request_ready;
	wire [35:0] monitor_request_route;
	wire monitor_active_valid;
	wire monitor_active_ready;
	wire [35:0] monitor_active_route;
	wire monitor_prepared_valid;
	wire monitor_pixel_active_valid;
	wire [35:0] monitor_prepared_route;
	wire [35:0] monitor_pixel_active_route;
	wire monitor_pixel_pop;
	wire monitor_pixel_valid;
	wire [15:0] monitor_pixel_rgb565;
	wire [159:0] monitor_composite_tag;
	wire [23:0] monitor_composite_rgb;
	wire [8:0] monitor_composite_hud;
	wire [27:0] monitor_quad_araddr;
	wire [27:0] monitor_reader_araddr;
	wire [3:0] monitor_quad_arlen;
	wire [3:0] monitor_reader_arlen;
	wire monitor_quad_arvalid;
	wire monitor_reader_arvalid;
	wire monitor_quad_arready;
	wire monitor_quad_rvalid;
	wire [2:0] monitor_client_arready;
	wire [2:0] monitor_client_rvalid;
	wire [11:0] monitor_slot_held;
	wire [11:0] monitor_lease_ready;
	wire [3:0] monitor_lease_acquire;
	wire [3:0] monitor_lease_release;
	wire avm_pixel_frame_ready;
	wire [7:0] monitor_lease_slot;
	wire [63:0] monitor_lease_epoch;
	wire [63:0] monitor_display_epoch;
	reg monitor_quad_snapshot_d1;
	reg [1:0] monitor_control_ddr_reset_sync;
	wire monitor_control_ddr_rstn;

	assign rgmii_clk = i_rgmii_clk;
	assign udp_ctrl_valid = i_udp_ctrl_valid;
	assign udp_ctrl_data = i_udp_ctrl_data;
	assign udp_ctrl_length = i_udp_ctrl_length;
	assign udp_ctrl_start = i_udp_ctrl_start;
	assign udp_ctrl_end = i_udp_ctrl_end;
	assign udp_ctrl_error = i_udp_ctrl_error;
	assign algo_common_reset_n = i_algo_common_reset_n;
	assign algo_net_rst_n = i_algo_net_rst_n;
	assign algo_pixel_rst_n = i_algo_pixel_rst_n;
	assign algo_ack_valid = i_algo_ack_valid;
	assign algo_ack_payload = i_algo_ack_payload;
	assign algo_ack_length = i_algo_ack_length;
	assign o_algo_ack_ready = algo_ack_ready;
	assign hdmi_pixel_clk = i_hdmi_pixel_clk;
	assign algo_output_tag = i_algo_output_tag;
	assign hdmi_algo_rgb = i_hdmi_algo_rgb;
	assign hud_state = i_hud_state;
	assign c5_ddr_clk = i_c5_ddr_clk;
	assign c5_ddr_init_done = i_c5_ddr_init_done;
	assign o_c5_awaddr = c5_awaddr;
	assign o_c5_awlen = c5_awlen;
	assign c5_awready = i_c5_awready;
	assign o_c5_awvalid = c5_awvalid;
	assign o_c5_wdata = c5_wdata;
	assign o_c5_wstrb = c5_wstrb;
	assign c5_wready = i_c5_wready;
	assign o_c5_araddr = c5_araddr;
	assign o_c5_arlen = c5_arlen;
	assign c5_arready = i_c5_arready;
	assign o_c5_arvalid = c5_arvalid;
	assign c5_rdata = i_c5_rdata;
	assign c5_rvalid = i_c5_rvalid;
	assign quad_display_slot = i_quad_display_slot;
	assign quad_display_valid = i_quad_display_valid;
	assign quad_retire_done = i_quad_retire_done;
	assign quad_snapshot = i_quad_snapshot;
	assign quad_read_urgent = i_quad_read_urgent;
	assign quad_ddr_rst_n = i_quad_ddr_rst_n;
	assign o_avm_render_busy = avm_render_busy;
	assign o_avm_frame_toggle_dbg = avm_frame_toggle_dbg;
	assign o_avm_renderer_ready_dbg = avm_renderer_ready_dbg;
	assign o_avm_free_available_dbg = avm_free_available_dbg;
	assign o_avm_pixel_alive_dbg = avm_pixel_alive_dbg;
	assign o_avm_sd_busy_dbg = avm_sd_busy_dbg;
	assign o_avm_sd_ready_dbg = avm_sd_ready_dbg;
	assign o_avm_sd_error_dbg = avm_sd_error_dbg;
	assign o_avm_sd_committed_beats_dbg = avm_sd_committed_beats_dbg;
	assign o_avm_sd_loaded_bytes_dbg = avm_sd_loaded_bytes_dbg;
	assign avm_quad_awaddr = i_avm_quad_awaddr;
	assign avm_quad_awlen = i_avm_quad_awlen;
	assign avm_quad_awvalid = i_avm_quad_awvalid;
	assign avm_quad_wdata = i_avm_quad_wdata;
	assign avm_quad_wstrb = i_avm_quad_wstrb;
	assign o_avm_write_client_awready = avm_write_client_awready;
	assign o_avm_write_client_wready = avm_write_client_wready;
	assign o_avm_pixel_valid = avm_pixel_valid;
	assign o_monitor_algo_start = monitor_algo_start;
	assign o_monitor_algo_valid = monitor_algo_valid;
	assign o_monitor_algo_end = monitor_algo_end;
	assign o_monitor_algo_error = monitor_algo_error;
	assign o_monitor_algo_data = monitor_algo_data;
	assign o_monitor_algo_length = monitor_algo_length;
	assign o_monitor_mux_ack_valid = monitor_mux_ack_valid;
	assign monitor_mux_ack_ready = i_monitor_mux_ack_ready;
	assign o_monitor_mux_ack_payload = monitor_mux_ack_payload;
	assign o_monitor_mux_ack_length = monitor_mux_ack_length;
	assign o_behavior_ack_ready = behavior_ack_ready;
	assign o_monitor_composite_tag = monitor_composite_tag;
	assign o_monitor_composite_rgb = monitor_composite_rgb;
	assign o_monitor_composite_hud = monitor_composite_hud;
	assign monitor_quad_araddr = i_monitor_quad_araddr;
	assign monitor_quad_arlen = i_monitor_quad_arlen;
	assign monitor_quad_arvalid = i_monitor_quad_arvalid;
	assign o_monitor_quad_arready = monitor_quad_arready;
	assign o_monitor_quad_rvalid = monitor_quad_rvalid;
	assign o_monitor_slot_held = monitor_slot_held;
	assign o_monitor_lease_ready = monitor_lease_ready;
	assign o_avm_pixel_frame_ready = avm_pixel_frame_ready;
	assign monitor_display_epoch = i_monitor_display_epoch;

// HDMI监视独立支路；不能重排quad reader或移动dehaze_algo_* PCIe tap。
	// LU loader使用board reset，PLL丢锁不撤销已接受的HMIC写事务。










	// SD/UART bring-up observability exported to top_design.





















	// Flash reader is an alternate LUT producer.  It writes the same DDR
	// client interface as the legacy UDP loader; the generate below ensures
	// that the two producers can never drive the LUT write channel together.


	assign avm_lut_net_rstn=avm_lut_net_reset_sync[1];
	// U9/SPI1 is not used by this build; hold its CS inactive explicitly.
	assign o_flash1_cs_n = 1'b1;
	assign avm_render_budget_ok=!quad_read_urgent && !monitor_read_urgent;
	always@(posedge rgmii_clk or negedge i_button_rst_n)begin
		if(!i_button_rst_n) avm_lut_net_reset_sync<=2'd0;
		else avm_lut_net_reset_sync<={avm_lut_net_reset_sync[0],1'b1};
	end




































	assign monitor_control_ddr_rstn = monitor_control_ddr_reset_sync[1];
	always@(posedge c5_ddr_clk or negedge algo_common_reset_n)begin
		if(!algo_common_reset_n)begin monitor_control_ddr_reset_sync <= 2'd0; end
		else begin monitor_control_ddr_reset_sync <= {monitor_control_ddr_reset_sync[0], 1'b1}; end
	end
	udp_control_magic_dispatch #(.C_LUT_ENABLE((C_LUT_SOURCE == 0) ? 1 : 0),.C_ISP_ENABLE(1)) udp_control_magic_dispatch_Inst_monitor(
		.i_clk(rgmii_clk), .i_rstn(algo_net_rst_n),
		.i_start(udp_ctrl_start), .i_valid(udp_ctrl_valid), .i_data(udp_ctrl_data),
		.i_end(udp_ctrl_end), .i_length(udp_ctrl_length), .i_error(udp_ctrl_error),
		.o_algo_start(monitor_algo_start), .o_algo_valid(monitor_algo_valid), .o_algo_data(monitor_algo_data),
		.o_algo_end(monitor_algo_end), .o_algo_length(monitor_algo_length), .o_algo_error(monitor_algo_error),
		.o_route_start(monitor_ctrl_start), .o_route_valid(monitor_ctrl_valid), .o_route_data(monitor_ctrl_data),
		.o_route_end(monitor_ctrl_end), .o_route_length(monitor_ctrl_length), .o_route_error(monitor_ctrl_error),
		.o_lut_start(avm_lut_start), .o_lut_valid(avm_lut_valid), .o_lut_data(avm_lut_data),
		.o_lut_end(avm_lut_end), .o_lut_length(avm_lut_length), .o_lut_error(avm_lut_error),
		.o_behavior_start(), .o_behavior_valid(), .o_behavior_data(),
		.o_behavior_end(), .o_behavior_length(), .o_behavior_error(),
		.o_isp_start(o_isp_start),.o_isp_valid(o_isp_valid),.o_isp_data(o_isp_data),
		.o_isp_end(o_isp_end),.o_isp_length(o_isp_length),.o_isp_error(o_isp_error));
	monitor_route_control monitor_route_control_Inst_monitor(
		.i_control_clk(rgmii_clk), .i_control_rstn(algo_net_rst_n),
		.i_ddr_clk(c5_ddr_clk), .i_ddr_rstn(monitor_control_ddr_rstn),
		.i_start(monitor_ctrl_start), .i_valid(monitor_ctrl_valid), .i_data(monitor_ctrl_data),
		.i_end(monitor_ctrl_end), .i_length(monitor_ctrl_length), .i_error(monitor_ctrl_error),
		.o_request_valid(monitor_request_valid), .o_request_route(monitor_request_route), .i_request_ready(monitor_request_ready),
		.i_active_valid(monitor_active_valid), .i_active_route(monitor_active_route), .o_active_ready(monitor_active_ready),
		.o_busy(), .o_active_route(), .o_ack_valid(monitor_route_ack_valid), .o_ack_payload(monitor_route_ack_payload),
		.o_ack_length(monitor_route_ack_length), .i_ack_ready(monitor_route_ack_ready), .o_dropped_ack_count());
	udp_control_ack_arbiter #(.C_LUT_ENABLE((C_LUT_SOURCE == 0) ? 1 : 0),.C_ISP_ENABLE(1)) udp_control_ack_arbiter_Inst_monitor(
		.i_clk(rgmii_clk), .i_rstn(algo_net_rst_n),
		.i_algo_valid(algo_ack_valid), .i_algo_payload(algo_ack_payload), .i_algo_length(algo_ack_length), .o_algo_ready(algo_ack_ready),
		.i_route_valid(monitor_route_ack_valid), .i_route_payload(monitor_route_ack_payload), .i_route_length(monitor_route_ack_length), .o_route_ready(monitor_route_ack_ready),
		.i_lut_valid(avm_lut_ack_valid), .i_lut_payload(avm_lut_ack_payload), .i_lut_length(avm_lut_ack_length), .o_lut_ready(avm_lut_ack_ready),
		.i_behavior_valid(i_behavior_ack_valid), .i_behavior_payload(i_behavior_ack_payload), .i_behavior_length(i_behavior_ack_length), .o_behavior_ready(behavior_ack_ready),
		.i_isp_valid(i_isp_ack_valid),.i_isp_payload(i_isp_ack_payload),.i_isp_length(i_isp_ack_length),.o_isp_ready(o_isp_ack_ready),
		.o_valid(monitor_mux_ack_valid), .o_payload(monitor_mux_ack_payload), .o_length(monitor_mux_ack_length), .i_ready(monitor_mux_ack_ready));
	// quad原有retiring状态继续独立推进；FREE槽额外检查各读者引用。
	always@(posedge c5_ddr_clk or negedge quad_ddr_rst_n)begin
		if(!quad_ddr_rst_n)begin monitor_quad_snapshot_d1 <= 1'b0; end
		else begin monitor_quad_snapshot_d1 <= quad_snapshot; end
	end
	video_source_slot_leases video_source_slot_leases_Inst_monitor(
		.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
		.i_acquire({avm_lease_acquire, monitor_lease_acquire, {4{monitor_quad_snapshot_d1}} & quad_display_valid}),
		.i_acquire_slot({avm_lease_slot, monitor_lease_slot, quad_display_slot}),
		.i_acquire_epoch({avm_lease_epoch, monitor_lease_epoch, monitor_display_epoch}),
		.i_release({avm_lease_release, monitor_lease_release, quad_retire_done}),
		.i_completed_valid(quad_display_valid), .i_completed_slot(quad_display_slot), .i_completed_epoch(monitor_display_epoch),
		// AVM启动lease不能被右侧显示FIFO的瞬时urgent状态拒绝；读仲裁器仍在
		// avm_read_urgent/avm_render_budget_ok处执行带宽仲裁，避免启动闭环死锁。
		.i_avm_budget_ok(1'b1), .o_acquire_ready(monitor_lease_ready), .o_slot_held(monitor_slot_held),
		.o_lease_valid(), .o_lease_slot(), .o_lease_epoch());
	monitor_display_pipeline monitor_display_pipeline_Inst_monitor(
		.i_ddr_clk(c5_ddr_clk), .i_ddr_rst_n(quad_ddr_rst_n), .i_ddr_init_done(c5_ddr_init_done), .i_route_rst_n(monitor_control_ddr_rstn),
		.i_route_valid(monitor_request_valid), .i_route(monitor_request_route), .o_route_ready(monitor_request_ready),
		.i_display_slot(quad_display_slot), .i_display_valid(quad_display_valid), .i_display_frame_epoch(monitor_display_epoch),
		.i_lease_ready(monitor_lease_ready[7:4]), .o_lease_acquire(monitor_lease_acquire),
		.o_lease_slot(monitor_lease_slot), .o_lease_frame_epoch(monitor_lease_epoch), .o_lease_release(monitor_lease_release),
		.o_axi_araddr(monitor_reader_araddr), .o_axi_arlen(monitor_reader_arlen), .o_axi_arvalid(monitor_reader_arvalid),
		.i_axi_arready(monitor_client_arready[1]), .i_axi_rdata(c5_rdata), .i_axi_rvalid(monitor_client_rvalid[1]),
		.i_pixel_clk(hdmi_pixel_clk), .i_pixel_rst_n(algo_pixel_rst_n), .i_tag(algo_output_tag), .i_pixel_pop(monitor_pixel_pop),
		.o_pixel_rgb565(monitor_pixel_rgb565), .o_pixel_valid(monitor_pixel_valid),
		.o_prepared_valid(monitor_prepared_valid), .o_prepared_route(monitor_prepared_route),
		.i_pixel_active_valid(monitor_pixel_active_valid), .i_pixel_active_route(monitor_pixel_active_route),
		.o_active_valid(monitor_active_valid), .o_active_route(monitor_active_route), .i_active_ready(monitor_active_ready),
		.o_read_urgent(monitor_read_urgent));
	assign monitor_quad_arready = monitor_client_arready[0];
	assign monitor_quad_rvalid = monitor_client_rvalid[0];
	hdmi_ddr_read_arbiter hdmi_ddr_read_arbiter_Inst_monitor(
		.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
		.i_client_araddr({avm_reader_araddr, monitor_reader_araddr, monitor_quad_araddr}),
		.i_client_arlen({avm_reader_arlen, monitor_reader_arlen, monitor_quad_arlen}),
		.i_client_arvalid({avm_reader_arvalid, monitor_reader_arvalid, monitor_quad_arvalid}),
		.i_display_urgent({monitor_read_urgent, quad_read_urgent}),
		// renderer占用期间必须获得共享HMIC的连续读机会，否则右侧FIFO
		// 的urgent会把AVM cache/LUT读取完全屏蔽并在33ms deadline前取消。
		.i_avm_budget_ok(avm_read_urgent || avm_render_budget_ok || avm_render_busy),
		.o_client_arready(monitor_client_arready), .o_client_rvalid(monitor_client_rvalid), .o_client_rdata(),
		.o_axi_araddr(c5_araddr), .o_axi_arlen(c5_arlen), .o_axi_arvalid(c5_arvalid), .i_axi_arready(c5_arready),
		.i_axi_rdata(c5_rdata), .i_axi_rvalid(c5_rvalid), .o_busy());
	// 左480x640真实DDR输出居中于640x640，缺帧/欠流由AVM显示支路输出黑色。
	avm_monitor_compositor #(.C_LEFT_AVM_ENABLE(1), .C_EXTERNAL_COMMIT(0)) avm_monitor_compositor_Inst_monitor(
		.i_clk(hdmi_pixel_clk), .i_rst_n(algo_pixel_rst_n), .i_tag(algo_output_tag), .i_rgb(hdmi_algo_rgb), .i_hud_state(hud_state),
		.i_prepared_valid(monitor_prepared_valid), .i_prepared_route(monitor_prepared_route), .i_route_applied(1'b0), .i_active_route(36'd0),
		.i_monitor_rgb565(monitor_pixel_rgb565), .i_monitor_valid(monitor_pixel_valid), .i_avm_rgb565(avm_pixel_rgb565), .i_avm_valid(avm_pixel_valid),
		.o_monitor_pop(monitor_pixel_pop), .o_avm_pop(avm_pixel_pop), .o_tag(monitor_composite_tag), .o_rgb(monitor_composite_rgb), .o_hud_state(monitor_composite_hud),
		.o_active_valid(monitor_pixel_active_valid), .o_active_route(monitor_pixel_active_route));
	generate
		if ((C_LUT_SOURCE == 1) && (C_FLASH_ENABLE != 0)) begin : g_flash_lut_reader
			// Flash 读取只在 DDR 初始化完成后启动。读取期间占用现有 LUT
			// 写 client，完成后才释放 avm_lut_ready，避免 renderer 读半张表。
			avm_lut_flash_reader_gtp #(
				.C_FLASH_SELECT(C_FLASH_SELECT),
				.C_FLASH_BASE_ADDR(C_FLASH_LUT_BASE_ADDR),
				.C_LUT_BASE(28'h0300000),
				.C_LUT_BYTES(32'd4915200),
				.C_SPI_DIV(16),
				.C_TIMEOUT_CYCLES(32'd2000000000)
			) avm_lut_flash_reader_gtp_Inst_avm(
				.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
				.i_enable(c5_ddr_init_done),
				.o_flash_cs_n(o_flash_cs_n), .io_flash_dq(io_flash_dq),
				.o_lut_ready(flash_lut_ready_i), .o_busy(flash_lut_busy_i), .o_error(flash_lut_error_i),
				.o_jedec_ok(flash_jedec_ok_i), .o_bytes_read(flash_bytes_read_i),
				.o_axi_awaddr(avm_lut_awaddr), .o_axi_awlen(avm_lut_awlen), .o_axi_awvalid(avm_lut_awvalid),
				.i_axi_awready(avm_write_client_awready[1]), .o_axi_wdata(avm_lut_wdata),
				.o_axi_wstrb(avm_lut_wstrb), .o_axi_wvalid(avm_lut_wvalid), .i_axi_wready(avm_write_client_wready[1]));
			assign avm_lut_ready = flash_lut_ready_i;
			assign avm_lut_update_pending = flash_lut_busy_i;
			assign avm_sd_busy_dbg = flash_lut_busy_i;
			assign avm_sd_ready_dbg = flash_lut_ready_i;
			assign avm_sd_error_dbg = flash_lut_error_i ? 8'hf1 : 8'h00;
			assign avm_sd_committed_beats_dbg = flash_bytes_read_i >> 5;
			assign avm_sd_loaded_bytes_dbg = flash_bytes_read_i;
			assign avm_lut_ack_valid = 1'b0;
			assign avm_lut_ack_payload = 144'd0;
			assign avm_lut_ack_length = 16'd0;
			assign o_sd_clk = 1'b0;
			assign o_sd_mosi = 1'b1;
			assign o_sd_cs = 1'b1;
		end else if (C_LUT_SOURCE == 2) begin : g_sd_lut_reader
			// SD分支直接消费原始payload；UDP parser/ACK在C_LUT_SOURCE=2时关闭。
			assign o_flash_cs_n = 1'b1;
			assign io_flash_dq = 4'bzzzz;
			assign flash_lut_ready_i = 1'b0;
			assign flash_lut_busy_i = 1'b0;
			assign flash_lut_error_i = 1'b0;
			assign flash_jedec_ok_i = 1'b0;
			assign flash_bytes_read_i = 32'd0;
			avm_lut_sd_loader #(
				.C_FIRST_LBA(32'd16672), .C_BLOCKS(9600),
				.C_LUT_BASE(28'h0300000), .C_INIT_DIV(54), .C_READ_DIV(4),
				.C_POWER_WAIT(270000), .C_TOKEN_TIMEOUT_CYCLES(28620000),
				.C_PROGRESS_TIMEOUT_CYCLES(27000000)
			) avm_lut_sd_loader_Inst_avm(
				.i_sd_clk(i_sys_clk), .i_ddr_clk(c5_ddr_clk), .i_rstn(i_button_rst_n),
				.i_ddr_init_done(c5_ddr_init_done), .o_sd_clk(o_sd_clk), .o_sd_mosi(o_sd_mosi),
				.i_sd_miso(i_sd_miso), .o_sd_cs(o_sd_cs), .o_lut_ready(avm_lut_ready),
				.o_busy(avm_sd_busy_dbg), .o_error(avm_sd_error_dbg),
				.o_committed_beats(avm_sd_committed_beats_dbg), .o_loaded_bytes(avm_sd_loaded_bytes_dbg),
				.o_axi_awaddr(avm_lut_awaddr), .o_axi_awlen(avm_lut_awlen), .o_axi_awvalid(avm_lut_awvalid),
				.i_axi_awready(avm_write_client_awready[1]), .o_axi_wdata(avm_lut_wdata),
				.o_axi_wstrb(avm_lut_wstrb), .o_axi_wvalid(avm_lut_wvalid),
				.i_axi_wready(avm_write_client_wready[1]));
			assign avm_lut_update_pending = !avm_lut_ready;
			assign avm_sd_ready_dbg = avm_lut_ready;
			assign avm_lut_ack_valid = 1'b0;
			assign avm_lut_ack_payload = 144'd0;
			assign avm_lut_ack_length = 16'd0;
		end else begin : g_udp_lut_loader
			// Flash pads remain electrically idle in the UDP compatibility build.
			// Keep CS# deasserted and release all DQ pins so the default bitstream
			// cannot contend with the board Flash.
			assign o_flash_cs_n = 1'b1;
			assign io_flash_dq = 4'bzzzz;
			assign o_sd_clk = 1'b0;
			assign o_sd_mosi = 1'b1;
			assign o_sd_cs = 1'b1;
			avm_lut_udp_loader avm_lut_udp_loader_Inst_avm(
				.i_net_clk(rgmii_clk), .i_net_rst_n(avm_lut_net_rstn),
				.i_start(avm_lut_start), .i_valid(avm_lut_valid), .i_data(avm_lut_data),
				.i_end(avm_lut_end), .i_length(avm_lut_length), .i_error(avm_lut_error),
				.o_ack_valid(avm_lut_ack_valid), .o_ack_payload(avm_lut_ack_payload),
				.o_ack_length(avm_lut_ack_length), .i_ack_ready(avm_lut_ack_ready), .o_dropped_packet_count(),
				.i_ddr_clk(c5_ddr_clk), .i_ddr_rst_n(quad_ddr_rst_n), .i_lut_in_use(avm_render_busy),
				.o_upload_active(), .o_update_pending(avm_lut_update_pending), .o_lut_ready(avm_lut_ready),
				.o_upload_sequence(), .o_next_offset(), .o_committed_bytes(),
				.o_axi_awaddr(avm_lut_awaddr), .o_axi_awlen(avm_lut_awlen), .o_axi_awvalid(avm_lut_awvalid),
				.i_axi_awready(avm_write_client_awready[1]), .o_axi_wdata(avm_lut_wdata),
				.o_axi_wstrb(avm_lut_wstrb), .o_axi_wvalid(avm_lut_wvalid), .i_axi_wready(avm_write_client_wready[1]));
			assign flash_lut_ready_i = 1'b0;
			assign flash_lut_busy_i = 1'b0;
			assign flash_lut_error_i = 1'b0;
			assign flash_jedec_ok_i = 1'b0;
			assign flash_bytes_read_i = 32'd0;
			assign avm_sd_busy_dbg = 1'b0;
			assign avm_sd_ready_dbg = 1'b0;
			assign avm_sd_error_dbg = 8'd0;
			assign avm_sd_committed_beats_dbg = 32'd0;
			assign avm_sd_loaded_bytes_dbg = 32'd0;
		end
	endgenerate
	avm_single_beat_writer avm_single_beat_writer_Inst_avm(
		.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
		.i_valid(avm_write_valid), .i_address(avm_write_address), .i_data(avm_write_data),
		.o_ready(avm_write_ready), .o_drained(avm_write_drained),
		.o_axi_awaddr(avm_render_awaddr), .o_axi_awlen(avm_render_awlen), .o_axi_awvalid(avm_render_awvalid),
		.i_axi_awready(avm_write_client_awready[2]), .o_axi_wdata(avm_render_wdata),
		.o_axi_wstrb(avm_render_wstrb), .o_axi_wvalid(avm_render_wvalid), .i_axi_wready(avm_write_client_wready[2]));
	ddr_write_owner_arbiter ddr_write_owner_arbiter_Inst_avm(
		.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
		.i_source_awaddr({avm_render_awaddr,avm_lut_awaddr,avm_quad_awaddr}),
		.i_source_awlen({avm_render_awlen,avm_lut_awlen,avm_quad_awlen}),
		.i_source_awvalid({avm_render_awvalid,avm_lut_awvalid,avm_quad_awvalid}),
		.i_source_wdata({avm_render_wdata,avm_lut_wdata,avm_quad_wdata}),
		.i_source_wstrb({avm_render_wstrb,avm_lut_wstrb,avm_quad_wstrb}),
		.o_source_awready(avm_write_client_awready), .o_source_wready(avm_write_client_wready),
		.o_axi_awaddr(c5_awaddr), .o_axi_awlen(c5_awlen), .o_axi_awvalid(c5_awvalid),
		.i_axi_awready(c5_awready), .o_axi_wdata(c5_wdata), .o_axi_wstrb(c5_wstrb), .i_axi_wready(c5_wready), .o_busy(), .o_active_owner());
	avm_video_subsystem #(
		// 新 avm_slam_fpga_640x640 LUT 槽位为 [左,前,右,后]；
		// 2026-10-15：物理 source 为 [Ethernet右,OV5640后,HSST前,HDMI左]。
		.C_SOURCE_SLOT0(2'd3), // LUT 左视角对应 HDMI 输入。
		.C_SOURCE_SLOT1(2'd2), // LUT 前视角对应 HSST 光模块输入。
		.C_SOURCE_SLOT2(2'd0), // LUT 右视角对应 Ethernet 输入。
		.C_SOURCE_SLOT3(2'd1)  // LUT 后视角对应 OV5640 输入。
	) avm_video_subsystem_Inst_avm(
		.i_ddr_clk(c5_ddr_clk), .i_ddr_rst_n(quad_ddr_rst_n), .i_ddr_init_done(c5_ddr_init_done),
		.i_pixel_clk(hdmi_pixel_clk), .i_pixel_rst_n(algo_pixel_rst_n), .i_tag(algo_output_tag),
		.i_lut_ready(avm_lut_ready), .i_lut_update_pending(avm_lut_update_pending),
		.i_display_slot(quad_display_slot), .i_display_present(quad_display_valid), .i_display_frame_epoch(monitor_display_epoch),
		.i_lease_ready(monitor_lease_ready[11:8]), .o_lease_acquire(avm_lease_acquire),
		.o_lease_slot(avm_lease_slot), .o_lease_frame_epoch(avm_lease_epoch), .o_lease_release(avm_lease_release),
		.o_axi_araddr(avm_reader_araddr), .o_axi_arlen(avm_reader_arlen), .o_axi_arvalid(avm_reader_arvalid),
		.i_axi_arready(monitor_client_arready[2]), .i_axi_rdata(c5_rdata), .i_axi_rvalid(monitor_client_rvalid[2]),
		.o_write_valid(avm_write_valid), .o_write_address(avm_write_address), .o_write_data(avm_write_data),
		.i_write_ready(avm_write_ready), .i_write_drained(avm_write_drained),
		.i_pixel_pop(avm_pixel_pop), .o_pixel_rgb565(avm_pixel_rgb565), .o_pixel_valid(avm_pixel_valid), .o_pixel_frame_ready(avm_pixel_frame_ready),
		.o_read_urgent(avm_read_urgent), .o_render_busy(avm_render_busy), .o_output_valid(),
		.o_render_done(), .o_render_drop_count(), .o_underflow_count(), .o_last_render_cycles(),
		.o_debug_frame_toggle(avm_frame_toggle_dbg), .o_debug_renderer_ready(avm_renderer_ready_dbg),
		.o_debug_free_available(avm_free_available_dbg), .o_debug_pixel_alive(avm_pixel_alive_dbg));

endmodule
