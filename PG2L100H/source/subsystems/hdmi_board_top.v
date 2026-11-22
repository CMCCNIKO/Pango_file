//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: HDMI 与板级控制子系统，由官方 PLL 将 27MHz 生成为 74.25MHz 像素时钟，像素域复位在锁定同步后释放。
// Module Name: hdmi_board_top
// File Name: source/subsystems/hdmi_board_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// HDMI 与板级控制子系统，由官方 PLL 将 27MHz 生成为 74.25MHz 像素时钟，像素域复位在锁定同步后释放。
// 选择显示基流并叠加车灯/车道/行为 HUD，通过 I2C 管理 MS7200/MS7210；ODDR 专用资源连续转发 HDMI TX PCLK。
// 独立执行 RTL8211F MDIO 初始化、TX 内部延迟配置和链路轮询，将欠流及源3丢帧事件同步锁存到诊断 LED。
//
// Dependencies: hdmi_720p_pll、mdio_link_status、hdmi_hud_road_lights、hud_control_subsystem、hdmi_hud_overlay、hdmi_i2c_manager、c6_event_sync、GTP_ODDR_E1、GTP_OUTBUF。
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
// Module Name: hdmi_board_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Structural extraction; original clocks, handshakes and latency retained.
// Dependencies: Original project modules instantiated below.
// Revision: 0.01 - Extracted from top_design.
// Revision: 0.02 - 2026-10-21 Combined scene/behavior display snapshot and commit feedback.
// Revision: 0.03 - 2026-10-23 独立系统域HSST车道接入，脱离PCIe会话更新HUD。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hdmi_board_top #(
	parameter [4:0] C_PHY_ADDR = 5'd1,
	parameter integer C_USE_QUAD_OUTPUT = 0
)(
	input i_algo_pixel_rst_n,
	output o_hdmi_pixel_clk,
	output o_hdmi_pll_lock,
	output o_hdmi_pixel_rst_n,
	input i_hdmi_pclk,
	input i_quad_h1_pclk,
	input i_quad_h1_vs,
	input i_quad_h1_hs,
	input i_quad_h1_de,
	input [23:0] i_quad_h1_rgb,
	output o_quad_base_vs,
	output o_quad_base_hs,
	output o_quad_base_de,
	output [23:0] o_quad_base_rgb,
	input [159:0] i_algo_output_tag,
	input i_hdmi_algo_vs,
	input [1:0] i_night_level,
	input i_dehaze_hud_enable,
	output [8:0] o_hud_state,
	input i_hud_wr_en,
	input [11:0] i_hud_wr_addr,
	input [31:0] i_hud_wr_data,
	input [3:0] i_hud_wr_strb,
	input [11:0] i_hud_rd_addr,
	output [127:0] o_hud_rd_window,
	input i_fiber_pll_ready,
	input [63:0] i_fiber_lane_snapshot,
	input i_fiber_lane_valid,
	input i_fiber_lane_stale,
	input i_pcie_pclk_div2,
	input i_pcie_core_rst_n,
	input i_pcie_link_up,
	input [175:0] i_display_snapshot,
	output o_scene_commit_valid,
	output [111:0] o_scene_commit_payload,
	output o_control_pcie_rstn,
	input i_c5_ddr_clk,
	input i_c5_underflow_event,
	input [3:0] i_quad_frame_aborted,
	input i_quad_ddr_rst_n,
	input [159:0] i_monitor_composite_tag,
	input [23:0] i_monitor_composite_rgb,
	input [8:0] i_monitor_composite_hud,
	input i_sys_clk,
	input i_button_rst_n,
	output o_phy_reset_n,
	output o_mdc,
	inout io_mdio,
	output o_hdmi_pclk,
	output o_hdmi_vs,
	output o_hdmi_hs,
	output o_hdmi_de,
	output [23:0] o_hdmi_rgb,
	output o_hdmi_scl,
	inout io_hdmi_sda,
	input i_sfp0_los,
	output o_sfp0_tx_disable,
	input i_pcie_perst_n,
	output [3:0] o_status_led
);
	wire algo_pixel_rst_n;
	wire [1:0] phy_speed;
	wire phy_link;
	wire phy_config_done;
	wire phy_config_error;
	wire hdmi_pixel_clk;
	wire hdmi_pll_lock;
	wire hdmi_pixel_rst_n;
	reg [1:0] hdmi_pll_lock_sync;
	reg [1:0] hdmi_pixel_reset_sync;
	wire hdmi_pclk;
	wire hdmi_scl;
	wire hdmi_sda_drive_low;
	wire hdmi_init_done;
	wire hdmi_init_error;
	wire hdmi_rx_init_done;
	wire hdmi_all_init_done;
	wire quad_h1_pclk;
	wire quad_h1_vs;
	wire quad_h1_hs;
	wire quad_h1_de;
	wire [23:0] quad_h1_rgb;
	wire quad_base_pclk;
	wire quad_base_vs;
	wire quad_base_hs;
	wire quad_base_de;
	wire [23:0] quad_base_rgb;
	wire [159:0] algo_output_tag;
	wire hdmi_algo_vs;
	wire hdmi_hud_vs;
	wire hdmi_hud_hs;
	wire hdmi_hud_de;
	wire [23:0] hdmi_hud_rgb;
	wire [8:0] hud_state;
	wire [31:0] hud_ext_state;
	wire [31:0] hud_time0;
	wire [31:0] hud_time1;
	wire hud_ext_valid;
	wire [4:0] hud_dehaze_lights;
	wire hud_control_latch;
	wire hud_status_latched;
	wire [8:0] hud_latched_state;
	wire hud_wr_en;
	wire [11:0] hud_wr_addr;
	wire [31:0] hud_wr_data;
	wire [3:0] hud_wr_strb;
	wire [11:0] hud_rd_addr;
	wire [127:0] hud_rd_window;
	wire fiber_pll_ready;
	wire pcie_pclk_div2;
	wire pcie_core_rst_n;
	wire pcie_link_up;
	wire c5_ddr_clk;
	wire c5_underflow_event;
	wire [3:0] quad_frame_aborted;
	wire quad_ddr_rst_n;
	wire hdmi_underflow_event_sys;
	wire source3_abort_event_sys;
	reg hdmi_underflow_seen_sys;
	reg source3_abort_seen_sys;
	wire [159:0] monitor_composite_tag;
	wire [23:0] monitor_composite_rgb;
	wire [8:0] monitor_composite_hud;
	wire [63:0] behavior_snapshot_pixel;
	wire [31:0] nav_state;
	wire [19:0] nav_bcd;
	wire [15:0] nav_seq;
	wire [15:0] nav_token;
	wire nav_valid;
	wire [31:0] nav_latched_state;
	wire [15:0] nav_latched_seq;
	wire [15:0] nav_latched_token;
	wire nav_latched_valid;

	assign algo_pixel_rst_n = i_algo_pixel_rst_n;
	assign o_hdmi_pixel_clk = hdmi_pixel_clk;
	assign o_hdmi_pll_lock = hdmi_pll_lock;
	assign o_hdmi_pixel_rst_n = hdmi_pixel_rst_n;
	assign hdmi_pclk = i_hdmi_pclk;
	assign quad_h1_pclk = i_quad_h1_pclk;
	assign quad_h1_vs = i_quad_h1_vs;
	assign quad_h1_hs = i_quad_h1_hs;
	assign quad_h1_de = i_quad_h1_de;
	assign quad_h1_rgb = i_quad_h1_rgb;
	assign o_quad_base_vs = quad_base_vs;
	assign o_quad_base_hs = quad_base_hs;
	assign o_quad_base_de = quad_base_de;
	assign o_quad_base_rgb = quad_base_rgb;
	assign algo_output_tag = i_algo_output_tag;
	assign hdmi_algo_vs = i_hdmi_algo_vs;
	assign o_hud_state = hud_state;
	assign hud_wr_en = i_hud_wr_en;
	assign hud_wr_addr = i_hud_wr_addr;
	assign hud_wr_data = i_hud_wr_data;
	assign hud_wr_strb = i_hud_wr_strb;
	assign hud_rd_addr = i_hud_rd_addr;
	assign o_hud_rd_window = hud_rd_window;
	assign fiber_pll_ready = i_fiber_pll_ready;
	assign pcie_pclk_div2 = i_pcie_pclk_div2;
	assign pcie_core_rst_n = i_pcie_core_rst_n;
	assign pcie_link_up = i_pcie_link_up;
	assign c5_ddr_clk = i_c5_ddr_clk;
	assign c5_underflow_event = i_c5_underflow_event;
	assign quad_frame_aborted = i_quad_frame_aborted;
	assign quad_ddr_rst_n = i_quad_ddr_rst_n;
	assign monitor_composite_tag = i_monitor_composite_tag;
	assign monitor_composite_rgb = i_monitor_composite_rgb;
	assign monitor_composite_hud = i_monitor_composite_hud;

	// 两种单拍事件同步到 LED 驱动时钟域，复位后保持锁存，便于肉眼读取。
	c6_event_sync u_hdmi_underflow_diag_sync(
		.i_src_clk(hdmi_pixel_clk), .i_src_rst_n(hdmi_pixel_rst_n),
		.i_src_event(c5_underflow_event),
		.i_dst_clk(i_sys_clk), .i_dst_rst_n(i_button_rst_n),
		.o_dst_event(hdmi_underflow_event_sys)
	);
	c6_event_sync u_source3_abort_diag_sync(
		.i_src_clk(c5_ddr_clk), .i_src_rst_n(quad_ddr_rst_n),
		.i_src_event(quad_frame_aborted[3]),
		.i_dst_clk(i_sys_clk), .i_dst_rst_n(i_button_rst_n),
		.o_dst_event(source3_abort_event_sys)
	);
	always @(posedge i_sys_clk or negedge i_button_rst_n) begin
		if (!i_button_rst_n) begin
			hdmi_underflow_seen_sys <= 1'b0;
			source3_abort_seen_sys <= 1'b0;
		end else begin
			if (hdmi_underflow_event_sys) hdmi_underflow_seen_sys <= 1'b1;
			if (source3_abort_event_sys) source3_abort_seen_sys <= 1'b1;
		end
	end


	assign quad_base_pclk = quad_h1_pclk;
	assign quad_base_vs = quad_h1_vs;
	assign quad_base_hs = quad_h1_hs;
	assign quad_base_de = quad_h1_de;
	assign quad_base_rgb = quad_h1_rgb;
	// 专用 ODDR 转发像素时钟，保持与原时钟相同的极性和频率。
	// 直接把全局时钟送到普通输出缓冲时，布线器可引入无外部约束的长线；
	// 2025.2 故障实现的 PCLK 比已验证版本慢约 3.42 ns，已靠近数据翻转沿。
	// 时钟保持连续，不受显示模式或视频数据复位门控。
	wire hdmi_tx_pixel_clk;
	wire hdmi_tx_clock_oddr;
	assign hdmi_tx_pixel_clk = C_USE_QUAD_OUTPUT ? quad_base_pclk : hdmi_pclk;
	GTP_ODDR_E1 #(
		.GRS_EN("TRUE"),
		.ODDR_MODE("SAME_EDGE"),
		.RS_TYPE("ASYNC_RESET")
	) u_hdmi_tx_pclk_oddr(
		.Q(hdmi_tx_clock_oddr),
		.CE(1'b1),
		.CLK(hdmi_tx_pixel_clk),
		.D0(1'b1),
		.D1(1'b0),
		.RS(1'b0)
	);
	GTP_OUTBUF #(
		.IOSTANDARD("LVCMOS33"),
		.SLEW_RATE("FAST"),
		.DRIVE_STRENGTH("8")
	) u_hdmi_tx_pclk_obuf(
		.I(hdmi_tx_clock_oddr),
		.O(o_hdmi_pclk)
	);
	assign o_hdmi_vs = C_USE_QUAD_OUTPUT ? quad_base_vs : hdmi_hud_vs;
	assign o_hdmi_hs = C_USE_QUAD_OUTPUT ? quad_base_hs : hdmi_hud_hs;
	assign o_hdmi_de = C_USE_QUAD_OUTPUT ? quad_base_de : hdmi_hud_de;
	assign o_hdmi_rgb = C_USE_QUAD_OUTPUT ? quad_base_rgb : hdmi_hud_rgb;
	assign o_hdmi_scl = hdmi_scl;
	assign io_hdmi_sda = hdmi_sda_drive_low ? 1'b0 : 1'bz;


	mdio_link_status #(
		.C_PHY_ADDR(C_PHY_ADDR)
	) u_mdio_link_status(
		.i_clk(i_sys_clk),
		.i_rst_n(i_button_rst_n),
		.io_mdio(io_mdio),
		.o_mdc(o_mdc),
		.o_link(phy_link),
		.o_speed(phy_speed),
		.o_phy_reset_n(o_phy_reset_n),
		.o_config_done(phy_config_done),
		.o_config_error(phy_config_error)
	);

	// 官方 PLL：27 MHz 输入，74.25 MHz 输出，对应 720p60 的像素时钟。
	// lock 先在像素域同步，再经两级复位移位寄存器同步释放像素逻辑复位。
	hdmi_720p_pll u_hdmi_720p_pll(
		.clkout0(hdmi_pixel_clk),
		.clkin1(i_sys_clk),
		.lock(hdmi_pll_lock)
	);

	always@(posedge hdmi_pixel_clk or negedge i_button_rst_n)begin
		if(i_button_rst_n == 1'b0)begin
			hdmi_pll_lock_sync <= 2'b00;
			hdmi_pixel_reset_sync <= 2'b00;
		end else begin
			hdmi_pll_lock_sync <= {hdmi_pll_lock_sync[0], hdmi_pll_lock};
			if(hdmi_pll_lock_sync[1] == 1'b0)hdmi_pixel_reset_sync <= 2'b00;
			else hdmi_pixel_reset_sync <= {hdmi_pixel_reset_sync[0], 1'b1};
		end
	end
	assign hdmi_pixel_rst_n = hdmi_pixel_reset_sync[1];


	// 灯光跟随已走完算法流水的本帧去雾配置，不使用UDP请求或accepted状态。
	hdmi_hud_road_lights hdmi_hud_road_lights_Inst_hdmi(
		.i_clk(hdmi_pixel_clk),
		.i_rst_n(algo_pixel_rst_n),
		.i_vs(hdmi_algo_vs),
		.i_dehaze_enable(i_dehaze_hud_enable),
		.o_left(hud_dehaze_lights[0]),
		.o_right(hud_dehaze_lights[1]),
		.o_hazard(hud_dehaze_lights[2]),
		.o_front_fog(hud_dehaze_lights[3]),
		.o_position(hud_dehaze_lights[4])
	);

	// 算法之后叠加 HUD，保留图标原色。PCIe 使用同一份无 HUD 算法流，
	// 这样 HDMI 与 PCIe 的像素内容一致，同时避免把 HUD 图标编码进视频帧。
	hud_control_subsystem hud_control_subsystem_Inst_control(
		.i_pcie_clk(pcie_pclk_div2),
		.i_display_snapshot(i_display_snapshot),
		.o_scene_commit_valid(o_scene_commit_valid),
		.o_scene_commit_payload(o_scene_commit_payload),
		.o_control_pcie_rstn(o_control_pcie_rstn),
		.o_behavior_result(behavior_snapshot_pixel[31:0]),
		.o_behavior_effective(behavior_snapshot_pixel[63:32]),
		.i_pcie_rstn(pcie_core_rst_n && i_pcie_perst_n && i_button_rst_n),
		.i_pixel_clk(hdmi_pixel_clk),
		.i_pixel_rstn(algo_pixel_rst_n),
		.i_lane_sys_clk(i_sys_clk),
		.i_lane_sys_rst_n(i_button_rst_n),
		.i_fiber_lane_payload(i_fiber_lane_snapshot[31:0]),
		.i_fiber_lane_valid(i_fiber_lane_valid),
		.i_fiber_lane_stale(i_fiber_lane_stale),
		.i_link_up(pcie_link_up),
		.i_wr_en(hud_wr_en),
		.i_wr_addr(hud_wr_addr),
		.i_wr_data(hud_wr_data),
		.i_wr_strb(hud_wr_strb),
		.i_rd_addr(hud_rd_addr),
		.o_rd_window(hud_rd_window),
		.i_latch(hud_control_latch),
		.i_latched(hud_status_latched),
		.i_latched_state(hud_latched_state),
		.i_dehaze_enable(i_dehaze_hud_enable),
		.i_dehaze_lights(hud_dehaze_lights),
		.i_night_level(i_night_level),
		.o_state(hud_state),
		.o_ext_state(hud_ext_state), .o_time0(hud_time0), .o_time1(hud_time1),
		.o_ext_valid(hud_ext_valid),
		.o_nav_state(nav_state), .o_nav_bcd(nav_bcd), .o_nav_seq(nav_seq),
		.o_nav_token(nav_token), .o_nav_valid(nav_valid),
		.i_nav_latched_state(nav_latched_state), .i_nav_latched_seq(nav_latched_seq),
		.i_nav_latched_token(nav_latched_token), .i_nav_latched_valid(nav_latched_valid)
	);

	hdmi_hud_overlay hdmi_hud_overlay_Inst_hdmi(
		.i_clk(hdmi_pixel_clk),
		.i_rst_n(algo_pixel_rst_n),
		.i_vs(monitor_composite_tag[159]),
		.i_hs(monitor_composite_tag[158]),
		.i_de(monitor_composite_tag[157]),
		.i_rgb(monitor_composite_rgb),
		// 控制在overlay本身的帧边界锁存，与行为共用当前快照，不走合成器像素寄存。
		.i_enable(hud_state[0]),
		.i_left(hud_state[1]),
		.i_right(hud_state[2]),
		.i_hazard(hud_state[3]),
		.i_low_beam(hud_state[4]),
		.i_high_beam(hud_state[5]),
		.i_front_fog(hud_state[6]),
		.i_position(hud_state[7]),
		.i_adas_enable(hud_state[8]),
		.i_behavior_result(behavior_snapshot_pixel[31:0]),
		.i_behavior_effective(behavior_snapshot_pixel[63:32]),
		.i_ext_state(hud_ext_state),
		.i_time0(hud_time0),
		.i_time1(hud_time1),
		.i_ext_valid(hud_ext_valid),
		.i_nav_state(nav_state), .i_nav_bcd(nav_bcd), .i_nav_seq(nav_seq),
		.i_nav_token(nav_token), .i_nav_valid(nav_valid),
		.o_nav_latched_state(nav_latched_state), .o_nav_latched_seq(nav_latched_seq),
		.o_nav_latched_token(nav_latched_token), .o_nav_latched_valid(nav_latched_valid),
		.o_vs(hdmi_hud_vs),
		.o_hs(hdmi_hud_hs),
		.o_de(hdmi_hud_de),
		.o_rgb(hdmi_hud_rgb),
		.o_control_latch(hud_control_latch),
		.o_status_latched(hud_status_latched),
		.o_status(hud_latched_state)
	);


	hdmi_i2c_manager u_hdmi_i2c_manager(
		.i_clk(i_sys_clk),
		.i_rst_n(i_button_rst_n),
		.i_sda(io_hdmi_sda),
		.o_scl(hdmi_scl),
		.o_sda_drive_low(hdmi_sda_drive_low),
		.o_rx_init_done(hdmi_rx_init_done),
		.o_tx_init_done(hdmi_init_done),
		.o_init_done(hdmi_all_init_done),
		.o_init_error(hdmi_init_error)
	);


	// RX-only bring-up diagnostics from the implementation plan.
	assign o_sfp0_tx_disable = 1'b1;
	assign o_status_led[0] = !i_sfp0_los;
	assign o_status_led[1] = fiber_pll_ready;
	// HDMI-IN 源3卡顿诊断：LED2 锁存显示侧欠流，LED3 锁存源3输入/DDR帧丢弃。
	assign o_status_led[2] = hdmi_underflow_seen_sys;
	assign o_status_led[3] = source3_abort_seen_sys;


endmodule
