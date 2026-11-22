//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: PG2L100H 板级总顶层，连接 Ethernet/PGV1、OV5640 CMOS1、SFP1（板端 SFP2）HSST CH2、HDMI-IN/MS7200 四路视频以及 DDR3、PCIe x4、HDMI-OUT/MS7210。
// Module Name: top_design
// File Name: source/top_design.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// PG2L100H 板级总顶层，连接 Ethernet/PGV1、OV5640 CMOS1、SFP1（板端 SFP2）HSST CH2、HDMI-IN/MS7200 四路视频以及 DDR3、PCIe x4、HDMI-OUT/MS7210。
// ethernet_top 负责以太网与 PGV1 入帧；video_sources_top 负责摄像头、HDMI 与光纤输入及源域到 DDR 的跨域。
// frame_buffer_top 负责四源三帧槽、DDR 读写与四宫格；video_algorithm_top 负责共享 ISP/去雾和 HDMI 专用后级；avm_monitor_top 负责 LUT 装载、AVM 渲染、双路监视与合成。
// pcie_top 从共享去雾结果采集并传输，不含 HDMI 专用滤波和 HUD；hdmi_board_top 负责像素时钟、HUD、HDMI I2C、PHY MDIO 和板级诊断。
// 当前默认开启四路输入，Q8 写入采用每 AW 一个 256-bit word；LUT 来源为 SD（C_LUT_SOURCE=2），最终 HDMI 默认经过含左侧 AVM 的 monitor compositor。
//
// Dependencies: ethernet_top、video_sources_top、frame_buffer_top、video_algorithm_top、avm_monitor_top、pcie_top、hdmi_board_top、vehicle_behavior_action_rx；工程登记的 Pango IP 与板级约束。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 历史 SFP0 端口及 fiber_hsst_ch0 IP 名称保留，实际业务接入 HSST Channel 2。
// 本文件原始创建日期未注明；本次文件头整理日期为 2026-11-01。
// 参考工具为 PDS 2025.2 build 211867，不能沿用模板中的 PDS 2022.2-SP6.4 作为当前版本。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// PG2L100H AVM/PCIe production hierarchy
//
// 当前顶层接入 Ethernet、HDMI-IN、OV5640 与 SFP1/HSST RX 四路视频源，
// 并通过统一四源 DDR 帧槽管理与四宫格读出送往 MS7210 HDMI 输出。
//
// Seven functional tops own their original leaf modules and clock domains.
//////////////////////////////////////////////////////////////////////////////////
module top_design #(
	parameter [47:0] C_LOCAL_MAC = 48'h02_00_00_00_00_01,
	parameter [31:0] C_LOCAL_IP  = 32'hC0_A8_01_0B,
	parameter [15:0] C_LOCAL_PORT = 16'd5000,
	parameter [15:0] C_CONTROL_PORT = 16'd5002,
	parameter [31:0] C_DEST_IP   = 32'hC0_A8_01_69,
	parameter [15:0] C_DEST_PORT = 16'd5001,
	parameter [4:0] C_PHY_ADDR = 5'd1,
	// Q8：统一四源 DDR/四宫格；置 0 保留原 C7 单源回归路径。
	parameter C_QUAD_H1_PATTERN = 1,
	parameter C_HDMI_SOURCE3_ENABLE = 1,
	// Historical SFP0 port names are retained; decoded video is the verified
	// SFP1 (board SFP2) / HSST Channel 2 path and occupies Q8 source2.
	parameter C_FIBER_SOURCE2_ENABLE = 1,
	// Q5 CMOS1 source path. CMOS2 remains a reserved/local-pattern source.
	parameter C_CMOS1_ENABLE = 1,
	// Diagnostic Golden/Q8 cross selector.  Zero preserves the normal build.
	parameter integer C_Q8_DIAG_PATH = 0,
	// HDMI输出选择：生产构建必须经过monitor compositor（包含左侧AVM）。
	// 仅在需要观察原始Q8四宫格时显式置1；不改变Q8读写链本身。
	parameter integer C_HDMI_QUAD_OUTPUT = 0,
	// Board-verified HMIC write compatibility mode: one 256-bit word per AW.
	// AWLEN=15 passes the behavioral model but corrupts source0 on hardware.
	parameter integer C_Q8_WRITE_BURST_BEATS = 1,
	// Flash LUT 启动读取。默认关闭，避免在未核实板级管脚/配置镜像前改变既有位流。
	parameter integer C_FLASH_ENABLE = 1,
	parameter integer C_FLASH_SELECT = 0,
	parameter [23:0] C_FLASH_LUT_BASE_ADDR = 24'h800000,
	// LUT来源：0=UDP在线，1=Flash启动，2=SD启动。本交付构建固定为2。
	parameter integer C_LUT_SOURCE = 2
)(
	input i_sys_clk,
	input i_button_rst_n,
	// DDR3 reference clock and physical interface.  Names match the current
	// HMIC_S-generated constraint set for the MES2L676-100HP board.
	input ref_clk_p,
	input ref_clk_n,
	output mem_cs_n,
	output mem_rst_n,
	output mem_ck,
	output mem_ck_n,
	output mem_cke,
	output mem_ras_n,
	output mem_cas_n,
	output mem_we_n,
	output mem_odt,
	output [14:0] mem_a,
	output [2:0] mem_ba,
	inout [3:0] mem_dqs,
	inout [3:0] mem_dqs_n,
	inout [31:0] mem_dq,
	output [3:0] mem_dm,

	input i_rgmii_rxc,
	input i_rgmii_rx_ctl,
	input [3:0] i_rgmii_rxd,
	output o_rgmii_txc,
	output o_rgmii_tx_ctl,
	output [3:0] o_rgmii_txd,
	output o_phy_reset_n,
	output o_mdc,
	inout io_mdio,

	// U15 user SPI0 Flash.  SCK is routed through the dedicated GTP_CFGCLK
	// resource; only CS and the four DQ pads are exposed for board constraints.
	output o_flash_cs_n,
	output o_flash1_cs_n,
	inout [3:0] io_flash_dq,

	// MicroSD SPI启动装载：27MHz系统时钟域，CLK/MOSI/CS输出，DAT0/MISO输入。
	output o_sd_clk,
	output o_sd_mosi,
	input i_sd_miso,
	output o_sd_cs,

	output o_hdmi_pclk,
	output o_hdmi_vs,
	output o_hdmi_hs,
	output o_hdmi_de,
	output [23:0] o_hdmi_rgb,
	output o_hdmi_scl,
	inout io_hdmi_sda,

	// HDMI-IN / MS7200 parallel RGB input.  Pin locations are taken from
	// demo/2_10_hdmi_loop; this port group remains in the RX PCLK domain.
	input i_hdmi_rx_pclk,
	input i_hdmi_rx_vs,
	input i_hdmi_rx_hs,
	input i_hdmi_rx_de,
	input [23:0] i_hdmi_rx_rgb,

	// OV5640 CMOS1 DVP input. The camera remains in its original PCLK domain.
	input i_cmos1_pclk,
	input i_cmos1_vsync,
	input i_cmos1_href,
	input [7:0] i_cmos1_data,
	inout io_cmos1_scl,
	inout io_cmos1_sda,
	output o_cmos1_reset,

	// Historical port names remain for PDS compatibility.  The generated IP
	// behind this boundary is RX-only HSST Channel 2 on board SFP1/SFP2.
	input i_p_refckn_1,
	input i_p_refckp_1,
	input i_sfp0_rxn,
	input i_sfp0_rxp,
	input i_sfp0_los,
	output o_sfp0_tx_disable,

	// PCIe x4 独立参考时钟、PERST# 和差分收发器接口。不得复用 DDR ref_clk 或 SFP HSST。
	input i_pcie_ref_clk_p,
	input i_pcie_ref_clk_n,
	input i_pcie_perst_n,
	input [3:0] i_pcie_rxp,
	input [3:0] i_pcie_rxn,
	output [3:0] o_pcie_txp,
	output [3:0] o_pcie_txn,

	output [3:0] o_status_led,
	// UART disabled: retain board TX pin C24 at the idle high level.
	output o_uart_tx
);
	localparam integer C_USE_Q8_WRITER = (C_Q8_DIAG_PATH == 0) ?
		((C_QUAD_H1_PATTERN != 0) ? 1 : 0) :
		(((C_Q8_DIAG_PATH == 2) || (C_Q8_DIAG_PATH == 4)) ? 1 : 0);
	localparam integer C_USE_Q8_READER = (C_Q8_DIAG_PATH == 0) ?
		((C_QUAD_H1_PATTERN != 0) ? 1 : 0) :
		(((C_Q8_DIAG_PATH == 3) || (C_Q8_DIAG_PATH == 4)) ? 1 : 0);
	localparam integer C_USE_QUAD_OUTPUT = (C_HDMI_QUAD_OUTPUT != 0) ? 1 : 0;
	localparam integer C_Q8_MULTI_SOURCE =
		((C_Q8_DIAG_PATH == 0) && (C_QUAD_H1_PATTERN != 0)) ? 1 : 0;
	localparam integer C_QUAD_STREAM_ALL =
		((C_Q8_DIAG_PATH == 0) && (C_QUAD_H1_PATTERN != 0)) ||
		(C_Q8_DIAG_PATH == 3) || (C_Q8_DIAG_PATH == 4);
	// UART监视及发送链已移除，板载TX保持空闲；没有状态采样时钟或复位。
	assign o_uart_tx = 1'b1;

	// Shared subsystem interfaces; no combinational processing in board top.
	wire c5_ddr_clk;
	wire rgmii_clk;
	wire monitor_mux_ack_valid;
	wire [143:0] monitor_mux_ack_payload;
	wire [15:0] monitor_mux_ack_length;
	wire monitor_mux_ack_ready;
	wire udp_ctrl_start;
	wire udp_ctrl_valid;
	wire [7:0] udp_ctrl_data;
	wire udp_ctrl_end;
	wire [15:0] udp_ctrl_length;
	wire udp_ctrl_error;
	wire c7_frame_grant;
	wire [1:0] c7_frame_grant_slot;
	wire c6_frame_prepare;
	wire c6_frame_commit;
	wire c6_frame_abort;
	wire c5_input_pop;
	wire [255:0] c5_input_word;
	wire c5_input_empty;
	wire [15:0] c6_net_rd_water_level;
	wire [3:0] quad_grant_toggle;
	wire [3:0] quad_word_pop;
	wire [255:0] hdmi_source3_word_data;
	wire hdmi_source3_word_empty;
	wire [15:0] hdmi_source3_rd_level;
	wire hdmi_source3_prepare_ddr;
	wire hdmi_source3_complete_ddr;
	wire hdmi_source3_abort_ddr;
	wire [255:0] cmos1_word_data;
	wire cmos1_word_empty;
	wire [15:0] cmos1_rd_level;
	wire cmos1_prepare_ddr;
	wire cmos1_complete_ddr;
	wire cmos1_abort_ddr;
	wire fiber_pll_ready;
	wire [255:0] fiber_source2_word_data;
	wire fiber_source2_word_empty;
	wire [15:0] fiber_source2_rd_level;
	wire fiber_source2_prepare_ddr;
	wire fiber_source2_complete_ddr;
	wire fiber_source2_abort_ddr;
	wire [63:0] fiber_lane_snapshot;
	wire fiber_lane_valid;
	wire fiber_lane_stale;
	wire hdmi_pixel_clk;
	wire hdmi_pixel_rst_n;
	wire hdmi_pclk;
	wire quad_h1_pclk;
	wire quad_h1_vs;
	wire quad_h1_hs;
	wire quad_h1_de;
	wire [23:0] quad_h1_rgb;
	wire quad_content_valid;
	wire [11:0] quad_display_x;
	wire [11:0] quad_display_y;
	wire [1:0] quad_display_tile;
	wire [9:0] quad_tile_x;
	wire [8:0] quad_tile_y;
	wire [3:0] quad_source_present;
	wire quad_source_present_valid;
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
	wire c5_underflow_event;
	wire [7:0] quad_display_slot;
	wire [3:0] quad_display_valid;
	wire [3:0] quad_retire_done;
	wire quad_snapshot;
	wire quad_read_urgent;
	wire [3:0] quad_frame_aborted;
	wire quad_ddr_rst_n;
	wire [27:0] avm_quad_awaddr;
	wire [3:0] avm_quad_awlen;
	wire avm_quad_awvalid;
	wire [255:0] avm_quad_wdata;
	wire [31:0] avm_quad_wstrb;
	wire [2:0] avm_write_client_awready;
	wire [2:0] avm_write_client_wready;
	wire [27:0] monitor_quad_araddr;
	wire [3:0] monitor_quad_arlen;
	wire monitor_quad_arvalid;
	wire monitor_quad_arready;
	wire monitor_quad_rvalid;
	wire [11:0] monitor_slot_held;
	wire [63:0] monitor_display_epoch;
	wire algo_common_reset_n;
	wire isp_ext_valid;
	wire isp_ext_query;
	wire [519:0] isp_ext_config;
	wire isp_ext_ready;
	wire isp_ext_done;
	wire [7:0] isp_ext_status;
	wire [519:0] isp_ext_snapshot;
	wire [15:0] isp_ext_epoch;
	wire [1:0] isp_ext_source;
	wire algo_ext_valid;
	wire algo_ext_query;
	wire [63:0] algo_ext_config;
	wire algo_ext_ready;
	wire algo_ext_done;
	wire [7:0] algo_ext_status;
	wire [63:0] algo_ext_snapshot;
	wire [15:0] algo_ext_epoch;
	wire [1:0] algo_ext_source;
	wire algo_net_rst_n;
	wire algo_pixel_rst_n;
	wire algo_ack_valid;
	wire [143:0] algo_ack_payload;
	wire [15:0] algo_ack_length;
	wire algo_ack_ready;
	wire hdmi_pll_lock;
	wire quad_base_vs;
	wire quad_base_hs;
	wire quad_base_de;
	wire [23:0] quad_base_rgb;
	wire [23:0] dehaze_algo_rgb;
	wire [159:0] dehaze_algo_tag;
	wire [159:0] algo_output_tag;
	wire hdmi_algo_vs;
	wire [23:0] hdmi_algo_rgb;
	wire [1:0] night_level;
	wire dehaze_hud_enable;
	wire monitor_algo_start;
	wire monitor_algo_valid;
	wire monitor_algo_end;
	wire monitor_algo_error;
	wire [7:0] monitor_algo_data;
	wire [15:0] monitor_algo_length;
	wire isp_ack_valid;
	wire [143:0] isp_ack_payload;
	wire [15:0] isp_ack_length;
	wire isp_ack_ready;
	wire isp_start;
	wire isp_valid;
	wire [7:0] isp_data;
	wire isp_end;
	wire [15:0] isp_length;
	wire isp_error;
	wire [8:0] hud_state;
	wire [159:0] monitor_composite_tag;
	wire [23:0] monitor_composite_rgb;
	wire [8:0] monitor_composite_hud;
	wire hud_wr_en;
	wire [11:0] hud_wr_addr;
	wire [31:0] hud_wr_data;
	wire [3:0] hud_wr_strb;
	wire [11:0] hud_rd_addr;
	wire [127:0] hud_rd_window;
	wire behavior_action_valid_net;
	wire [15:0] behavior_action_sequence_net;
	wire [2:0] behavior_action_type_net /* synthesis syn_preserve = 1 */;
	wire behavior_action_request_net /* synthesis syn_preserve = 1 */;
	wire behavior_action_packet_error_net;
	wire behavior_action_ready_net;
	wire behavior_action_online_net;
	wire behavior_ack_valid;
	wire [143:0] behavior_ack_payload;
	wire [15:0] behavior_ack_length;
	wire behavior_ack_ready;
	wire [31:0] behavior_result;
	wire [31:0] behavior_effective;
	wire [175:0] behavior_display_snapshot;
	wire scene_commit_valid;
	wire [111:0] scene_commit_payload;
	wire behavior_control_rst_n;
	wire pcie_pclk_div2;
	wire pcie_core_rst_n;
	wire pcie_link_up;

// Ethernet协议、PGV1和网络到DDR FIFO统一入口；无新增数据拍数。
	ethernet_top
	#(
		.C_LOCAL_MAC(C_LOCAL_MAC),
		.C_LOCAL_IP(C_LOCAL_IP),
		.C_LOCAL_PORT(C_LOCAL_PORT),
		.C_CONTROL_PORT(C_CONTROL_PORT),
		.C_DEST_IP(C_DEST_IP),
		.C_DEST_PORT(C_DEST_PORT)
	)
	ethernet_top_Inst_network
	(
		.i_rstn(i_button_rst_n),
		.i_ddr_clk(c5_ddr_clk),
		.i_ddr_rstn(i_button_rst_n),
		.i_rgmii_rxc(i_rgmii_rxc),
		.i_rgmii_rx_ctl(i_rgmii_rx_ctl),
		.i_rgmii_rxd(i_rgmii_rxd),
		.o_rgmii_clk(rgmii_clk),
		.o_rgmii_txc(o_rgmii_txc),
		.o_rgmii_tx_ctl(o_rgmii_tx_ctl),
		.o_rgmii_txd(o_rgmii_txd),
		.i_ack_valid(monitor_mux_ack_valid),
		.i_ack_payload(monitor_mux_ack_payload),
		.i_ack_length(monitor_mux_ack_length),
		.o_ack_ready(monitor_mux_ack_ready),
		.o_video_start(),
		.o_video_valid(),
		.o_video_data(),
		.o_video_end(),
		.o_video_length(),
		.o_ctrl_start(udp_ctrl_start),
		.o_ctrl_valid(udp_ctrl_valid),
		.o_ctrl_data(udp_ctrl_data),
		.o_ctrl_end(udp_ctrl_end),
		.o_ctrl_length(udp_ctrl_length),
		.o_ctrl_error(udp_ctrl_error),
		.i_frame_grant(c7_frame_grant),
		.i_frame_grant_slot(c7_frame_grant_slot),
		.o_frame_prepare(c6_frame_prepare),
		.o_frame_commit(c6_frame_commit),
		.o_frame_abort(c6_frame_abort),
		.o_frame_active(),
		.o_frame_slot(),
		.o_frame_sequence(),
		.o_crc_error_count(),
		.o_sequence_error_count(),
		.o_length_error_count(),
		.o_dropped_frame_count(),
		.o_packer_overflow(),
		.i_word_pop(c5_input_pop),
		.o_word_data(c5_input_word),
		.o_word_empty(c5_input_empty),
		.o_wr_water_level(),
		.o_rd_water_level(c6_net_rd_water_level)
	);

	// 上位机行为包：BV + sequence(LE) + action_type + valid。
	// 与既有算法/路径控制包共用control流，未知magic仍由原路径忽略。
	vehicle_behavior_action_rx u_vehicle_behavior_action_rx(
		.i_clk(rgmii_clk), .i_rst_n(i_button_rst_n),
		.i_start(udp_ctrl_start), .i_valid(udp_ctrl_valid), .i_data(udp_ctrl_data),
		.i_end(udp_ctrl_end), .i_length(udp_ctrl_length), .i_error(udp_ctrl_error),
		.i_ack_ready(behavior_ack_ready),
		.i_action_ready(behavior_action_ready_net),
		.i_action_online(behavior_action_online_net),
		.o_action_valid(behavior_action_valid_net),
		.o_action_sequence(behavior_action_sequence_net),
		.o_action_type(behavior_action_type_net),
		.o_action_request(behavior_action_request_net),
		.o_packet_error(behavior_action_packet_error_net),
		.o_ack_valid(behavior_ack_valid), .o_ack_payload(behavior_ack_payload),
		.o_ack_length(behavior_ack_length),
		.o_ack_drop_count()
	);

	video_sources_top #(
		.C_Q8_MULTI_SOURCE(C_Q8_MULTI_SOURCE),
		.C_HDMI_SOURCE3_ENABLE(C_HDMI_SOURCE3_ENABLE),
		.C_CMOS1_ENABLE(C_CMOS1_ENABLE),
		.C_FIBER_SOURCE2_ENABLE(C_FIBER_SOURCE2_ENABLE)
	) u_video_sources_top(
		.i_sys_clk(i_sys_clk),
		.i_button_rst_n(i_button_rst_n),
		.i_hdmi_rx_pclk(i_hdmi_rx_pclk),
		.i_hdmi_rx_vs(i_hdmi_rx_vs),
		.i_hdmi_rx_hs(i_hdmi_rx_hs),
		.i_hdmi_rx_de(i_hdmi_rx_de),
		.i_hdmi_rx_rgb(i_hdmi_rx_rgb),
		.i_cmos1_pclk(i_cmos1_pclk),
		.i_cmos1_vsync(i_cmos1_vsync),
		.i_cmos1_href(i_cmos1_href),
		.i_cmos1_data(i_cmos1_data),
		.io_cmos1_scl(io_cmos1_scl),
		.io_cmos1_sda(io_cmos1_sda),
		.o_cmos1_reset(o_cmos1_reset),
		.i_p_refckn_1(i_p_refckn_1),
		.i_p_refckp_1(i_p_refckp_1),
		.i_sfp0_rxn(i_sfp0_rxn),
		.i_sfp0_rxp(i_sfp0_rxp),
		.i_sfp0_los(i_sfp0_los),
		.i_quad_grant_toggle(quad_grant_toggle),
		.i_quad_word_pop(quad_word_pop),
		.i_ddr_clk(c5_ddr_clk),
		.i_ddr_rst_n(i_button_rst_n),
		.o_hdmi_rx_tile_valid(),
		.o_hdmi_rx_tile_rgb565(),
		.o_hdmi_rx_tile_frame_start(),
		.o_hdmi_rx_tile_frame_end(),
		.o_hdmi_rx_tile_frame_error(),
		.o_hdmi_rx_vs_active(),
		.o_hdmi_rx_hs_active(),
		.o_hdmi_rx_de_active(),
		.o_hdmi_source3_word_data(hdmi_source3_word_data),
		.o_hdmi_source3_word_empty(hdmi_source3_word_empty),
		.o_hdmi_source3_wr_level(),
		.o_hdmi_source3_rd_level(hdmi_source3_rd_level),
		.o_hdmi_source3_prepare_ddr(hdmi_source3_prepare_ddr),
		.o_hdmi_source3_complete_ddr(hdmi_source3_complete_ddr),
		.o_hdmi_source3_abort_ddr(hdmi_source3_abort_ddr),
		.o_cmos1_word_data(cmos1_word_data),
		.o_cmos1_word_empty(cmos1_word_empty),
		.o_cmos1_wr_level(),
		.o_cmos1_rd_level(cmos1_rd_level),
		.o_cmos1_prepare_ddr(cmos1_prepare_ddr),
		.o_cmos1_complete_ddr(cmos1_complete_ddr),
		.o_cmos1_abort_ddr(cmos1_abort_ddr),
		.o_cmos1_sccb_scl(),
		.o_cmos1_sccb_sda_drive_low(),
		.o_cmos1_sccb_done(),
		.o_cmos1_sccb_error(),
		.o_fiber_rx_clk(),
		.o_fiber_pll_ready(fiber_pll_ready),
		.o_fiber_rx_lane_done(),
		.o_fiber_rx_lane_ready(),
		.o_fiber_rxd(),
		.o_fiber_rxk(),
		.o_fiber_decode_error(),
		.o_fiber_disparity_error(),
		.o_fiber_signal_detect(),
		.o_fiber_cdr_align(),
		.o_fiber_watchdog_status(),
		.o_fiber_aligned_valid(),
		.o_fiber_aligned_rxd(),
		.o_fiber_aligned_rxk(),
		.o_fiber_align_lock(),
		.o_fiber_align_error(),
		.o_fiber_frame_start(),
		.o_fiber_frame_end(),
		.o_fiber_frame_error(),
		.o_fiber_word256_valid(),
		.o_fiber_word256_data(),
		.o_fiber_pixel_x(),
		.o_fiber_pixel_y(),
		.o_fiber_good_frame_count(),
		.o_fiber_bad_frame_count(),
		.o_fiber_source2_word_data(fiber_source2_word_data),
		.o_fiber_source2_word_empty(fiber_source2_word_empty),
		.o_fiber_source2_wr_level(),
		.o_fiber_source2_rd_level(fiber_source2_rd_level),
		.o_fiber_source2_prepare_ddr(fiber_source2_prepare_ddr),
		.o_fiber_source2_complete_ddr(fiber_source2_complete_ddr),
		.o_fiber_source2_abort_ddr(fiber_source2_abort_ddr),
		.o_fiber_lane_snapshot(fiber_lane_snapshot),
		.o_fiber_lane_valid(fiber_lane_valid),
		.o_fiber_lane_stale(fiber_lane_stale)
	);

	frame_buffer_top #(
		.C_CMOS1_ENABLE(C_CMOS1_ENABLE),
		.C_FIBER_SOURCE2_ENABLE(C_FIBER_SOURCE2_ENABLE),
		.C_HDMI_SOURCE3_ENABLE(C_HDMI_SOURCE3_ENABLE),
		.C_Q8_MULTI_SOURCE(C_Q8_MULTI_SOURCE),
		.C_Q8_WRITE_BURST_BEATS(C_Q8_WRITE_BURST_BEATS),
		.C_QUAD_STREAM_ALL(C_QUAD_STREAM_ALL),
		.C_USE_Q8_READER(C_USE_Q8_READER),
		.C_USE_Q8_WRITER(C_USE_Q8_WRITER)
	) u_frame_buffer_top(
		.i_rgmii_clk(rgmii_clk),
		.i_hdmi_pixel_clk(hdmi_pixel_clk),
		.i_hdmi_pixel_rst_n(hdmi_pixel_rst_n),
		.o_hdmi_pclk(hdmi_pclk),
		.i_hdmi_source3_word_empty(hdmi_source3_word_empty),
		.i_hdmi_source3_word_data(hdmi_source3_word_data),
		.i_hdmi_source3_rd_level(hdmi_source3_rd_level),
		.i_hdmi_source3_prepare_ddr(hdmi_source3_prepare_ddr),
		.i_hdmi_source3_complete_ddr(hdmi_source3_complete_ddr),
		.i_hdmi_source3_abort_ddr(hdmi_source3_abort_ddr),
		.i_cmos1_word_empty(cmos1_word_empty),
		.i_cmos1_word_data(cmos1_word_data),
		.i_cmos1_rd_level(cmos1_rd_level),
		.i_cmos1_prepare_ddr(cmos1_prepare_ddr),
		.i_cmos1_complete_ddr(cmos1_complete_ddr),
		.i_cmos1_abort_ddr(cmos1_abort_ddr),
		.o_quad_h1_pclk(quad_h1_pclk),
		.o_quad_h1_vs(quad_h1_vs),
		.o_quad_h1_hs(quad_h1_hs),
		.o_quad_h1_de(quad_h1_de),
		.o_quad_h1_rgb(quad_h1_rgb),
		.o_quad_content_valid(quad_content_valid),
		.o_quad_display_x(quad_display_x),
		.o_quad_display_y(quad_display_y),
		.o_quad_display_tile(quad_display_tile),
		.o_quad_tile_x(quad_tile_x),
		.o_quad_tile_y(quad_tile_y),
		.o_quad_source_present(quad_source_present),
		.o_quad_source_present_valid(quad_source_present_valid),
		.i_fiber_source2_word_data(fiber_source2_word_data),
		.i_fiber_source2_word_empty(fiber_source2_word_empty),
		.i_fiber_source2_rd_level(fiber_source2_rd_level),
		.i_fiber_source2_prepare_ddr(fiber_source2_prepare_ddr),
		.i_fiber_source2_complete_ddr(fiber_source2_complete_ddr),
		.i_fiber_source2_abort_ddr(fiber_source2_abort_ddr),
		.o_c5_ddr_clk(c5_ddr_clk),
		.o_c5_ddr_init_done(c5_ddr_init_done),
		.i_c6_frame_prepare(c6_frame_prepare),
		.i_c6_frame_commit(c6_frame_commit),
		.i_c6_frame_abort(c6_frame_abort),
		.o_c7_frame_grant(c7_frame_grant),
		.o_c7_frame_grant_slot(c7_frame_grant_slot),
		.o_c7_frame_written(),
		.o_c7_frame_failed(),
		.o_c7_display_valid(),
		.i_c5_input_word(c5_input_word),
		.i_c5_input_empty(c5_input_empty),
		.o_c5_input_pop(c5_input_pop),
		.i_c5_awaddr(c5_awaddr),
		.i_c5_awlen(c5_awlen),
		.o_c5_awready(c5_awready),
		.i_c5_awvalid(c5_awvalid),
		.i_c5_wdata(c5_wdata),
		.i_c5_wstrb(c5_wstrb),
		.o_c5_wready(c5_wready),
		.i_c5_araddr(c5_araddr),
		.i_c5_arlen(c5_arlen),
		.o_c5_arready(c5_arready),
		.i_c5_arvalid(c5_arvalid),
		.o_c5_rdata(c5_rdata),
		.o_c5_rvalid(c5_rvalid),
		.i_c6_net_rd_water_level(c6_net_rd_water_level),
		.o_c5_underflow_event(c5_underflow_event),
		.o_c7_hdmi_rd_water_level(),
		.o_quad_grant_toggle(quad_grant_toggle),
		.o_quad_word_pop(quad_word_pop),
		.o_quad_display_slot(quad_display_slot),
		.o_quad_display_valid(quad_display_valid),
		.o_quad_retire_done(quad_retire_done),
		.o_quad_snapshot(quad_snapshot),
		.o_quad_read_urgent(quad_read_urgent),
		.o_quad_frame_aborted(quad_frame_aborted),
		.o_quad_ddr_rst_n(quad_ddr_rst_n),
		.o_avm_quad_awaddr(avm_quad_awaddr),
		.o_avm_quad_awlen(avm_quad_awlen),
		.o_avm_quad_awvalid(avm_quad_awvalid),
		.o_avm_quad_wdata(avm_quad_wdata),
		.o_avm_quad_wstrb(avm_quad_wstrb),
		.i_avm_write_client_awready(avm_write_client_awready),
		.i_avm_write_client_wready(avm_write_client_wready),
		.o_monitor_quad_araddr(monitor_quad_araddr),
		.o_monitor_quad_arlen(monitor_quad_arlen),
		.o_monitor_quad_arvalid(monitor_quad_arvalid),
		.i_monitor_quad_arready(monitor_quad_arready),
		.i_monitor_quad_rvalid(monitor_quad_rvalid),
		.i_monitor_slot_held(monitor_slot_held),
		.o_monitor_display_epoch(monitor_display_epoch),
		.i_button_rst_n(i_button_rst_n),
		.i_ref_clk_p(ref_clk_p),
		.i_ref_clk_n(ref_clk_n),
		.o_mem_cs_n(mem_cs_n),
		.o_mem_rst_n(mem_rst_n),
		.o_mem_ck(mem_ck),
		.o_mem_ck_n(mem_ck_n),
		.o_mem_cke(mem_cke),
		.o_mem_ras_n(mem_ras_n),
		.o_mem_cas_n(mem_cas_n),
		.o_mem_we_n(mem_we_n),
		.o_mem_odt(mem_odt),
		.o_mem_a(mem_a),
		.o_mem_ba(mem_ba),
		.mem_dqs(mem_dqs),
		.mem_dqs_n(mem_dqs_n),
		.mem_dq(mem_dq),
		.o_mem_dm(mem_dm)
	);

	video_algorithm_top u_video_algorithm_top(
		.i_isp_ext_valid(isp_ext_valid),
		.i_isp_ext_query(isp_ext_query),
		.i_isp_ext_config(isp_ext_config),
		.o_isp_ext_ready(isp_ext_ready),
		.o_isp_ext_done(isp_ext_done),
		.o_isp_ext_status(isp_ext_status),
		.o_isp_ext_snapshot(isp_ext_snapshot),
		.o_isp_ext_epoch(isp_ext_epoch),
		.o_isp_ext_source(isp_ext_source),
		.i_algo_ext_valid(algo_ext_valid),
		.i_algo_ext_query(algo_ext_query),
		.i_algo_ext_config(algo_ext_config),
		.o_algo_ext_ready(algo_ext_ready),
		.o_algo_ext_done(algo_ext_done),
		.o_algo_ext_status(algo_ext_status),
		.o_algo_ext_snapshot(algo_ext_snapshot),
		.o_algo_ext_epoch(algo_ext_epoch),
		.o_algo_ext_source(algo_ext_source),
		.i_rgmii_clk(rgmii_clk),
		.o_algo_common_reset_n(algo_common_reset_n),
		.o_algo_net_rst_n(algo_net_rst_n),
		.o_algo_pixel_rst_n(algo_pixel_rst_n),
		.o_algo_ack_valid(algo_ack_valid),
		.o_algo_ack_payload(algo_ack_payload),
		.o_algo_ack_length(algo_ack_length),
		.i_algo_ack_ready(algo_ack_ready),
		.o_isp_ack_valid(isp_ack_valid),.o_isp_ack_payload(isp_ack_payload),
		.o_isp_ack_length(isp_ack_length),.i_isp_ack_ready(isp_ack_ready),
		.i_isp_start(isp_start),.i_isp_valid(isp_valid),.i_isp_data(isp_data),
		.i_isp_end(isp_end),.i_isp_length(isp_length),.i_isp_error(isp_error),
		.i_hdmi_pixel_clk(hdmi_pixel_clk),
		.i_hdmi_pll_lock(hdmi_pll_lock),
		.i_quad_base_vs(quad_base_vs),
		.i_quad_base_hs(quad_base_hs),
		.i_quad_base_de(quad_base_de),
		.i_quad_base_rgb(quad_base_rgb),
		.i_quad_content_valid(quad_content_valid),
		.i_quad_display_x(quad_display_x),
		.i_quad_display_y(quad_display_y),
		.i_quad_display_tile(quad_display_tile),
		.i_quad_tile_x(quad_tile_x),
		.i_quad_tile_y(quad_tile_y),
		.i_quad_source_present(quad_source_present),
		.i_quad_source_present_valid(quad_source_present_valid),
		.o_dehaze_algo_rgb(dehaze_algo_rgb),
		.o_dehaze_algo_tag(dehaze_algo_tag),
		.o_algo_output_tag(algo_output_tag),
		.o_hdmi_algo_vs(hdmi_algo_vs),
		.o_hdmi_algo_rgb(hdmi_algo_rgb),
		.o_night_level(night_level),
		.o_dehaze_hud_enable(dehaze_hud_enable),
		.i_monitor_algo_start(monitor_algo_start),
		.i_monitor_algo_valid(monitor_algo_valid),
		.i_monitor_algo_end(monitor_algo_end),
		.i_monitor_algo_error(monitor_algo_error),
		.i_monitor_algo_data(monitor_algo_data),
		.i_monitor_algo_length(monitor_algo_length),
		.i_button_rst_n(i_button_rst_n)
	);

	avm_monitor_top #(
		.C_FLASH_ENABLE(C_FLASH_ENABLE),
		.C_FLASH_LUT_BASE_ADDR(C_FLASH_LUT_BASE_ADDR),
		.C_FLASH_SELECT(C_FLASH_SELECT),
		.C_LUT_SOURCE(C_LUT_SOURCE)
	) u_avm_monitor_top(
		.i_rgmii_clk(rgmii_clk),
		.i_udp_ctrl_valid(udp_ctrl_valid),
		.i_udp_ctrl_data(udp_ctrl_data),
		.i_udp_ctrl_length(udp_ctrl_length),
		.i_udp_ctrl_start(udp_ctrl_start),
		.i_udp_ctrl_end(udp_ctrl_end),
		.i_udp_ctrl_error(udp_ctrl_error),
		.i_algo_common_reset_n(algo_common_reset_n),
		.i_algo_net_rst_n(algo_net_rst_n),
		.i_algo_pixel_rst_n(algo_pixel_rst_n),
		.i_algo_ack_valid(algo_ack_valid),
		.i_algo_ack_payload(algo_ack_payload),
		.i_algo_ack_length(algo_ack_length),
		.o_algo_ack_ready(algo_ack_ready),
		.i_isp_ack_valid(isp_ack_valid),.i_isp_ack_payload(isp_ack_payload),
		.i_isp_ack_length(isp_ack_length),.o_isp_ack_ready(isp_ack_ready),
		.o_isp_start(isp_start),.o_isp_valid(isp_valid),.o_isp_data(isp_data),
		.o_isp_end(isp_end),.o_isp_length(isp_length),.o_isp_error(isp_error),
		.i_hdmi_pixel_clk(hdmi_pixel_clk),
		.i_algo_output_tag(algo_output_tag),
		.i_hdmi_algo_rgb(hdmi_algo_rgb),
		.i_hud_state(hud_state),
		.i_c5_ddr_clk(c5_ddr_clk),
		.i_c5_ddr_init_done(c5_ddr_init_done),
		.o_c5_awaddr(c5_awaddr),
		.o_c5_awlen(c5_awlen),
		.i_c5_awready(c5_awready),
		.o_c5_awvalid(c5_awvalid),
		.o_c5_wdata(c5_wdata),
		.o_c5_wstrb(c5_wstrb),
		.i_c5_wready(c5_wready),
		.o_c5_araddr(c5_araddr),
		.o_c5_arlen(c5_arlen),
		.i_c5_arready(c5_arready),
		.o_c5_arvalid(c5_arvalid),
		.i_c5_rdata(c5_rdata),
		.i_c5_rvalid(c5_rvalid),
		.i_quad_display_slot(quad_display_slot),
		.i_quad_display_valid(quad_display_valid),
		.i_quad_retire_done(quad_retire_done),
		.i_quad_snapshot(quad_snapshot),
		.i_quad_read_urgent(quad_read_urgent),
		.i_quad_ddr_rst_n(quad_ddr_rst_n),
		.o_avm_render_busy(),
		.o_avm_frame_toggle_dbg(),
		.o_avm_renderer_ready_dbg(),
		.o_avm_free_available_dbg(),
		.o_avm_pixel_alive_dbg(),
		.o_avm_sd_busy_dbg(),
		.o_avm_sd_ready_dbg(),
		.o_avm_sd_error_dbg(),
		.o_avm_sd_committed_beats_dbg(),
		.o_avm_sd_loaded_bytes_dbg(),
		.i_avm_quad_awaddr(avm_quad_awaddr),
		.i_avm_quad_awlen(avm_quad_awlen),
		.i_avm_quad_awvalid(avm_quad_awvalid),
		.i_avm_quad_wdata(avm_quad_wdata),
		.i_avm_quad_wstrb(avm_quad_wstrb),
		.o_avm_write_client_awready(avm_write_client_awready),
		.o_avm_write_client_wready(avm_write_client_wready),
		.o_avm_pixel_valid(),
		.o_monitor_algo_start(monitor_algo_start),
		.o_monitor_algo_valid(monitor_algo_valid),
		.o_monitor_algo_end(monitor_algo_end),
		.o_monitor_algo_error(monitor_algo_error),
		.o_monitor_algo_data(monitor_algo_data),
		.o_monitor_algo_length(monitor_algo_length),
		.o_monitor_mux_ack_valid(monitor_mux_ack_valid),
		.i_monitor_mux_ack_ready(monitor_mux_ack_ready),
		.o_monitor_mux_ack_payload(monitor_mux_ack_payload),
		.o_monitor_mux_ack_length(monitor_mux_ack_length),
		.i_behavior_ack_valid(behavior_ack_valid),
		.i_behavior_ack_payload(behavior_ack_payload),
		.i_behavior_ack_length(behavior_ack_length),
		.o_behavior_ack_ready(behavior_ack_ready),
		.o_monitor_composite_tag(monitor_composite_tag),
		.o_monitor_composite_rgb(monitor_composite_rgb),
		.o_monitor_composite_hud(monitor_composite_hud),
		.i_monitor_quad_araddr(monitor_quad_araddr),
		.i_monitor_quad_arlen(monitor_quad_arlen),
		.i_monitor_quad_arvalid(monitor_quad_arvalid),
		.o_monitor_quad_arready(monitor_quad_arready),
		.o_monitor_quad_rvalid(monitor_quad_rvalid),
		.o_monitor_slot_held(monitor_slot_held),
		.o_monitor_lease_ready(),
		.o_avm_pixel_frame_ready(),
		.i_monitor_display_epoch(monitor_display_epoch),
		.i_sys_clk(i_sys_clk),
		.i_button_rst_n(i_button_rst_n),
		.o_flash_cs_n(o_flash_cs_n),
		.o_flash1_cs_n(o_flash1_cs_n),
		.io_flash_dq(io_flash_dq),
		.o_sd_clk(o_sd_clk),
		.o_sd_mosi(o_sd_mosi),
		.i_sd_miso(i_sd_miso),
		.o_sd_cs(o_sd_cs)
	);

	pcie_top u_pcie_top(
		.i_image_control_clk(rgmii_clk),
		.i_image_control_rst_n(algo_net_rst_n),
		.o_isp_ext_valid(isp_ext_valid),
		.o_isp_ext_query(isp_ext_query),
		.o_isp_ext_config(isp_ext_config),
		.i_isp_ext_ready(isp_ext_ready),
		.i_isp_ext_done(isp_ext_done),
		.i_isp_ext_status(isp_ext_status),
		.i_isp_ext_snapshot(isp_ext_snapshot),
		.i_isp_ext_epoch(isp_ext_epoch),
		.i_isp_ext_source(isp_ext_source),
		.o_algo_ext_valid(algo_ext_valid),
		.o_algo_ext_query(algo_ext_query),
		.o_algo_ext_config(algo_ext_config),
		.i_algo_ext_ready(algo_ext_ready),
		.i_algo_ext_done(algo_ext_done),
		.i_algo_ext_status(algo_ext_status),
		.i_algo_ext_snapshot(algo_ext_snapshot),
		.i_algo_ext_epoch(algo_ext_epoch),
		.i_algo_ext_source(algo_ext_source),
		.i_ref_clk(i_sys_clk),
		.i_fiber_lane_snapshot(fiber_lane_snapshot),
		.i_fiber_lane_valid(fiber_lane_valid),
		.i_fiber_lane_stale(fiber_lane_stale),
		.i_control_rst_n(behavior_control_rst_n),
		.i_scene_commit_valid(scene_commit_valid),
		.i_scene_commit_payload(scene_commit_payload),
		.o_display_snapshot(behavior_display_snapshot),
		.i_hdmi_pixel_clk(hdmi_pixel_clk),
		.i_hdmi_pixel_rst_n(hdmi_pixel_rst_n),
		.i_dehaze_algo_rgb(dehaze_algo_rgb),
		.i_dehaze_algo_tag(dehaze_algo_tag),
		.o_hud_wr_en(hud_wr_en),
		.o_hud_wr_addr(hud_wr_addr),
		.o_hud_wr_data(hud_wr_data),
		.o_hud_wr_strb(hud_wr_strb),
		.o_hud_rd_addr(hud_rd_addr),
		.i_hud_rd_window(hud_rd_window),
		.o_pcie_pclk_div2(pcie_pclk_div2),
		.o_pcie_core_rst_n(pcie_core_rst_n),
		.o_pcie_link_up(pcie_link_up),
		.i_button_rst_n(i_button_rst_n),
		.i_pcie_ref_clk_p(i_pcie_ref_clk_p),
		.i_pcie_ref_clk_n(i_pcie_ref_clk_n),
		.i_pcie_perst_n(i_pcie_perst_n),
		.i_pcie_rxp(i_pcie_rxp),
		.i_pcie_rxn(i_pcie_rxn),
		.i_action_clk(rgmii_clk),
		.i_action_valid(behavior_action_valid_net),
		.i_action_sequence(behavior_action_sequence_net),
		.i_action_type(behavior_action_type_net),
		.i_action_request(behavior_action_request_net),
		.o_action_ready(behavior_action_ready_net),
		.o_action_online(behavior_action_online_net),
		.o_pcie_txp(o_pcie_txp),
		.o_pcie_txn(o_pcie_txn),
		.o_behavior_result(behavior_result),
		.o_behavior_effective(behavior_effective)
	);

	hdmi_board_top #(
		.C_PHY_ADDR(C_PHY_ADDR),
		.C_USE_QUAD_OUTPUT(C_USE_QUAD_OUTPUT)
	) u_hdmi_board_top(
		.i_algo_pixel_rst_n(algo_pixel_rst_n),
		.o_hdmi_pixel_clk(hdmi_pixel_clk),
		.o_hdmi_pll_lock(hdmi_pll_lock),
		.o_hdmi_pixel_rst_n(hdmi_pixel_rst_n),
		.i_hdmi_pclk(hdmi_pclk),
		.i_quad_h1_pclk(quad_h1_pclk),
		.i_quad_h1_vs(quad_h1_vs),
		.i_quad_h1_hs(quad_h1_hs),
		.i_quad_h1_de(quad_h1_de),
		.i_quad_h1_rgb(quad_h1_rgb),
		.o_quad_base_vs(quad_base_vs),
		.o_quad_base_hs(quad_base_hs),
		.o_quad_base_de(quad_base_de),
		.o_quad_base_rgb(quad_base_rgb),
		.i_algo_output_tag(algo_output_tag),
		.i_hdmi_algo_vs(hdmi_algo_vs),
		.i_night_level(night_level),
		.i_dehaze_hud_enable(dehaze_hud_enable),
		.o_hud_state(hud_state),
		.i_hud_wr_en(hud_wr_en),
		.i_hud_wr_addr(hud_wr_addr),
		.i_hud_wr_data(hud_wr_data),
		.i_hud_wr_strb(hud_wr_strb),
		.i_hud_rd_addr(hud_rd_addr),
		.o_hud_rd_window(hud_rd_window),
		.i_fiber_pll_ready(fiber_pll_ready),
		.i_fiber_lane_snapshot(fiber_lane_snapshot),
		.i_fiber_lane_valid(fiber_lane_valid),
		.i_fiber_lane_stale(fiber_lane_stale),
		.i_pcie_pclk_div2(pcie_pclk_div2),
		.i_pcie_core_rst_n(pcie_core_rst_n),
		.i_pcie_link_up(pcie_link_up),
		.i_display_snapshot(behavior_display_snapshot),
		.o_scene_commit_valid(scene_commit_valid),
		.o_scene_commit_payload(scene_commit_payload),
		.o_control_pcie_rstn(behavior_control_rst_n),
		.i_c5_ddr_clk(c5_ddr_clk),
		.i_c5_underflow_event(c5_underflow_event),
		.i_quad_frame_aborted(quad_frame_aborted),
		.i_quad_ddr_rst_n(quad_ddr_rst_n),
		.i_monitor_composite_tag(monitor_composite_tag),
		.i_monitor_composite_rgb(monitor_composite_rgb),
		.i_monitor_composite_hud(monitor_composite_hud),
		.i_sys_clk(i_sys_clk),
		.i_button_rst_n(i_button_rst_n),
		.o_phy_reset_n(o_phy_reset_n),
		.o_mdc(o_mdc),
		.io_mdio(io_mdio),
		.o_hdmi_pclk(o_hdmi_pclk),
		.o_hdmi_vs(o_hdmi_vs),
		.o_hdmi_hs(o_hdmi_hs),
		.o_hdmi_de(o_hdmi_de),
		.o_hdmi_rgb(o_hdmi_rgb),
		.o_hdmi_scl(o_hdmi_scl),
		.io_hdmi_sda(io_hdmi_sda),
		.i_sfp0_los(i_sfp0_los),
		.o_sfp0_tx_disable(o_sfp0_tx_disable),
		.i_pcie_perst_n(i_pcie_perst_n),
		.o_status_led(o_status_led)
	);

endmodule
