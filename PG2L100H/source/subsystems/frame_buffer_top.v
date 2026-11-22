//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: DDR 帧缓冲与四宫格子系统，连接四源授权/帧事件/256-bit FIFO，维护每源三槽所有权并执行 DDR 写入、读出与像素跨域。
// Module Name: frame_buffer_top
// File Name: source/subsystems/frame_buffer_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// DDR 帧缓冲与四宫格子系统，连接四源授权/帧事件/256-bit FIFO，维护每源三槽所有权并执行 DDR 写入、读出与像素跨域。
// 参数可选择 Q8 四源路径或旧 C7 单源回归路径；Q8 读链形成公共四宫格流并保留内容标签，缺源由读链补黑。
// 封装唯一 ddr3_subsystem，向 monitor/AVM 暴露共享 HMIC 读写口；当前顶层 Q8 写模式为单拍兼容模式。
//
// Dependencies: quad_source_ddr_write_ingress、quad_mosaic_display_pipeline、c6_event_sync、c7_slot_grant_sync、旧 C7 调度/帧槽/FIFO 模块、video_window、quad_video_content_tag、ddr3_subsystem。
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
// Module Name: frame_buffer_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Structural extraction; original clocks, handshakes and latency retained.
// Dependencies: Original project modules instantiated below.
// Revision: 0.01 - Extracted from top_design.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module frame_buffer_top #(
	parameter C_CMOS1_ENABLE = 1,
	parameter C_FIBER_SOURCE2_ENABLE = 1,
	parameter C_HDMI_SOURCE3_ENABLE = 1,
	parameter integer C_Q8_MULTI_SOURCE = 1,
	parameter integer C_Q8_WRITE_BURST_BEATS = 1,
	parameter integer C_QUAD_STREAM_ALL = 1,
	parameter integer C_USE_Q8_READER = 1,
	parameter integer C_USE_Q8_WRITER = 1
)(
	input i_rgmii_clk,
	input i_hdmi_pixel_clk,
	input i_hdmi_pixel_rst_n,
	output o_hdmi_pclk,
	input i_hdmi_source3_word_empty,
	input [255:0] i_hdmi_source3_word_data,
	input [15:0] i_hdmi_source3_rd_level,
	input i_hdmi_source3_prepare_ddr,
	input i_hdmi_source3_complete_ddr,
	input i_hdmi_source3_abort_ddr,
	input i_cmos1_word_empty,
	input [255:0] i_cmos1_word_data,
	input [15:0] i_cmos1_rd_level,
	input i_cmos1_prepare_ddr,
	input i_cmos1_complete_ddr,
	input i_cmos1_abort_ddr,
	output o_quad_h1_pclk,
	output o_quad_h1_vs,
	output o_quad_h1_hs,
	output o_quad_h1_de,
	output [23:0] o_quad_h1_rgb,
	output o_quad_content_valid,
	output [11:0] o_quad_display_x,
	output [11:0] o_quad_display_y,
	output [1:0] o_quad_display_tile,
	output [9:0] o_quad_tile_x,
	output [8:0] o_quad_tile_y,
	output [3:0] o_quad_source_present,
	output o_quad_source_present_valid,
	input [255:0] i_fiber_source2_word_data,
	input i_fiber_source2_word_empty,
	input [15:0] i_fiber_source2_rd_level,
	input i_fiber_source2_prepare_ddr,
	input i_fiber_source2_complete_ddr,
	input i_fiber_source2_abort_ddr,
	output o_c5_ddr_clk,
	output o_c5_ddr_init_done,
	input i_c6_frame_prepare,
	input i_c6_frame_commit,
	input i_c6_frame_abort,
	output o_c7_frame_grant,
	output [1:0] o_c7_frame_grant_slot,
	output o_c7_frame_written,
	output o_c7_frame_failed,
	output o_c7_display_valid,
	input [255:0] i_c5_input_word,
	input i_c5_input_empty,
	output o_c5_input_pop,
	input [27:0] i_c5_awaddr,
	input [3:0] i_c5_awlen,
	output o_c5_awready,
	input i_c5_awvalid,
	input [255:0] i_c5_wdata,
	input [31:0] i_c5_wstrb,
	output o_c5_wready,
	input [27:0] i_c5_araddr,
	input [3:0] i_c5_arlen,
	output o_c5_arready,
	input i_c5_arvalid,
	output [255:0] o_c5_rdata,
	output o_c5_rvalid,
	input [15:0] i_c6_net_rd_water_level,
	output o_c5_underflow_event,
	output [15:0] o_c7_hdmi_rd_water_level,
	output [3:0] o_quad_grant_toggle,
	output [3:0] o_quad_word_pop,
	output [7:0] o_quad_display_slot,
	output [3:0] o_quad_display_valid,
	output [3:0] o_quad_retire_done,
	output o_quad_snapshot,
	output o_quad_read_urgent,
	output [3:0] o_quad_frame_aborted,
	output o_quad_ddr_rst_n,
	output [27:0] o_avm_quad_awaddr,
	output [3:0] o_avm_quad_awlen,
	output o_avm_quad_awvalid,
	output [255:0] o_avm_quad_wdata,
	output [31:0] o_avm_quad_wstrb,
	input [2:0] i_avm_write_client_awready,
	input [2:0] i_avm_write_client_wready,
	output [27:0] o_monitor_quad_araddr,
	output [3:0] o_monitor_quad_arlen,
	output o_monitor_quad_arvalid,
	input i_monitor_quad_arready,
	input i_monitor_quad_rvalid,
	input [11:0] i_monitor_slot_held,
	output [63:0] o_monitor_display_epoch,
	input i_button_rst_n,
	input i_ref_clk_p,
	input i_ref_clk_n,
	output o_mem_cs_n,
	output o_mem_rst_n,
	output o_mem_ck,
	output o_mem_ck_n,
	output o_mem_cke,
	output o_mem_ras_n,
	output o_mem_cas_n,
	output o_mem_we_n,
	output o_mem_odt,
	output [14:0] o_mem_a,
	output [2:0] o_mem_ba,
	inout [3:0] mem_dqs,
	inout [3:0] mem_dqs_n,
	inout [31:0] mem_dq,
	output [3:0] o_mem_dm
);
	wire rgmii_clk;
	wire hdmi_pixel_clk;
	wire hdmi_pixel_rst_n;
	wire hdmi_pclk;
	wire hdmi_vs;
	wire hdmi_hs;
	wire hdmi_de;
	wire [23:0] hdmi_rgb;
	wire hdmi_source3_word_empty;
	wire [255:0] hdmi_source3_word_data;
	wire [15:0] hdmi_source3_rd_level;
	wire hdmi_source3_prepare_ddr;
	wire hdmi_source3_complete_ddr;
	wire hdmi_source3_abort_ddr;
	wire cmos1_word_empty;
	wire [255:0] cmos1_word_data;
	wire [15:0] cmos1_rd_level;
	wire cmos1_prepare_ddr;
	wire cmos1_complete_ddr;
	wire cmos1_abort_ddr;
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
	wire quad_pixel_pop;
	wire quad_stream_pop;
	wire quad_frame_boundary;
	wire legacy_pixel_pop;
	wire legacy_frame_boundary;
	wire quad_stream_mode;
	wire [255:0] fiber_source2_word_data;
	wire fiber_source2_word_empty;
	wire [15:0] fiber_source2_rd_level;
	wire fiber_source2_prepare_ddr;
	wire fiber_source2_complete_ddr;
	wire fiber_source2_abort_ddr;
	wire c5_ddr_clk;
	wire c5_ddr_init_done;
	wire c5_ddr_init_fail;
	wire c6_frame_prepare;
	wire c6_frame_commit;
	wire c6_frame_abort;
	wire c6_frame_abort_ddr;
	wire c6_frame_commit_ddr;
	wire c7_frame_prepare_ddr;
	wire c7_frame_grant;
	wire [1:0] c7_frame_grant_slot;
	wire [1:0] c7_grant_slot_ddr;
	wire c7_grant_toggle_ddr;
	wire c7_frame_written;
	wire c7_frame_failed;
	wire [1:0] c7_display_slot;
	wire c7_display_valid;
	wire c7_display_switch;
	wire c7_video_frame_boundary;
	wire c7_vblank_ddr;
	wire c7_read_switch_done;
	wire [31:0] c7_valid_frame_count;
	wire [31:0] c7_slot_dropped_frame_count;
	wire [31:0] c7_display_commit_count;
	wire [2:0] c7_slot0_state;
	wire [2:0] c7_slot1_state;
	wire [2:0] c7_slot2_state;
	wire [255:0] c5_input_word;
	wire c5_input_empty;
	wire c5_input_pop;
	wire [27:0] c5_awaddr;
	wire c5_awuser_ap;
	wire [3:0] c5_awuser_id;
	wire [3:0] c5_awlen;
	wire c5_awready;
	wire c5_awvalid;
	wire [255:0] c5_wdata;
	wire [31:0] c5_wstrb;
	wire c5_wready;
	wire [15:0] c5_words_written;
	wire [27:0] c5_araddr;
	wire c5_aruser_ap;
	wire [3:0] c5_aruser_id;
	wire [3:0] c5_arlen;
	wire c5_arready;
	wire c5_arvalid;
	wire [255:0] c5_rdata;
	wire c5_rvalid;
	wire c5_output_full;
	wire c5_output_push;
	wire [255:0] c5_output_data;
	wire [15:0] c5_pixel_rgb565;
	wire c5_pixel_valid;
	wire c5_pixel_pop;
	wire [31:0] c5_underflow_count;
	wire [15:0] c5_words_read;
	wire [15:0] c6_net_rd_water_level;
	wire c5_underflow_event;
	wire [15:0] c7_hdmi_wr_water_level;
	wire [15:0] c7_hdmi_rd_water_level;
	wire [3:0] quad_grant_toggle;
	wire [7:0] quad_grant_slot;
	wire [3:0] quad_word_pop;
	wire [7:0] quad_display_slot;
	wire [3:0] quad_display_valid;
	wire [3:0] quad_display_switch;
	wire [3:0] quad_source_fresh;
	wire [3:0] quad_retire_done;
	wire quad_snapshot;
	wire quad_read_urgent;
	wire [3:0] quad_frame_ready;
	wire [3:0] quad_frame_aborted;
	wire [127:0] quad_valid_frame_count;
	wire [127:0] quad_dropped_frame_count;
	wire [127:0] quad_display_commit_count;
	wire [35:0] quad_slot_state;
	wire [63:0] quad_written_words;
	reg [1:0] quad_ddr_reset_sync;
	wire quad_ddr_rst_n;
	wire [27:0] avm_quad_awaddr;
	wire [3:0] avm_quad_awlen;
	wire avm_quad_awvalid;
	wire [255:0] avm_quad_wdata;
	wire [31:0] avm_quad_wstrb;
	wire avm_quad_wvalid;
	wire [2:0] avm_write_client_awready;
	wire [2:0] avm_write_client_wready;
	wire [27:0] monitor_quad_araddr;
	wire [3:0] monitor_quad_arlen;
	wire monitor_quad_arvalid;
	wire monitor_quad_arready;
	wire monitor_quad_rvalid;
	wire [11:0] monitor_slot_held;
	wire [63:0] monitor_display_epoch;
	wire ref_clk_p;
	wire ref_clk_n;
	wire mem_cs_n;
	wire mem_rst_n;
	wire mem_ck;
	wire mem_ck_n;
	wire mem_cke;
	wire mem_ras_n;
	wire mem_cas_n;
	wire mem_we_n;
	wire mem_odt;
	wire [14:0] mem_a;
	wire [2:0] mem_ba;
	wire [3:0] mem_dm;

	assign rgmii_clk = i_rgmii_clk;
	assign hdmi_pixel_clk = i_hdmi_pixel_clk;
	assign hdmi_pixel_rst_n = i_hdmi_pixel_rst_n;
	assign o_hdmi_pclk = hdmi_pclk;
	assign hdmi_source3_word_empty = i_hdmi_source3_word_empty;
	assign hdmi_source3_word_data = i_hdmi_source3_word_data;
	assign hdmi_source3_rd_level = i_hdmi_source3_rd_level;
	assign hdmi_source3_prepare_ddr = i_hdmi_source3_prepare_ddr;
	assign hdmi_source3_complete_ddr = i_hdmi_source3_complete_ddr;
	assign hdmi_source3_abort_ddr = i_hdmi_source3_abort_ddr;
	assign cmos1_word_empty = i_cmos1_word_empty;
	assign cmos1_word_data = i_cmos1_word_data;
	assign cmos1_rd_level = i_cmos1_rd_level;
	assign cmos1_prepare_ddr = i_cmos1_prepare_ddr;
	assign cmos1_complete_ddr = i_cmos1_complete_ddr;
	assign cmos1_abort_ddr = i_cmos1_abort_ddr;
	assign o_quad_h1_pclk = quad_h1_pclk;
	assign o_quad_h1_vs = quad_h1_vs;
	assign o_quad_h1_hs = quad_h1_hs;
	assign o_quad_h1_de = quad_h1_de;
	assign o_quad_h1_rgb = quad_h1_rgb;
	assign o_quad_content_valid = quad_content_valid;
	assign o_quad_display_x = quad_display_x;
	assign o_quad_display_y = quad_display_y;
	assign o_quad_display_tile = quad_display_tile;
	assign o_quad_tile_x = quad_tile_x;
	assign o_quad_tile_y = quad_tile_y;
	assign o_quad_source_present = quad_source_present;
	assign o_quad_source_present_valid = quad_source_present_valid;
	assign fiber_source2_word_data = i_fiber_source2_word_data;
	assign fiber_source2_word_empty = i_fiber_source2_word_empty;
	assign fiber_source2_rd_level = i_fiber_source2_rd_level;
	assign fiber_source2_prepare_ddr = i_fiber_source2_prepare_ddr;
	assign fiber_source2_complete_ddr = i_fiber_source2_complete_ddr;
	assign fiber_source2_abort_ddr = i_fiber_source2_abort_ddr;
	assign o_c5_ddr_clk = c5_ddr_clk;
	assign o_c5_ddr_init_done = c5_ddr_init_done;
	assign c6_frame_prepare = i_c6_frame_prepare;
	assign c6_frame_commit = i_c6_frame_commit;
	assign c6_frame_abort = i_c6_frame_abort;
	assign o_c7_frame_grant = c7_frame_grant;
	assign o_c7_frame_grant_slot = c7_frame_grant_slot;
	assign o_c7_frame_written = c7_frame_written;
	assign o_c7_frame_failed = c7_frame_failed;
	assign o_c7_display_valid = c7_display_valid;
	assign c5_input_word = i_c5_input_word;
	assign c5_input_empty = i_c5_input_empty;
	assign o_c5_input_pop = c5_input_pop;
	assign c5_awaddr = i_c5_awaddr;
	assign c5_awlen = i_c5_awlen;
	assign o_c5_awready = c5_awready;
	assign c5_awvalid = i_c5_awvalid;
	assign c5_wdata = i_c5_wdata;
	assign c5_wstrb = i_c5_wstrb;
	assign o_c5_wready = c5_wready;
	assign c5_araddr = i_c5_araddr;
	assign c5_arlen = i_c5_arlen;
	assign o_c5_arready = c5_arready;
	assign c5_arvalid = i_c5_arvalid;
	assign o_c5_rdata = c5_rdata;
	assign o_c5_rvalid = c5_rvalid;
	assign c6_net_rd_water_level = i_c6_net_rd_water_level;
	assign o_c5_underflow_event = c5_underflow_event;
	assign o_c7_hdmi_rd_water_level = c7_hdmi_rd_water_level;
	assign o_quad_grant_toggle = quad_grant_toggle;
	assign o_quad_word_pop = quad_word_pop;
	assign o_quad_display_slot = quad_display_slot;
	assign o_quad_display_valid = quad_display_valid;
	assign o_quad_retire_done = quad_retire_done;
	assign o_quad_snapshot = quad_snapshot;
	assign o_quad_read_urgent = quad_read_urgent;
	assign o_quad_frame_aborted = quad_frame_aborted;
	assign o_quad_ddr_rst_n = quad_ddr_rst_n;
	assign o_avm_quad_awaddr = avm_quad_awaddr;
	assign o_avm_quad_awlen = avm_quad_awlen;
	assign o_avm_quad_awvalid = avm_quad_awvalid;
	assign o_avm_quad_wdata = avm_quad_wdata;
	assign o_avm_quad_wstrb = avm_quad_wstrb;
	assign avm_write_client_awready = i_avm_write_client_awready;
	assign avm_write_client_wready = i_avm_write_client_wready;
	assign o_monitor_quad_araddr = monitor_quad_araddr;
	assign o_monitor_quad_arlen = monitor_quad_arlen;
	assign o_monitor_quad_arvalid = monitor_quad_arvalid;
	assign monitor_quad_arready = i_monitor_quad_arready;
	assign monitor_quad_rvalid = i_monitor_quad_rvalid;
	assign monitor_slot_held = i_monitor_slot_held;
	assign o_monitor_display_epoch = monitor_display_epoch;
	assign ref_clk_p = i_ref_clk_p;
	assign ref_clk_n = i_ref_clk_n;
	assign o_mem_cs_n = mem_cs_n;
	assign o_mem_rst_n = mem_rst_n;
	assign o_mem_ck = mem_ck;
	assign o_mem_ck_n = mem_ck_n;
	assign o_mem_cke = mem_cke;
	assign o_mem_ras_n = mem_ras_n;
	assign o_mem_cas_n = mem_cas_n;
	assign o_mem_we_n = mem_we_n;
	assign o_mem_odt = mem_odt;
	assign o_mem_a = mem_a;
	assign o_mem_ba = mem_ba;
	assign o_mem_dm = mem_dm;

	always @(posedge c5_ddr_clk or negedge i_button_rst_n) begin
		if (!i_button_rst_n) begin
			quad_ddr_reset_sync <= 2'b00;
		end else begin
			quad_ddr_reset_sync <= {quad_ddr_reset_sync[0], 1'b1};
		end
	end
	assign quad_ddr_rst_n = quad_ddr_reset_sync[1];
	// Q8读链仍为算法/四源公共输入；它不应与最终HDMI输出选择耦合。
	assign c5_pixel_pop = C_USE_Q8_READER ? quad_stream_pop : legacy_pixel_pop;
	assign c7_video_frame_boundary = C_USE_Q8_READER ? quad_frame_boundary : legacy_frame_boundary;
	// 缺源由 reader 填零；整幅流始终消费，不能等待 Ethernet 首帧才 pop。
	assign quad_stream_mode = 1'b1;


	// commit/abort 以 toggle 跨到 DDR 域，避免将短脉冲直接打两拍而漏采样。
	c6_event_sync u_c6_abort_event_sync(
		.i_src_clk(rgmii_clk),
		.i_src_rst_n(i_button_rst_n),
		.i_src_event(c6_frame_abort),
		.i_dst_clk(c5_ddr_clk),
		.i_dst_rst_n(i_button_rst_n),
		.o_dst_event(c6_frame_abort_ddr)
	);

	// C7 帧槽 grant 是 DDR->网络的稳定描述符；prepare/commit/abort 是
	// 网络->DDR 的 toggle 事务。多位 slot id 从不逐位直接作为 CDC 数据使用。
	c7_slot_grant_sync u_c7_slot_grant_sync(
		.i_clk(rgmii_clk),
		.i_rst_n(i_button_rst_n),
		.i_grant_toggle(c7_grant_toggle_ddr),
		.i_grant_slot(c7_grant_slot_ddr),
		.o_grant(c7_frame_grant),
		.o_slot(c7_frame_grant_slot)
	);

	c6_event_sync u_c7_prepare_event_sync(
		.i_src_clk(rgmii_clk),
		.i_src_rst_n(i_button_rst_n),
		.i_src_event(c6_frame_prepare),
		.i_dst_clk(c5_ddr_clk),
		.i_dst_rst_n(i_button_rst_n),
		.o_dst_event(c7_frame_prepare_ddr)
	);

	c6_event_sync u_c6_commit_event_sync(
		.i_src_clk(rgmii_clk),
		.i_src_rst_n(i_button_rst_n),
		.i_src_event(c6_frame_commit),
		.i_dst_clk(c5_ddr_clk),
		.i_dst_rst_n(i_button_rst_n),
		.o_dst_event(c6_frame_commit_ddr)
	);

	// 帧边界只传一次 toggle，四源 ownership 和 reader 共用 DDR 域事件。
	c6_event_sync u_c7_vblank_event_sync(
		.i_src_clk(hdmi_pixel_clk), .i_src_rst_n(hdmi_pixel_rst_n),
		.i_src_event(c7_video_frame_boundary), .i_dst_clk(c5_ddr_clk),
		.i_dst_rst_n(i_button_rst_n), .o_dst_event(c7_vblank_ddr)
	);

	generate
	if (C_USE_Q8_WRITER != 0) begin: gen_q8_writer_path
		quad_source_ddr_write_ingress #(
			.C_BURST_BEATS(C_Q8_WRITE_BURST_BEATS), .C_EXTERNAL_LEASE_ENABLE(1)
		) u_write_ingress(
			.i_clk(c5_ddr_clk), .i_rst_n(quad_ddr_rst_n),
			.i_source_enable({((C_Q8_MULTI_SOURCE != 0) && (C_HDMI_SOURCE3_ENABLE != 0)),
				((C_Q8_MULTI_SOURCE != 0) && (C_FIBER_SOURCE2_ENABLE != 0)),
				((C_Q8_MULTI_SOURCE != 0) && (C_CMOS1_ENABLE != 0)), 1'b1}),
			.i_frame_started({(C_Q8_MULTI_SOURCE != 0) ? hdmi_source3_prepare_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? fiber_source2_prepare_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? cmos1_prepare_ddr : 1'b0, c7_frame_prepare_ddr}),
			.i_frame_end({(C_Q8_MULTI_SOURCE != 0) ? hdmi_source3_complete_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? fiber_source2_complete_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? cmos1_complete_ddr : 1'b0, c6_frame_commit_ddr}),
			.i_frame_error({(C_Q8_MULTI_SOURCE != 0) ? hdmi_source3_abort_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? fiber_source2_abort_ddr : 1'b0,
				(C_Q8_MULTI_SOURCE != 0) ? cmos1_abort_ddr : 1'b0, c6_frame_abort_ddr}),
			.i_hdmi_frame_boundary(C_USE_Q8_READER != 0 ? quad_snapshot : c7_vblank_ddr),
			.i_retire_done(quad_retire_done),
			.i_external_slot_held(monitor_slot_held), .o_display_epoch(monitor_display_epoch),
			.i_fifo_word_count({hdmi_source3_rd_level, fiber_source2_rd_level,
				cmos1_rd_level, c6_net_rd_water_level}),
			.i_fifo_empty({hdmi_source3_word_empty, fiber_source2_word_empty,
				cmos1_word_empty, c5_input_empty}),
			.i_fifo_data({hdmi_source3_word_data, fiber_source2_word_data,
				cmos1_word_data, c5_input_word}),
			.i_fifo_almost_full({(hdmi_source3_rd_level >= 16'd448),
				(fiber_source2_rd_level >= 16'd448),
				(cmos1_rd_level >= 16'd448), (c6_net_rd_water_level >= 16'd448)}),
			.i_read_urgent(quad_read_urgent), .i_engine_ready(c5_ddr_init_done),
			.i_axi_awready(avm_write_client_awready[0]), .i_axi_wready(avm_write_client_wready[0]),
			.o_word_pop(quad_word_pop), .o_axi_awaddr(avm_quad_awaddr),
			.o_axi_awlen(avm_quad_awlen), .o_axi_awvalid(avm_quad_awvalid),
			.o_axi_wdata(avm_quad_wdata), .o_axi_wvalid(avm_quad_wvalid), .o_burst_done(), .o_burst_failed(),
			.o_busy(), .o_active_source(), .o_grant_valid(), .o_grant_toggle(quad_grant_toggle),
			.o_grant_slot(quad_grant_slot), .o_frame_ready(quad_frame_ready),
			.o_frame_aborted(quad_frame_aborted), .o_display_slot(quad_display_slot),
			.o_display_valid(quad_display_valid), .o_display_switch(quad_display_switch),
			.o_valid_frame_count(quad_valid_frame_count), .o_dropped_frame_count(quad_dropped_frame_count),
			.o_display_commit_count(quad_display_commit_count), .o_slot_state(quad_slot_state),
			.o_written_words(quad_written_words)
		);
		assign c5_input_pop = quad_word_pop[0];
		assign c5_awuser_ap = 1'b0;
		assign c5_awuser_id = 4'd0;
		assign avm_quad_wstrb = 32'hffffffff;
		assign c5_aruser_ap = 1'b0;
		assign c5_aruser_id = 4'd0;
		assign c7_grant_toggle_ddr = quad_grant_toggle[0];
		assign c7_grant_slot_ddr = quad_grant_slot[1:0];
		assign c7_frame_written = quad_frame_ready[0];
		assign c7_frame_failed = quad_frame_aborted[0];
		assign c7_display_slot = quad_display_slot[1:0];
		assign c7_display_valid = quad_display_valid[0];
		assign c7_display_switch = quad_display_switch[0];
		assign c7_valid_frame_count = quad_valid_frame_count[31:0];
		assign c7_slot_dropped_frame_count = quad_dropped_frame_count[31:0];
		assign c7_display_commit_count = quad_display_commit_count[31:0];
		assign c7_slot0_state = quad_slot_state[2:0];
		assign c7_slot1_state = quad_slot_state[5:3];
		assign c7_slot2_state = quad_slot_state[8:6];
		assign c5_words_written = quad_written_words[15:0];
	end else begin: gen_c7_writer_path
		assign quad_grant_toggle = 4'd0;
		assign quad_word_pop = 4'd0;
		c7_ddr_write_scheduler u_c7_ddr_write_scheduler(
			.i_clk(c5_ddr_clk), .i_rst_n(i_button_rst_n), .i_ddr_init_done(c5_ddr_init_done),
			.i_frame_prepare(c7_frame_prepare_ddr), .i_write_slot(c7_grant_slot_ddr),
			.i_frame_commit(c6_frame_commit_ddr), .i_frame_abort(c6_frame_abort_ddr),
			.i_word_data(c5_input_word), .i_word_empty(c5_input_empty), .o_word_pop(c5_input_pop),
			.o_axi_awaddr(avm_quad_awaddr), .o_axi_awuser_ap(c5_awuser_ap), .o_axi_awuser_id(c5_awuser_id),
			.o_axi_awlen(avm_quad_awlen), .i_axi_awready(avm_write_client_awready[0]), .o_axi_awvalid(avm_quad_awvalid),
			.o_axi_wdata(avm_quad_wdata), .o_axi_wstrb(avm_quad_wstrb), .i_axi_wready(avm_write_client_wready[0]),
			.o_frame_written(c7_frame_written), .o_frame_failed(c7_frame_failed),
			.o_words_written(c5_words_written)
		);
		// 旧single-beat scheduler在AW接受后始终提供稳定数据，无独立WVALID。
		assign avm_quad_wvalid = 1'b1;
		c7_frame_slot_manager u_c7_frame_slot_manager(
			.i_clk(c5_ddr_clk), .i_rst_n(i_button_rst_n), .i_frame_prepare(c7_frame_prepare_ddr),
			.i_frame_written(c7_frame_written), .i_frame_failed(c7_frame_failed),
			.i_vblank(c7_vblank_ddr),
			.i_retire_done(C_USE_Q8_READER != 0 ? quad_retire_done[0] : c7_read_switch_done),
			.o_grant_slot(c7_grant_slot_ddr), .o_grant_toggle(c7_grant_toggle_ddr),
			.o_display_slot(c7_display_slot), .o_display_valid(c7_display_valid),
			.o_display_switch(c7_display_switch), .o_valid_frame_count(c7_valid_frame_count),
			.o_dropped_frame_count(c7_slot_dropped_frame_count),
			.o_display_commit_count(c7_display_commit_count), .o_slot0_state(c7_slot0_state),
			.o_slot1_state(c7_slot1_state), .o_slot2_state(c7_slot2_state)
		);
	end
	endgenerate

	generate
	if (C_USE_Q8_READER != 0) begin: gen_q8_reader_path
		// 完整新帧被提升到显示槽才续期；旧帧重放不代表来源仍在线。
		quad_source_freshness_ctrl quad_source_freshness_ctrl_Inst_statistics(
			.i_clk(c5_ddr_clk),
			.i_rstn(quad_ddr_rst_n),
			.i_frame_boundary(c7_vblank_ddr),
			.i_display_switch(C_USE_Q8_WRITER != 0 ? quad_display_switch : {3'b000, c7_display_switch}),
			.o_source_fresh(quad_source_fresh)
		);
		quad_mosaic_display_pipeline u_display_pipeline(
			.i_ddr_clk(c5_ddr_clk), .i_ddr_rst_n(quad_ddr_rst_n), .i_ddr_init_done(c5_ddr_init_done),
			.i_frame_boundary(c7_vblank_ddr),
			.i_display_slot(C_USE_Q8_WRITER != 0 ? quad_display_slot : {6'd0, c7_display_slot}),
			.i_display_valid(C_USE_Q8_WRITER != 0 ? quad_display_valid : {3'b000, c7_display_valid}),
			.i_source_fresh(quad_source_fresh),
			.o_snapshot(quad_snapshot), .o_retire_done(quad_retire_done),
			.o_axi_araddr(monitor_quad_araddr), .o_axi_arlen(monitor_quad_arlen), .o_axi_arvalid(monitor_quad_arvalid),
			.i_axi_arready(monitor_quad_arready), .i_axi_rdata(c5_rdata), .i_axi_rvalid(monitor_quad_rvalid),
			.i_pixel_clk(hdmi_pixel_clk), .i_pixel_rst_n(hdmi_pixel_rst_n), .i_pixel_pop(c5_pixel_pop),
			.i_pixel_frame_boundary(quad_frame_boundary),
			.o_pixel_source_present(quad_source_present), .o_pixel_source_present_valid(quad_source_present_valid),
			.o_pixel_rgb565(c5_pixel_rgb565), .o_pixel_valid(c5_pixel_valid),
			.o_underflow_count(c5_underflow_count), .o_underflow_event(c5_underflow_event),
			.o_wr_water_level(c7_hdmi_wr_water_level), .o_rd_water_level(c7_hdmi_rd_water_level),
			.o_read_urgent(quad_read_urgent)
		);
		assign c5_words_read = 16'd0;
	end else begin: gen_c7_reader_path
		// 旧C7仅网络tile为真实内容；valid由原显示状态决定。
		assign quad_source_present = {3'b000,c5_pixel_valid};
		assign quad_source_present_valid = 1'b1;
		assign quad_retire_done = {3'b000, c7_read_switch_done};
		c7_ddr_read_scheduler u_c7_ddr_read_scheduler(
			.i_clk(c5_ddr_clk), .i_rst_n(i_button_rst_n),
			.i_display_valid(C_USE_Q8_WRITER != 0 ? quad_display_valid[0] : c7_display_valid),
			.i_display_slot(C_USE_Q8_WRITER != 0 ? quad_display_slot[1:0] : c7_display_slot),
			.i_display_switch(C_USE_Q8_WRITER != 0 ? quad_display_switch[0] : c7_display_switch),
			.i_output_full(c5_output_full), .o_output_push(c5_output_push),
			.o_output_data(c5_output_data), .o_axi_araddr(monitor_quad_araddr),
			.o_axi_aruser_ap(c5_aruser_ap), .o_axi_aruser_id(c5_aruser_id), .o_axi_arlen(monitor_quad_arlen),
			.i_axi_arready(monitor_quad_arready), .o_axi_arvalid(monitor_quad_arvalid), .i_axi_rdata(c5_rdata),
			.i_axi_rvalid(monitor_quad_rvalid), .o_switch_done(c7_read_switch_done), .o_words_read(c5_words_read)
		);
		c7_ddr_to_hdmi_fifo_wrapper u_c7_ddr_to_hdmi_fifo_wrapper(
			.i_ddr_clk(c5_ddr_clk), .i_ddr_rst_n(i_button_rst_n),
			.i_frame_switch(C_USE_Q8_WRITER != 0 ? quad_display_switch[0] : c7_display_switch),
			.i_word_push(c5_output_push), .i_word_data(c5_output_data), .o_word_full(c5_output_full),
			.i_pixel_clk(hdmi_pixel_clk), .i_pixel_rst_n(hdmi_pixel_rst_n),
			.i_measure_enable(C_USE_Q8_WRITER != 0 ? quad_display_valid[0] : c7_display_valid),
			.i_pixel_pop(c5_pixel_pop), .o_pixel_rgb565(c5_pixel_rgb565), .o_pixel_valid(c5_pixel_valid),
			.o_underflow_count(c5_underflow_count), .o_underflow_event(c5_underflow_event),
			.o_wr_water_level(c7_hdmi_wr_water_level), .o_rd_water_level(c7_hdmi_rd_water_level)
		);
	end
	endgenerate


	video_window u_video_window(
		.i_pixel_clk(hdmi_pixel_clk),
		.i_rst_n(hdmi_pixel_rst_n),
		.i_frame_ready(c7_display_valid),
		.i_pixel_valid(c5_pixel_valid),
		.i_pixel_rgb565(c5_pixel_rgb565),
		.o_pixel_pop(legacy_pixel_pop),
		.o_frame_boundary(legacy_frame_boundary),
		.o_pclk(hdmi_pclk),
		.o_vs(hdmi_vs),
		.o_hs(hdmi_hs),
		.o_de(hdmi_de),
		.o_rgb(hdmi_rgb)
	);

	quad_video_content_tag #(.C_STREAM_ALL_TILES(C_QUAD_STREAM_ALL), .C_STREAM_TILE_ID(0)) u_quad_h1_pattern(
		.i_pixel_clk(hdmi_pixel_clk),
		.i_rst_n(hdmi_pixel_rst_n),
		.o_pclk(quad_h1_pclk),
		.o_vs(quad_h1_vs),
		.o_hs(quad_h1_hs),
		.o_de(quad_h1_de),
		.o_rgb(quad_h1_rgb),
		.o_pixel_pop(quad_pixel_pop),
		.o_stream_pop(quad_stream_pop),
		.o_frame_boundary(quad_frame_boundary),
		.o_x(),
		.o_y(),
		.o_tile_id(),
		.i_stream_mode(quad_stream_mode),
		.i_stream_rgb565(c5_pixel_rgb565),
		.i_stream_valid(c5_pixel_valid),
		.i_source_present(quad_source_present),
		.i_source_present_valid(quad_source_present_valid),
		.o_content_valid(quad_content_valid),
		.o_display_x(quad_display_x), .o_display_y(quad_display_y),
		.o_display_tile_id(quad_display_tile), .o_tile_x(quad_tile_x), .o_tile_y(quad_tile_y)
	);


	ddr3_subsystem u_ddr3_subsystem(
		.ref_clk_p(ref_clk_p),
		.ref_clk_n(ref_clk_n),
		.i_rst_n(i_button_rst_n),
		.o_core_clk(c5_ddr_clk),
		.o_init_done(c5_ddr_init_done),
		.o_init_fail(c5_ddr_init_fail),
		.i_axi_awaddr(c5_awaddr),
		.i_axi_awuser_ap(c5_awuser_ap),
		.i_axi_awuser_id(c5_awuser_id),
		.i_axi_awlen(c5_awlen),
		.o_axi_awready(c5_awready),
		.i_axi_awvalid(c5_awvalid),
		.i_axi_wdata(c5_wdata),
		.i_axi_wstrb(c5_wstrb),
		.o_axi_wready(c5_wready),
		.i_axi_araddr(c5_araddr),
		.i_axi_aruser_ap(c5_aruser_ap),
		.i_axi_aruser_id(c5_aruser_id),
		.i_axi_arlen(c5_arlen),
		.o_axi_arready(c5_arready),
		.i_axi_arvalid(c5_arvalid),
		.o_axi_rdata(c5_rdata),
		.o_axi_rvalid(c5_rvalid),
		.mem_cs_n(mem_cs_n),
		.mem_rst_n(mem_rst_n),
		.mem_ck(mem_ck),
		.mem_ck_n(mem_ck_n),
		.mem_cke(mem_cke),
		.mem_ras_n(mem_ras_n),
		.mem_cas_n(mem_cas_n),
		.mem_we_n(mem_we_n),
		.mem_odt(mem_odt),
		.mem_a(mem_a),
		.mem_ba(mem_ba),
		.mem_dqs(mem_dqs),
		.mem_dqs_n(mem_dqs_n),
		.mem_dq(mem_dq),
		.mem_dm(mem_dm)
	);


endmodule
