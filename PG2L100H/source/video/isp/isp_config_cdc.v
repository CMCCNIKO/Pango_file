//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 通过稳定 520 位载荷和 req/ack toggle 将 ISP 设置跨到像素域，一次只允许一项在途请求。
// Module Name: isp_config_cdc
// File Name: source/video/isp/isp_config_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 通过稳定 520 位载荷和 req/ack toggle 将 ISP 设置跨到像素域，一次只允许一项在途请求。
// 目的域先捕获再在后续帧边界应用；分离原始配置 epoch 与各 tile 有效 epoch，避免禁用等级变化误清自动大气光统计。
//
// Dependencies: 调用方协调复位、各域同步释放及像素帧边界。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-16 / v025 - 修复有效tile位号、跨帧关闭恢复；增加独立稳定载荷遥测握手。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: ISP configuration CDC
// Module Name: isp_config_cdc
// Project Name: PG2L100H_AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 520-bit stable payload request/acknowledge CDC for ISP settings.
//              Source permits one request in flight; destination captures the
//              payload first and applies it only on the following frame boundary.
//              Raw configuration epoch and effective per-tile epochs are kept
//              separate so disabled levels do not invalidate auto-A statistics.
// Dependencies: Coordinated reset; source and destination reset release are
//                synchronized by the caller.
// Revision: 0.01 - 2026-10-19, initial implementation.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module isp_config_cdc
(
	input i_src_clk,
	input i_src_rst_n,
	input i_dst_clk,
	input i_dst_rst_n,
	input i_update,
	input [519:0] i_config,
	input i_frame_boundary,
	input i_adaptive_enable,
	output o_adaptive_pixel_enable,
	input [23:0] i_adaptive_rgb,
	input [159:0] i_adaptive_tag,
	input [23:0] i_scene_rgb,
	input [159:0] i_scene_tag,
	input i_scene_frame_boundary,
	input [15:0] i_scene_epoch,
	input [159:0] i_weather_actual,
	output o_summary_valid,
	output o_summary_complete,
	output [1:0] o_summary_tile,
	output [31:0] o_summary_frame,
	output [15:0] o_summary_epoch,
	output [7:0] o_summary_p10,
	output [7:0] o_summary_p50,
	output [7:0] o_summary_p90,
	output [13:0] o_summary_low48,
	output [13:0] o_summary_fog_count,
	output [13:0] o_summary_texture_count,
	output [13:0] o_summary_neutral_count,
	output o_busy,
	output o_applied,
	output o_pending,
	output [519:0] o_active_config,
	output [15:0] o_config_epoch,
	output [63:0] o_effective_epoch,
	output o_isp_continuous_update,
	output [3:0] o_change_mask,
	output [519:0] o_src_active_snapshot,
	output [15:0] o_src_config_epoch,
	output [519:0] o_src_live_config,
	output [15:0] o_src_live_epoch,
	output [63:0] o_src_live_status,
	output [15:0] o_src_live_id,
	output o_src_live_enable,
	output o_src_live_valid,
	output [159:0] o_src_live_weather_actual
);
	localparam [127:0] DEFAULT_TILE = {
		8'd0, 10'd256, 10'd256, 10'd256,
		8'd0, 8'd0, 8'd0, 2'd1, 3'd1, 8'd12,
		8'd32, 3'd1, 8'd4, 8'd16, 3'd0, 3'd0, 20'd0
	};
	// The global byte occupies [519:512]; tile0 is the low 128-bit slice.
	// Keep the concatenation order explicit so reset state matches isp_control's
	// absolute configuration encoding ({global,tile3,tile2,tile1,tile0}).
	localparam [519:0] DEFAULT_CONFIG = {
		8'h1e, DEFAULT_TILE, DEFAULT_TILE, DEFAULT_TILE, DEFAULT_TILE
	};

	reg [519:0] reg_payload;
	reg flag_request;
	reg flag_ack;
	(* syn_preserve = 1 *) reg [1:0] reg_ack_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_request_sync;
	reg reg_ack_previous;
	reg applied_o;
	reg pending_o;
	reg [519:0] reg_pending_payload;
	reg [519:0] active_config_o;
	reg [15:0] config_epoch_o;
	reg [63:0] effective_epoch_o;
	reg [3:0] change_mask_o;
	(* async_reg = "true" *) reg [1:0] adaptive_enable_sync;
	reg adaptive_enable_sync_d;
	reg reg_adaptive_boundary_enable;
	reg isp_continuous_update_o;
	reg flag_restore_pending;
	wire [519:0] adaptive_next_config;
	wire [519:0] adaptive_restore_config;
	wire [63:0] adaptive_status;
	// 遥测使用独立稳定载荷握手，不产生配置 applied 事件。
	// v026高160bit扩展真实天气处理快照，原0..616位布局保持不变。
	reg [776:0] reg_live_payload;
	reg flag_live_request;
	reg flag_live_ack;
	reg flag_live_capture;
	reg [15:0] reg_live_id;
	(* async_reg = "true" *) reg [1:0] reg_live_request_sync;
	(* async_reg = "true" *) reg [1:0] reg_live_ack_sync;
	reg [776:0] src_live_payload_o;
	reg src_live_valid_o;

	// Target-to-source bundled return. These buses remain stable between ACKs.
	reg [519:0] reg_ack_active_config;
	reg [15:0] reg_ack_config_epoch;
	reg [519:0] src_active_snapshot_o;
	reg [15:0] src_config_epoch_o;

	// Canonicalize a tile for effective-version tracking.  A parameter that is
	// behind a disabled stage is retained in active_config_o for later use, but
	// it must not invalidate the dehaze/A statistics until that stage is enabled.
	// The function keeps the six stage-enable bits and only the fields consumed
	// by an enabled stage; reserved bits are excluded from the effective image.
	function [127:0] effective_tile;
		input [127:0] tile_value;
		reg [127:0] canonical;
		begin
			canonical = 128'd0;
			canonical[125:120] = tile_value[125:120];
			if(tile_value[120])begin
				canonical[119:110] = tile_value[119:110];
				canonical[109:100] = tile_value[109:100];
				canonical[99:90] = tile_value[99:90];
				canonical[89:82] = tile_value[89:82];
				canonical[81:74] = tile_value[81:74];
				canonical[73:66] = tile_value[73:66];
			end
			if(tile_value[121])begin
				canonical[65:64] = tile_value[65:64];
				canonical[63:61] = tile_value[63:61];
				canonical[60:53] = tile_value[60:53];
			end
			if(tile_value[122])begin
				canonical[52:45] = tile_value[52:45];
			end
			if(tile_value[123])begin
				canonical[44:42] = tile_value[44:42];
				canonical[41:34] = tile_value[41:34];
				canonical[33:26] = tile_value[33:26];
			end
			if(tile_value[124])begin
				canonical[25:23] = tile_value[25:23];
				canonical[126] = tile_value[126];
			end
			if(tile_value[125])begin
				canonical[22:20] = tile_value[22:20];
			end
			effective_tile = canonical;
		end
	endfunction

	wire [7:0] pending_global = reg_pending_payload[519:512];
	wire [7:0] active_global = active_config_o[519:512];
	wire pending_master = pending_global[0];
	wire active_master = active_global[0];
	wire [3:0] pending_tile_enable = pending_global[4:1];
	wire [3:0] active_tile_enable = active_global[4:1];
	wire [127:0] pending_tile0 = reg_pending_payload[127:0];
	wire [127:0] pending_tile1 = reg_pending_payload[255:128];
	wire [127:0] pending_tile2 = reg_pending_payload[383:256];
	wire [127:0] pending_tile3 = reg_pending_payload[511:384];
	wire [127:0] active_tile0 = active_config_o[127:0];
	wire [127:0] active_tile1 = active_config_o[255:128];
	wire [127:0] active_tile2 = active_config_o[383:256];
	wire [127:0] active_tile3 = active_config_o[511:384];
	wire [127:0] pending_effective_tile0 = (pending_master && pending_tile_enable[0]) ? effective_tile(pending_tile0) : 128'd0;
	wire [127:0] pending_effective_tile1 = (pending_master && pending_tile_enable[1]) ? effective_tile(pending_tile1) : 128'd0;
	wire [127:0] pending_effective_tile2 = (pending_master && pending_tile_enable[2]) ? effective_tile(pending_tile2) : 128'd0;
	wire [127:0] pending_effective_tile3 = (pending_master && pending_tile_enable[3]) ? effective_tile(pending_tile3) : 128'd0;
	wire [127:0] active_effective_tile0 = (active_master && active_tile_enable[0]) ? effective_tile(active_tile0) : 128'd0;
	wire [127:0] active_effective_tile1 = (active_master && active_tile_enable[1]) ? effective_tile(active_tile1) : 128'd0;
	wire [127:0] active_effective_tile2 = (active_master && active_tile_enable[2]) ? effective_tile(active_tile2) : 128'd0;
	wire [127:0] active_effective_tile3 = (active_master && active_tile_enable[3]) ? effective_tile(active_tile3) : 128'd0;
	wire [519:0] frame_apply_config = pending_o ? reg_pending_payload :
		(adaptive_enable_sync[1] ? adaptive_next_config : adaptive_restore_config);
	wire [127:0] frame_apply_effective_tile0 =
		(frame_apply_config[512] && frame_apply_config[513]) ? effective_tile(frame_apply_config[127:0]) : 128'd0;
	wire [127:0] frame_apply_effective_tile1 =
		(frame_apply_config[512] && frame_apply_config[514]) ? effective_tile(frame_apply_config[255:128]) : 128'd0;
	wire [127:0] frame_apply_effective_tile2 =
		(frame_apply_config[512] && frame_apply_config[515]) ? effective_tile(frame_apply_config[383:256]) : 128'd0;
	wire [127:0] frame_apply_effective_tile3 =
		(frame_apply_config[512] && frame_apply_config[516]) ? effective_tile(frame_apply_config[511:384]) : 128'd0;

	assign o_busy = (flag_request != reg_ack_sync[1]) ||
		(reg_ack_sync[1] != reg_ack_previous) || applied_o;
	assign o_applied = applied_o;
	assign o_pending = pending_o;
	assign o_active_config = active_config_o;
	assign o_config_epoch = config_epoch_o;
	assign o_effective_epoch = effective_epoch_o;
	assign o_isp_continuous_update = isp_continuous_update_o;
	assign o_change_mask = change_mask_o;
	assign o_src_active_snapshot = src_active_snapshot_o;
	assign o_src_config_epoch = src_config_epoch_o;
	assign o_src_live_config = src_live_payload_o[519:0];
	assign o_src_live_epoch = src_live_payload_o[535:520];
	assign o_src_live_status = src_live_payload_o[599:536];
	assign o_src_live_enable = src_live_payload_o[600];
	assign o_src_live_id = src_live_payload_o[616:601];
	assign o_src_live_valid = src_live_valid_o;
	assign o_src_live_weather_actual = src_live_payload_o[776:617];
	assign o_adaptive_pixel_enable = adaptive_enable_sync[1];
	isp_adaptive_pixel_ctrl u_adaptive_pixel_ctrl(
		.i_clk(i_dst_clk),.i_rst_n(i_dst_rst_n),
		.i_enable(adaptive_enable_sync[1]),.i_frame_boundary(i_frame_boundary),
		.i_rgb(i_adaptive_rgb),.i_tag(i_adaptive_tag),
		.i_scene_rgb(i_scene_rgb),.i_scene_tag(i_scene_tag),
		.i_scene_frame_boundary(i_scene_frame_boundary),.i_scene_epoch(i_scene_epoch),
		.i_active_config(active_config_o),
		.o_next_config(adaptive_next_config),.o_restore_config(adaptive_restore_config),
		.o_status(adaptive_status),
		// 摘要始终为像素域组合转接，供外部天气控制复用，不增加CDC或拍数。
		.o_summary_valid(o_summary_valid),.o_summary_complete(o_summary_complete),
		.o_summary_tile(o_summary_tile),.o_summary_frame(o_summary_frame),
		.o_summary_epoch(o_summary_epoch),.o_summary_p10(o_summary_p10),
		.o_summary_p50(o_summary_p50),.o_summary_p90(o_summary_p90),
		.o_summary_low48(o_summary_low48),.o_summary_fog_count(o_summary_fog_count),
		.o_summary_texture_count(o_summary_texture_count),
		.o_summary_neutral_count(o_summary_neutral_count));

	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			adaptive_enable_sync <= 2'd0;
			adaptive_enable_sync_d <= 1'b0;
		end else begin
			adaptive_enable_sync <= {adaptive_enable_sync[0],i_adaptive_enable};
			adaptive_enable_sync_d <= adaptive_enable_sync[1];
		end
	end
	// v028: 保持整帧更新原因；首次开启、手动优先与关闭恢复均为硬更新。
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_adaptive_boundary_enable <= 1'b0;
			isp_continuous_update_o <= 1'b0;
		end else if(i_frame_boundary)begin
			reg_adaptive_boundary_enable <= adaptive_enable_sync[1];
			isp_continuous_update_o <= adaptive_enable_sync[1] &&
				reg_adaptive_boundary_enable && !pending_o && !flag_restore_pending;
		end
	end
	// 帧中关闭必须保持恢复请求直至下一输出帧边界。
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			flag_restore_pending <= 1'b0;
		end else if(adaptive_enable_sync[1])begin
			flag_restore_pending <= 1'b0;
		end else if(i_frame_boundary && (flag_restore_pending || adaptive_enable_sync_d))begin
			flag_restore_pending <= 1'b0;
		end else if(adaptive_enable_sync_d && !adaptive_enable_sync[1])begin
			flag_restore_pending <= 1'b1;
		end
	end

	// 帧应用后一拍取完整配置；源域确认消费前不覆盖稳定载荷。
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_live_payload <= {160'd0,16'd0,1'b0,64'd0,16'd0,DEFAULT_CONFIG};
			flag_live_request <= 1'b0;
			flag_live_capture <= 1'b0;
			reg_live_id <= 16'd0;
			reg_live_ack_sync <= 2'd0;
		end else begin
			reg_live_ack_sync <= {reg_live_ack_sync[0],flag_live_ack};
			flag_live_capture <= i_frame_boundary;
			if(flag_live_capture && flag_live_request == reg_live_ack_sync[1])begin
				reg_live_payload <= {i_weather_actual,reg_live_id + 16'd1,adaptive_enable_sync[1],
					adaptive_status,config_epoch_o,active_config_o};
				reg_live_id <= reg_live_id + 16'd1;
				flag_live_request <= !flag_live_request;
			end
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_live_request_sync <= 2'd0;
			flag_live_ack <= 1'b0;
			src_live_payload_o <= {160'd0,16'd0,1'b0,64'd0,16'd0,DEFAULT_CONFIG};
			src_live_valid_o <= 1'b0;
		end else begin
			reg_live_request_sync <= {reg_live_request_sync[0],flag_live_request};
			if(reg_live_request_sync[1] != flag_live_ack)begin
				src_live_payload_o <= reg_live_payload;
				src_live_valid_o <= 1'b1;
				flag_live_ack <= reg_live_request_sync[1];
			end
		end
	end

	// Source payload is held until the destination has applied and acknowledged it.
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_payload <= DEFAULT_CONFIG;
		end else if(i_update && !o_busy)begin
			reg_payload <= i_config;
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			flag_request <= 1'b0;
		end else if(i_update && !o_busy)begin
			flag_request <= !flag_request;
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_ack_sync <= 2'd0;
		end else begin
			reg_ack_sync <= {reg_ack_sync[0], flag_ack};
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_ack_previous <= 1'b0;
		end else begin
			reg_ack_previous <= reg_ack_sync[1];
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			applied_o <= 1'b0;
			src_active_snapshot_o <= DEFAULT_CONFIG;
			src_config_epoch_o <= 16'd0;
		end else begin
			applied_o <= reg_ack_sync[1] != reg_ack_previous;
			if(reg_ack_sync[1] != reg_ack_previous)begin
				src_active_snapshot_o <= reg_ack_active_config;
				src_config_epoch_o <= reg_ack_config_epoch;
			end
		end
	end

	// Destination synchronizes only the request event. The bundled payload is
	// stable before the toggle and remains stable until the returned ACK.
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_request_sync <= 2'd0;
		end else begin
			reg_request_sync <= {reg_request_sync[0], flag_request};
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			pending_o <= 1'b0;
		end else if(pending_o && i_frame_boundary)begin
			pending_o <= 1'b0;
		end else if(reg_request_sync[1] != flag_ack)begin
			pending_o <= 1'b1;
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_pending_payload <= DEFAULT_CONFIG;
		end else if(!pending_o && reg_request_sync[1] != flag_ack)begin
			reg_pending_payload <= reg_payload;
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			active_config_o <= DEFAULT_CONFIG;
			config_epoch_o <= 16'd0;
			effective_epoch_o <= 64'd0;
			change_mask_o <= 4'd0;
		end else begin
			change_mask_o <= 4'd0;
			if((pending_o || adaptive_enable_sync[1] || flag_restore_pending || adaptive_enable_sync_d) && i_frame_boundary)begin
				if(pending_o) active_config_o <= reg_pending_payload;
				else if(adaptive_enable_sync[1]) active_config_o <= adaptive_next_config;
				else active_config_o <= adaptive_restore_config;
				if(active_config_o != frame_apply_config)begin
					config_epoch_o <= config_epoch_o + 16'd1;
				end
				if(active_effective_tile0 != frame_apply_effective_tile0)begin
					effective_epoch_o[15:0] <= effective_epoch_o[15:0] + 16'd1;
					change_mask_o[0] <= 1'b1;
				end
				if(active_effective_tile1 != frame_apply_effective_tile1)begin
					effective_epoch_o[31:16] <= effective_epoch_o[31:16] + 16'd1;
					change_mask_o[1] <= 1'b1;
				end
				if(active_effective_tile2 != frame_apply_effective_tile2)begin
					effective_epoch_o[47:32] <= effective_epoch_o[47:32] + 16'd1;
					change_mask_o[2] <= 1'b1;
				end
				if(active_effective_tile3 != frame_apply_effective_tile3)begin
					effective_epoch_o[63:48] <= effective_epoch_o[63:48] + 16'd1;
					change_mask_o[3] <= 1'b1;
				end
			end
		end
	end
	// Return payload is written before the ACK toggle at the same destination
	// edge; the source sees both only after its two-stage toggle synchronizer.
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			flag_ack <= 1'b0;
			reg_ack_active_config <= DEFAULT_CONFIG;
			reg_ack_config_epoch <= 16'd0;
		end else if(pending_o && i_frame_boundary)begin
			flag_ack <= reg_request_sync[1];
			reg_ack_active_config <= reg_pending_payload;
			reg_ack_config_epoch <= (active_config_o != reg_pending_payload) ? config_epoch_o + 16'd1 : config_epoch_o;
		end
	end
endmodule
