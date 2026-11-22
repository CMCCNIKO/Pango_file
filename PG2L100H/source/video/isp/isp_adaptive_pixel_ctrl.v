//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-11-12
// Update Date: 2026-11-16
// Design Name: ISP 输入场景与输出近白预警的有界自适应
// Module Name: isp_adaptive_pixel_ctrl
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Description:
//   仅使用frame_id相同、输入epoch一致的完整4x4抽样摘要。
//   双组触发器直方图：每区32档、4区、计数14位，不保存整帧。
//   输入决定色调；输出使用任一通道>=248的新增近白预警限制增益。
//   近白预警不等于255剪切计数，不能用它声称恢复或减少真实剪切。
//   白平衡仅采可信中性像素，有界慢调；无极值范围减益或offset积分。
//   输入输出相位取tag frame_id，每16帧轮换完整4x4网格相位。
// Revision: 2026-11-16 v025 natural scene adaptation.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module isp_adaptive_pixel_ctrl
#(
	parameter integer C_EXPECTED_SAMPLES = 12800,
	parameter integer C_NEUTRAL_MIN_SAMPLES = 128
)
(
	input i_clk,
	input i_rst_n,
	input i_enable,
	input i_frame_boundary,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	input [23:0] i_scene_rgb,
	input [159:0] i_scene_tag,
	input i_scene_frame_boundary,
	input [15:0] i_scene_epoch,
	input [519:0] i_active_config,
	output [519:0] o_next_config,
	output [519:0] o_restore_config,
	output [63:0] o_status,
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
	output [13:0] o_summary_neutral_count
);
	localparam integer C_P10 = (C_EXPECTED_SAMPLES + 9) / 10;
	localparam integer C_P50 = (C_EXPECTED_SAMPLES + 1) / 2;
	localparam integer C_P90 = (C_EXPECTED_SAMPLES * 9 + 9) / 10;
	localparam integer C_P99 = (C_EXPECTED_SAMPLES * 99 + 99) / 100;
	localparam integer C_HIGH = C_EXPECTED_SAMPLES / 100;
	localparam integer C_CLIP = C_EXPECTED_SAMPLES * 3 / 100;
	localparam integer C_LOW_MIXED = (C_EXPECTED_SAMPLES * 35 + 99) / 100;
	localparam integer C_HIGH_MIXED = (C_EXPECTED_SAMPLES * 15 + 99) / 100;
	// 白平衡范围以软件画质回归确定，显式保留定点上下界。
	localparam [8:0] C_WB_MIN = 9'd254;
	localparam [8:0] C_WB_MAX = 9'd258;
	localparam [127:0] C_DEFAULT_TILE = {8'h71,10'd256,10'd256,10'd256,
		8'd0,8'd0,8'd0,2'd1,3'd1,8'd12,8'd32,3'd1,8'd4,8'd16,3'd0,3'd0,20'd0};
	localparam [3:0] ST_WAIT = 4'd0;
	localparam [3:0] ST_SCAN = 4'd1;
	localparam [3:0] ST_DECIDE = 4'd2;
	localparam [3:0] ST_WB_INIT = 4'd3;
	localparam [3:0] ST_WB_ADD = 4'd4;
	localparam [3:0] ST_WB_COMPARE = 4'd5;
	localparam [3:0] ST_COMMIT = 4'd6;

	reg [519:0] reg_auto_config;
	reg [519:0] reg_restore_config;
	reg reg_enable_d;
	reg reg_hist_bank;
	reg [13:0] reg_hist_count [0:255];
	reg [13:0] reg_in_count [0:3];
	reg [13:0] reg_in_clip [0:3];
	reg [13:0] reg_in_low48 [0:3];
	reg [13:0] reg_in_fog_count [0:3];
	reg [13:0] reg_in_texture_count [0:3];
	reg reg_previous_sample_valid;
	reg [1:0] reg_previous_sample_tile;
	reg [9:0] reg_previous_sample_x;
	reg [8:0] reg_previous_sample_y;
	reg [7:0] reg_previous_sample_luma;
	reg [13:0] reg_in_high224 [0:3];
	reg [13:0] reg_in_high240 [0:3];
	reg [13:0] reg_neutral_count [0:3];
	reg [21:0] reg_neutral_r [0:3];
	reg [21:0] reg_neutral_g [0:3];
	reg [21:0] reg_neutral_b [0:3];
	reg [3:0] reg_in_overflow;
	reg reg_in_seen;
	reg reg_in_consistent;
	reg [31:0] reg_in_frame;
	reg [15:0] reg_in_epoch;
	reg reg_in_generation;
	reg reg_in_bank;
	reg [31:0] reg_in_snap_frame;
	reg [15:0] reg_in_snap_epoch;
	reg [3:0] reg_in_valid;
	reg [13:0] reg_in_snap_clip [0:3];
	reg [13:0] reg_in_snap_low48 [0:3];
	reg [13:0] reg_in_snap_fog_count [0:3];
	reg [13:0] reg_in_snap_texture_count [0:3];
	reg [13:0] reg_in_snap_high224 [0:3];
	reg [13:0] reg_in_snap_high240 [0:3];
	reg [13:0] reg_neutral_snap_count [0:3];
	reg [21:0] reg_neutral_snap_r [0:3];
	reg [21:0] reg_neutral_snap_g [0:3];
	reg [21:0] reg_neutral_snap_b [0:3];
	reg [13:0] reg_out_count [0:3];
	reg [13:0] reg_out_clip [0:3];
	reg [3:0] reg_out_overflow;
	reg reg_out_seen;
	reg reg_out_consistent;
	reg [31:0] reg_out_frame;
	reg reg_out_generation;
	reg [31:0] reg_out_snap_frame;
	reg [3:0] reg_out_valid;
	reg [13:0] reg_out_snap_clip [0:3];

	reg [3:0] state_current;
	reg reg_in_consumed;
	reg reg_out_consumed;
	reg reg_work_bank;
	reg [31:0] reg_work_frame;
	reg [15:0] reg_work_epoch;
	reg [3:0] reg_work_valid;
	reg [13:0] reg_work_in_clip [0:3];
	reg [13:0] reg_work_out_clip [0:3];
	reg [13:0] reg_work_low48 [0:3];
	reg [13:0] reg_work_fog_count [0:3];
	reg [13:0] reg_work_texture_count [0:3];
	reg [13:0] reg_work_high224 [0:3];
	reg [13:0] reg_work_high240 [0:3];
	reg [13:0] reg_work_neutral_count [0:3];
	reg [21:0] reg_work_r [0:3];
	reg [21:0] reg_work_g [0:3];
	reg [21:0] reg_work_b [0:3];
	reg [1:0] reg_scan_tile;
	reg [4:0] reg_scan_bin;
	reg [14:0] reg_cumulative;
	reg [3:0] reg_quantile_found;
	reg [7:0] reg_p10;
	reg [7:0] reg_p50;
	reg [7:0] reg_p90;
	reg [7:0] reg_p99;
	reg [2:0] reg_candidate_profile [0:3];
	reg [2:0] reg_candidate_count [0:3];
	reg [1:0] reg_update_phase [0:3];
	reg [9:0] reg_common_gain [0:3];
	reg [9:0] reg_wb_r [0:3];
	reg [9:0] reg_wb_b [0:3];
	reg [2:0] reg_next_profile;
	reg [9:0] reg_next_common;
	reg [9:0] reg_next_r;
	reg [9:0] reg_next_b;
	reg reg_guard_active;
	reg reg_recover_phase;
	reg reg_search_blue;
	reg [29:0] reg_search_goal;
	reg [21:0] reg_search_channel;
	reg [29:0] reg_search_product;
	reg [29:0] reg_search_next;
	reg [29:0] reg_search_midpoint;
	reg [8:0] reg_search_gain;
	reg [31:0] reg_status_frame;
	reg [15:0] reg_status_epoch;
	reg [3:0] reg_status_valid;
	integer in_index;
	integer out_index;
	integer ctrl_index;
	genvar hist_index;

	wire [1:0] scene_tile;
	wire [1:0] output_tile;
	wire [9:0] scene_luma4;
	wire [7:0] scene_luma;
	wire scene_sample;
	wire output_sample;
	wire scene_clip;
	wire output_clip;
	reg [7:0] scene_max;
	reg [7:0] scene_min;
	wire scene_neutral;
	wire scene_fog_candidate;
	wire scene_texture_candidate;
	wire [7:0] scene_sample_difference;
	wire [7:0] histogram_write_address;
	wire [7:0] histogram_read_address;
	wire [14:0] scan_next_cumulative;
	wire [31:0] snapshot_frame_delta;
	reg [2:0] desired_profile;
	wire [2:0] current_profile;
	wire [8:0] search_rounded_gain;
	wire [8:0] search_target_gain;
	wire [21:0] search_input_channel;
	wire [9:0] normalized_base_target;
	wire [9:0] normalized_base_recovered;
	wire [9:0] normalized_base;
	wire signed [11:0] final_gain_r;
	wire signed [11:0] final_gain_b;

	assign scene_tile = i_scene_tag[133:132];
	assign output_tile = i_tag[133:132];
	assign scene_luma4 = {2'd0,i_scene_rgb[23:16]} +
		({2'd0,i_scene_rgb[15:8]} << 1) + {2'd0,i_scene_rgb[7:0]};
	assign scene_luma = scene_luma4[9:2];
	assign scene_sample = i_enable && i_scene_tag[157] && i_scene_tag[112] &&
		(i_scene_tag[123:122] == i_scene_tag[81:80]) &&
		(i_scene_tag[114:113] == i_scene_tag[83:82]);
	assign output_sample = i_enable && i_tag[157] && i_tag[112] &&
		(i_tag[123:122] == i_tag[81:80]) &&
		(i_tag[114:113] == i_tag[83:82]);
	assign scene_clip = i_scene_rgb[23:16] >= 8'd248 ||
		i_scene_rgb[15:8] >= 8'd248 || i_scene_rgb[7:0] >= 8'd248;
	assign output_clip = i_rgb[23:16] >= 8'd248 ||
		i_rgb[15:8] >= 8'd248 || i_rgb[7:0] >= 8'd248;
	always@(*)begin
		scene_max = i_scene_rgb[23:16];
		if(i_scene_rgb[15:8] > scene_max)begin scene_max = i_scene_rgb[15:8]; end
		if(i_scene_rgb[7:0] > scene_max)begin scene_max = i_scene_rgb[7:0]; end
		scene_min = i_scene_rgb[23:16];
		if(i_scene_rgb[15:8] < scene_min)begin scene_min = i_scene_rgb[15:8]; end
		if(i_scene_rgb[7:0] < scene_min)begin scene_min = i_scene_rgb[7:0]; end
	end
	assign scene_neutral = scene_luma >= 8'd48 && scene_luma <= 8'd208 &&
		(scene_max - scene_min) <= 8'd12;
	// 天气统计复用ISP前抽样，不复制暗通道/直方图或图像行缓存。
	// RGB加法扩展9位，避免亮蓝/白像素比较回绕造成误判。
	assign scene_fog_candidate = scene_min >= 8'd64 &&
		(scene_max - scene_min) >= 8'd16 && (scene_max - scene_min) <= 8'd64 &&
		!(({1'b0,i_scene_rgb[7:0]} >= {1'b0,i_scene_rgb[23:16]} + 9'd16) &&
		  ({1'b0,i_scene_rgb[7:0]} >= {1'b0,i_scene_rgb[15:8]} + 9'd8));
	assign scene_sample_difference = scene_luma >= reg_previous_sample_luma ?
		scene_luma - reg_previous_sample_luma : reg_previous_sample_luma - scene_luma;
	// v026修订：原温和纹理计数复用为强边缘S，避免新增成组计数器。
	assign scene_texture_candidate = reg_previous_sample_valid &&
		scene_tile == reg_previous_sample_tile &&
		i_scene_tag[121:113] == reg_previous_sample_y &&
		i_scene_tag[131:122] == reg_previous_sample_x + 10'd4 &&
		scene_sample_difference >= 8'd32;
	assign histogram_write_address = {reg_hist_bank,scene_tile,scene_luma[7:3]};
	assign histogram_read_address = {reg_work_bank,reg_scan_tile,reg_scan_bin};
	assign scan_next_cumulative = reg_cumulative + {1'b0,reg_hist_count[histogram_read_address]};
	assign snapshot_frame_delta = reg_in_snap_frame - reg_out_snap_frame;
	assign current_profile = reg_auto_config[reg_scan_tile * 128 + 23 +: 3];
	assign search_rounded_gain = reg_search_goal >= reg_search_midpoint ?
		reg_search_gain + 9'd1 : reg_search_gain;
	assign search_target_gain = reg_search_goal <= reg_search_product ? C_WB_MIN :
		(search_rounded_gain > C_WB_MAX ? C_WB_MAX : search_rounded_gain);
	assign search_input_channel = reg_search_blue ? reg_work_b[reg_scan_tile] : reg_work_r[reg_scan_tile];
	assign normalized_base_target = reg_next_r > reg_next_b ?
		(reg_next_r > 10'd256 ? 10'd512 - reg_next_r : 10'd256) :
		(reg_next_b > 10'd256 ? 10'd512 - reg_next_b : 10'd256);
	assign normalized_base_recovered = !reg_guard_active && reg_recover_phase &&
		reg_next_common < normalized_base_target ? reg_next_common + 10'd1 : reg_next_common;
	assign normalized_base = normalized_base_recovered > normalized_base_target ?
		normalized_base_target : normalized_base_recovered;
	assign final_gain_r = $signed({2'd0,normalized_base}) + $signed({2'd0,reg_next_r}) - 12'sd256;
	assign final_gain_b = $signed({2'd0,normalized_base}) + $signed({2'd0,reg_next_b}) - 12'sd256;
	assign o_next_config = reg_auto_config;
	assign o_restore_config = reg_restore_config;
	assign o_status = {reg_status_frame,i_active_config[409:407],i_active_config[281:279],
		i_active_config[153:151],i_active_config[25:23],reg_status_valid,reg_status_epoch};
	// ST_DECIDE时百分位已扫描完成；直接导出当前tile摘要，不增加像素拍数。
	assign o_summary_valid = i_enable && state_current == ST_DECIDE;
	assign o_summary_complete = reg_work_valid[reg_scan_tile];
	assign o_summary_tile = reg_scan_tile;
	assign o_summary_frame = reg_work_frame;
	assign o_summary_epoch = reg_work_epoch;
	assign o_summary_p10 = reg_p10;
	assign o_summary_p50 = reg_p50;
	assign o_summary_p90 = reg_p90;
	assign o_summary_low48 = reg_work_low48[reg_scan_tile];
	assign o_summary_fog_count = reg_work_fog_count[reg_scan_tile];
	// texture端口保留命名兼容；语义为abs(deltaY)>=32的同行强边缘对数。
	assign o_summary_texture_count = reg_work_texture_count[reg_scan_tile];
	// 复用原白平衡中性样本计数，不增加统计触发器或图像缓存。
	assign o_summary_neutral_count = reg_work_neutral_count[reg_scan_tile];

	// 仅同源同行相邻4px抽样形成纹理对；缺源、消隐及帧切换打断相邻性。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_previous_sample_valid <= 1'b0;
			reg_previous_sample_tile <= 2'd0;
			reg_previous_sample_x <= 10'd0;
			reg_previous_sample_y <= 9'd0;
			reg_previous_sample_luma <= 8'd0;
		end else if(!i_enable || i_scene_frame_boundary ||
			!i_scene_tag[157] || !i_scene_tag[112])begin
			reg_previous_sample_valid <= 1'b0;
		end else if(scene_sample)begin
			reg_previous_sample_valid <= 1'b1;
			reg_previous_sample_tile <= scene_tile;
			reg_previous_sample_x <= i_scene_tag[131:122];
			reg_previous_sample_y <= i_scene_tag[121:113];
			reg_previous_sample_luma <= scene_luma;
		end
	end

	always@(*)begin
		desired_profile = 3'd0;
		if(((reg_p50 < 8'd64) && (reg_work_high240[reg_scan_tile] > C_HIGH)) ||
			((reg_work_low48[reg_scan_tile] >= C_LOW_MIXED) &&
			(reg_work_high224[reg_scan_tile] >= C_HIGH_MIXED)))begin
			desired_profile = 3'd7;
		end else if(reg_p50 < 8'd32)begin desired_profile = 3'd5;
		end else if(reg_p50 < 8'd64)begin desired_profile = 3'd4;
		end else if(reg_work_high240[reg_scan_tile] > C_HIGH)begin desired_profile = 3'd6;
		end
	end

	// 独立FF桶计数使像素路径仅经过地址译码和一次递增。
	generate
		for(hist_index = 0; hist_index < 256; hist_index = hist_index + 1)begin:gen_histogram_ff
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin reg_hist_count[hist_index] <= 14'd0;
				end else if(!i_enable)begin reg_hist_count[hist_index] <= 14'd0;
				end else if(i_scene_frame_boundary && ((hist_index / 128) != reg_hist_bank))begin
					reg_hist_count[hist_index] <= 14'd0;
				end else if(scene_sample && !i_scene_frame_boundary &&
					(histogram_write_address == hist_index) && reg_hist_count[hist_index] != 14'h3fff)begin
					reg_hist_count[hist_index] <= reg_hist_count[hist_index] + 14'd1;
				end
			end
		end
	endgenerate

	// 计数在使用前复位；快照valid屏蔽复位时不必初始化的数据载荷。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_hist_bank <= 1'b0;
			reg_in_seen <= 1'b0;
			reg_in_consistent <= 1'b1;
			reg_in_frame <= 32'd0;
			reg_in_epoch <= 16'd0;
			reg_in_generation <= 1'b0;
			reg_in_bank <= 1'b0;
			reg_in_snap_frame <= 32'd0;
			reg_in_snap_epoch <= 16'd0;
			reg_in_valid <= 4'd0;
			reg_in_overflow <= 4'd0;
			for(in_index = 0; in_index < 4; in_index = in_index + 1)begin
				reg_in_count[in_index] <= 14'd0;
				reg_in_clip[in_index] <= 14'd0;
				reg_in_low48[in_index] <= 14'd0;
				reg_in_fog_count[in_index] <= 14'd0;
				reg_in_texture_count[in_index] <= 14'd0;
				reg_in_high224[in_index] <= 14'd0;
				reg_in_high240[in_index] <= 14'd0;
				reg_neutral_count[in_index] <= 14'd0;
				reg_neutral_r[in_index] <= 22'd0;
				reg_neutral_g[in_index] <= 22'd0;
				reg_neutral_b[in_index] <= 22'd0;
			end
		end else if(!i_enable || i_scene_frame_boundary)begin
			reg_hist_bank <= i_enable ? !reg_hist_bank : 1'b0;
			reg_in_generation <= i_enable ? !reg_in_generation : 1'b0;
			reg_in_bank <= reg_hist_bank;
			reg_in_snap_frame <= reg_in_seen ? reg_in_frame : i_scene_tag[111:80];
			reg_in_snap_epoch <= reg_in_epoch;
			reg_in_seen <= 1'b0;
			reg_in_consistent <= 1'b1;
			reg_in_overflow <= 4'd0;
			for(in_index = 0; in_index < 4; in_index = in_index + 1)begin
				reg_in_valid[in_index] <= i_enable && reg_in_seen && reg_in_consistent &&
					!reg_in_overflow[in_index] && reg_in_count[in_index] == C_EXPECTED_SAMPLES;
				reg_in_snap_clip[in_index] <= reg_in_clip[in_index];
				reg_in_snap_low48[in_index] <= reg_in_low48[in_index];
				reg_in_snap_fog_count[in_index] <= reg_in_fog_count[in_index];
				reg_in_snap_texture_count[in_index] <= reg_in_texture_count[in_index];
				reg_in_snap_high224[in_index] <= reg_in_high224[in_index];
				reg_in_snap_high240[in_index] <= reg_in_high240[in_index];
				reg_neutral_snap_count[in_index] <= reg_neutral_count[in_index];
				reg_neutral_snap_r[in_index] <= reg_neutral_r[in_index];
				reg_neutral_snap_g[in_index] <= reg_neutral_g[in_index];
				reg_neutral_snap_b[in_index] <= reg_neutral_b[in_index];
				reg_in_count[in_index] <= 14'd0;
				reg_in_clip[in_index] <= 14'd0;
				reg_in_low48[in_index] <= 14'd0;
				reg_in_fog_count[in_index] <= 14'd0;
				reg_in_texture_count[in_index] <= 14'd0;
				reg_in_high224[in_index] <= 14'd0;
				reg_in_high240[in_index] <= 14'd0;
				reg_neutral_count[in_index] <= 14'd0;
				reg_neutral_r[in_index] <= 22'd0;
				reg_neutral_g[in_index] <= 22'd0;
				reg_neutral_b[in_index] <= 22'd0;
			end
		end else if(scene_sample)begin
			if(!reg_in_seen)begin
				reg_in_frame <= i_scene_tag[111:80];
				reg_in_epoch <= i_scene_epoch;
				reg_in_seen <= 1'b1;
			end else if(reg_in_frame != i_scene_tag[111:80] || reg_in_epoch != i_scene_epoch)begin
				reg_in_consistent <= 1'b0;
			end
			if(reg_in_count[scene_tile] != 14'h3fff)begin
				reg_in_count[scene_tile] <= reg_in_count[scene_tile] + 14'd1;
				if(scene_clip)begin reg_in_clip[scene_tile] <= reg_in_clip[scene_tile] + 14'd1; end
				if(scene_luma < 8'd48)begin reg_in_low48[scene_tile] <= reg_in_low48[scene_tile] + 14'd1; end
				if(scene_fog_candidate)begin reg_in_fog_count[scene_tile] <= reg_in_fog_count[scene_tile] + 14'd1; end
				if(scene_texture_candidate)begin reg_in_texture_count[scene_tile] <= reg_in_texture_count[scene_tile] + 14'd1; end
				if(scene_luma >= 8'd224)begin reg_in_high224[scene_tile] <= reg_in_high224[scene_tile] + 14'd1; end
				if(scene_luma >= 8'd240)begin reg_in_high240[scene_tile] <= reg_in_high240[scene_tile] + 14'd1; end
				if(scene_neutral)begin
					reg_neutral_count[scene_tile] <= reg_neutral_count[scene_tile] + 14'd1;
					reg_neutral_r[scene_tile] <= reg_neutral_r[scene_tile] + {14'd0,i_scene_rgb[23:16]};
					reg_neutral_g[scene_tile] <= reg_neutral_g[scene_tile] + {14'd0,i_scene_rgb[15:8]};
					reg_neutral_b[scene_tile] <= reg_neutral_b[scene_tile] + {14'd0,i_scene_rgb[7:0]};
				end
			end else begin reg_in_overflow[scene_tile] <= 1'b1; end
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_out_generation <= 1'b0;
			reg_out_seen <= 1'b0;
			reg_out_consistent <= 1'b1;
			reg_out_frame <= 32'd0;
			reg_out_snap_frame <= 32'd0;
			reg_out_valid <= 4'd0;
			reg_out_overflow <= 4'd0;
			for(out_index = 0; out_index < 4; out_index = out_index + 1)begin
				reg_out_count[out_index] <= 14'd0;
				reg_out_clip[out_index] <= 14'd0;
			end
		end else if(!i_enable || i_frame_boundary)begin
			reg_out_generation <= i_enable ? !reg_out_generation : 1'b0;
			reg_out_snap_frame <= reg_out_seen ? reg_out_frame : i_tag[111:80];
			reg_out_seen <= 1'b0;
			reg_out_consistent <= 1'b1;
			reg_out_overflow <= 4'd0;
			for(out_index = 0; out_index < 4; out_index = out_index + 1)begin
				reg_out_valid[out_index] <= i_enable && reg_out_seen && reg_out_consistent &&
					!reg_out_overflow[out_index] && reg_out_count[out_index] == C_EXPECTED_SAMPLES;
				reg_out_snap_clip[out_index] <= reg_out_clip[out_index];
				reg_out_count[out_index] <= 14'd0;
				reg_out_clip[out_index] <= 14'd0;
			end
		end else if(output_sample)begin
			if(!reg_out_seen)begin
				reg_out_frame <= i_tag[111:80];
				reg_out_seen <= 1'b1;
			end else if(reg_out_frame != i_tag[111:80])begin reg_out_consistent <= 1'b0; end
			if(reg_out_count[output_tile] != 14'h3fff)begin
				reg_out_count[output_tile] <= reg_out_count[output_tile] + 14'd1;
				if(output_clip)begin reg_out_clip[output_tile] <= reg_out_clip[output_tile] + 14'd1; end
			end else begin reg_out_overflow[output_tile] <= 1'b1; end
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_auto_config <= {8'h1e,{4{C_DEFAULT_TILE}}};
			reg_restore_config <= {8'h1e,{4{C_DEFAULT_TILE}}};
			reg_enable_d <= 1'b0;
			state_current <= ST_WAIT;
			reg_in_consumed <= 1'b0;
			reg_out_consumed <= 1'b0;
			reg_work_bank <= 1'b0;
			reg_work_frame <= 32'd0;
			reg_work_epoch <= 16'd0;
			reg_work_valid <= 4'd0;
			reg_scan_tile <= 2'd0;
			reg_scan_bin <= 5'd0;
			reg_cumulative <= 15'd0;
			reg_quantile_found <= 4'd0;
			reg_p10 <= 8'd252;
			reg_p50 <= 8'd252;
			reg_p90 <= 8'd252;
			reg_p99 <= 8'd252;
			reg_next_profile <= 3'd0;
			reg_next_common <= 10'd256;
			reg_next_r <= 10'd256;
			reg_next_b <= 10'd256;
			reg_guard_active <= 1'b0;
			reg_recover_phase <= 1'b0;
			reg_search_blue <= 1'b0;
			reg_search_goal <= 30'd0;
			reg_search_channel <= 22'd0;
			reg_search_product <= 30'd0;
			reg_search_next <= 30'd0;
			reg_search_midpoint <= 30'd0;
			reg_search_gain <= C_WB_MIN;
			reg_status_frame <= 32'd0;
			reg_status_epoch <= 16'd0;
			reg_status_valid <= 4'd0;
			for(ctrl_index = 0; ctrl_index < 4; ctrl_index = ctrl_index + 1)begin
				reg_candidate_profile[ctrl_index] <= 3'd0;
				reg_candidate_count[ctrl_index] <= 3'd0;
				reg_update_phase[ctrl_index] <= 2'd0;
				reg_common_gain[ctrl_index] <= 10'd256;
				reg_wb_r[ctrl_index] <= 10'd256;
				reg_wb_b[ctrl_index] <= 10'd256;
			end
		end else begin
			reg_enable_d <= i_enable;
			if(i_enable && !reg_enable_d)begin
				reg_restore_config <= i_active_config;
				reg_auto_config <= i_active_config;
				reg_auto_config[519:512] <= i_active_config[519:512] | 8'h1f;
				for(ctrl_index = 0; ctrl_index < 4; ctrl_index = ctrl_index + 1)begin
					if(i_active_config[ctrl_index * 128 + 120 +: 8] == 8'd0)begin
						reg_auto_config[ctrl_index * 128 +: 128] <= C_DEFAULT_TILE;
					end
					reg_auto_config[ctrl_index * 128 + 120 +: 8] <= i_active_config[ctrl_index * 128 + 120 +: 8] | 8'h71;
					reg_auto_config[ctrl_index * 128 + 110 +: 10] <= 10'd256;
					reg_auto_config[ctrl_index * 128 + 100 +: 10] <= 10'd256;
					reg_auto_config[ctrl_index * 128 + 90 +: 10] <= 10'd256;
					reg_auto_config[ctrl_index * 128 + 82 +: 8] <= 8'd0;
					reg_auto_config[ctrl_index * 128 + 74 +: 8] <= 8'd0;
					reg_auto_config[ctrl_index * 128 + 66 +: 8] <= 8'd0;
					reg_auto_config[ctrl_index * 128 + 23 +: 3] <= 3'd0;
					reg_auto_config[ctrl_index * 128 + 20 +: 3] <= 3'd0;
				end
			end else if(!i_enable && reg_enable_d)begin reg_auto_config <= reg_restore_config; end
			if(!i_enable || !reg_enable_d)begin
				state_current <= ST_WAIT;
				reg_in_consumed <= 1'b0;
				reg_out_consumed <= 1'b0;
				reg_status_valid <= 4'd0;
				for(ctrl_index = 0; ctrl_index < 4; ctrl_index = ctrl_index + 1)begin
					reg_candidate_profile[ctrl_index] <= 3'd0;
					reg_candidate_count[ctrl_index] <= 3'd0;
					reg_update_phase[ctrl_index] <= 2'd0;
					reg_common_gain[ctrl_index] <= 10'd256;
					reg_wb_r[ctrl_index] <= 10'd256;
					reg_wb_b[ctrl_index] <= 10'd256;
				end
			end else begin
				case(state_current)
					ST_WAIT:begin
						if((reg_in_generation != reg_in_consumed) && (reg_out_generation != reg_out_consumed))begin
							if(reg_in_snap_frame == reg_out_snap_frame)begin
								reg_in_consumed <= reg_in_generation;
								reg_out_consumed <= reg_out_generation;
								reg_work_bank <= reg_in_bank;
								reg_work_frame <= reg_in_snap_frame;
								reg_work_epoch <= reg_in_snap_epoch;
								reg_work_valid <= reg_in_valid & reg_out_valid;
								for(ctrl_index = 0; ctrl_index < 4; ctrl_index = ctrl_index + 1)begin
									reg_work_in_clip[ctrl_index] <= reg_in_snap_clip[ctrl_index];
									reg_work_out_clip[ctrl_index] <= reg_out_snap_clip[ctrl_index];
									reg_work_low48[ctrl_index] <= reg_in_snap_low48[ctrl_index];
									reg_work_fog_count[ctrl_index] <= reg_in_snap_fog_count[ctrl_index];
									reg_work_texture_count[ctrl_index] <= reg_in_snap_texture_count[ctrl_index];
									reg_work_high224[ctrl_index] <= reg_in_snap_high224[ctrl_index];
									reg_work_high240[ctrl_index] <= reg_in_snap_high240[ctrl_index];
									reg_work_neutral_count[ctrl_index] <= reg_neutral_snap_count[ctrl_index];
									reg_work_r[ctrl_index] <= reg_neutral_snap_r[ctrl_index];
									reg_work_g[ctrl_index] <= reg_neutral_snap_g[ctrl_index];
									reg_work_b[ctrl_index] <= reg_neutral_snap_b[ctrl_index];
								end
								reg_scan_tile <= 2'd0;
								reg_scan_bin <= 5'd0;
								reg_cumulative <= 15'd0;
								reg_quantile_found <= 4'd0;
								reg_p10 <= 8'd252;
								reg_p50 <= 8'd252;
								reg_p90 <= 8'd252;
								reg_p99 <= 8'd252;
								state_current <= ST_SCAN;
							end else if(!snapshot_frame_delta[31])begin reg_out_consumed <= reg_out_generation;
							end else begin reg_in_consumed <= reg_in_generation; end
						end
					end
					ST_SCAN:begin
						reg_cumulative <= scan_next_cumulative;
						if(!reg_quantile_found[0] && scan_next_cumulative >= C_P10)begin
							reg_p10 <= {reg_scan_bin,3'b100}; reg_quantile_found[0] <= 1'b1;
						end
						if(!reg_quantile_found[1] && scan_next_cumulative >= C_P50)begin
							reg_p50 <= {reg_scan_bin,3'b100}; reg_quantile_found[1] <= 1'b1;
						end
						if(!reg_quantile_found[2] && scan_next_cumulative >= C_P90)begin
							reg_p90 <= {reg_scan_bin,3'b100}; reg_quantile_found[2] <= 1'b1;
						end
						if(!reg_quantile_found[3] && scan_next_cumulative >= C_P99)begin
							reg_p99 <= {reg_scan_bin,3'b100}; reg_quantile_found[3] <= 1'b1;
						end
						if(reg_scan_bin == 5'd31)begin state_current <= ST_DECIDE;
						end else begin reg_scan_bin <= reg_scan_bin + 5'd1; end
					end
					ST_DECIDE:begin
						reg_next_profile <= current_profile;
						reg_next_common <= reg_common_gain[reg_scan_tile];
						reg_next_r <= reg_wb_r[reg_scan_tile];
						reg_next_b <= reg_wb_b[reg_scan_tile];
						reg_guard_active <= 1'b0;
						reg_recover_phase <= reg_update_phase[reg_scan_tile] == 2'd3;
						reg_search_blue <= 1'b0;
						state_current <= ST_COMMIT;
						if(reg_work_valid[reg_scan_tile])begin
							reg_update_phase[reg_scan_tile] <= reg_update_phase[reg_scan_tile] + 2'd1;
							if(desired_profile == current_profile)begin
								reg_candidate_profile[reg_scan_tile] <= desired_profile;
								reg_candidate_count[reg_scan_tile] <= 3'd0;
							end else if(desired_profile != reg_candidate_profile[reg_scan_tile])begin
								reg_candidate_profile[reg_scan_tile] <= desired_profile;
								reg_candidate_count[reg_scan_tile] <= 3'd1;
							end else if(reg_candidate_count[reg_scan_tile] >= 3'd3)begin
								reg_next_profile <= desired_profile;
								reg_candidate_count[reg_scan_tile] <= 3'd0;
							end else begin
								reg_candidate_count[reg_scan_tile] <= reg_candidate_count[reg_scan_tile] + 3'd1;
							end
							if((reg_work_out_clip[reg_scan_tile] > C_CLIP) &&
								({1'b0,reg_work_out_clip[reg_scan_tile]} > ({1'b0,reg_work_in_clip[reg_scan_tile]} + C_HIGH)))begin
								reg_guard_active <= 1'b1;
								if(reg_common_gain[reg_scan_tile] > 10'd248)begin reg_next_common <= reg_common_gain[reg_scan_tile] - 10'd1; end
							end
							if(reg_update_phase[reg_scan_tile] == 2'd3 &&
								reg_work_neutral_count[reg_scan_tile] >= C_NEUTRAL_MIN_SAMPLES &&
								reg_work_r[reg_scan_tile] != 22'd0 && reg_work_b[reg_scan_tile] != 22'd0)begin
								state_current <= ST_WB_INIT;
							end
						end
					end
					ST_WB_INIT:begin
						reg_search_goal <= {reg_work_g[reg_scan_tile],8'd0};
						reg_search_channel <= search_input_channel;
						// 常数乘法可展开移位相加；比例搜索无动态除法或乘法。
						reg_search_product <= search_input_channel * C_WB_MIN;
						reg_search_gain <= C_WB_MIN;
						state_current <= ST_WB_ADD;
					end
					ST_WB_ADD:begin
						reg_search_next <= reg_search_product + {8'd0,reg_search_channel};
						reg_search_midpoint <= reg_search_product + (({8'd0,reg_search_channel} + 30'd1) >> 1);
						state_current <= ST_WB_COMPARE;
					end
					ST_WB_COMPARE:begin
						if(reg_search_goal <= reg_search_next || reg_search_gain >= C_WB_MAX - 9'd1)begin
							if(reg_search_blue)begin
								if(reg_next_b < {1'b0,search_target_gain})begin reg_next_b <= reg_next_b + 10'd1;
								end else if(reg_next_b > {1'b0,search_target_gain})begin reg_next_b <= reg_next_b - 10'd1; end
								state_current <= ST_COMMIT;
							end else begin
								if(reg_next_r < {1'b0,search_target_gain})begin reg_next_r <= reg_next_r + 10'd1;
								end else if(reg_next_r > {1'b0,search_target_gain})begin reg_next_r <= reg_next_r - 10'd1; end
								reg_search_blue <= 1'b1;
								state_current <= ST_WB_INIT;
							end
						end else begin
							reg_search_product <= reg_search_next;
							reg_search_gain <= reg_search_gain + 9'd1;
							state_current <= ST_WB_ADD;
						end
					end
					ST_COMMIT:begin
						if(reg_work_valid[reg_scan_tile])begin
							reg_common_gain[reg_scan_tile] <= normalized_base;
							reg_wb_r[reg_scan_tile] <= reg_next_r;
							reg_wb_b[reg_scan_tile] <= reg_next_b;
							reg_auto_config[reg_scan_tile * 128 + 110 +: 10] <= final_gain_r[9:0];
							reg_auto_config[reg_scan_tile * 128 + 100 +: 10] <= normalized_base;
							reg_auto_config[reg_scan_tile * 128 + 90 +: 10] <= final_gain_b[9:0];
							reg_auto_config[reg_scan_tile * 128 + 82 +: 8] <= 8'd0;
							reg_auto_config[reg_scan_tile * 128 + 74 +: 8] <= 8'd0;
							reg_auto_config[reg_scan_tile * 128 + 66 +: 8] <= 8'd0;
							reg_auto_config[reg_scan_tile * 128 + 23 +: 3] <= reg_next_profile;
							reg_auto_config[reg_scan_tile * 128 + 20 +: 3] <= 3'd0;
						end
						if(reg_scan_tile == 2'd3)begin
							reg_status_frame <= reg_work_frame;
							reg_status_epoch <= reg_work_epoch;
							reg_status_valid <= reg_work_valid;
							state_current <= ST_WAIT;
						end else begin
							reg_scan_tile <= reg_scan_tile + 2'd1;
							reg_scan_bin <= 5'd0;
							reg_cumulative <= 15'd0;
							reg_quantile_found <= 4'd0;
							reg_p10 <= 8'd252;
							reg_p50 <= 8'd252;
							reg_p90 <= 8'd252;
							reg_p99 <= 8'd252;
							state_current <= ST_SCAN;
						end
					end
					default:begin state_current <= ST_WAIT; reg_status_valid <= 4'd0; end
				endcase
			end
		end
	end
endmodule
