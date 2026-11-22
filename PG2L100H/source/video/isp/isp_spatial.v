//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 在保持彩色分量的邻域结构中，共享亮度统计完成降噪、脉冲像素替换和有界细节增强。
// Module Name: isp_spatial
// File Name: source/video/isp/isp_spatial.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在保持彩色分量的邻域结构中，共享亮度统计完成降噪、脉冲像素替换和有界细节增强。
// 完整栅格含消隐每拍推进，启用/旁路均固定 C_H_TOTAL+14 拍；帧边界与 tag[156] 一致，使用配置元组隔离跨帧参数。
//
// Dependencies: 官方 isp_line36x2048（同步读、OUTPUT_REG=0）。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: Centered color-preserving ISP neighborhood
// Module Name: isp_spatial
// Project Name: PG2L100H_AVM_ISP
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: II=1; complete raster including blanking advances every clock.
//              Fixed latency C_H_TOTAL+14 clocks, enabled and bypass alike.
//              i_frame_boundary must equal i_tag[156]. Configuration tuples
//              may change at frame boundaries, at most twice per input frame.
// Dependencies: Official isp_line36x2048, synchronous read, OUTPUT_REG=0.
// Revision: 0.01 - Joint luma denoise, impulse replacement and bounded detail.
//           0.02 - Bounded descriptor comparison tree; no latency change.
//           0.03 - 2026-10-29: Latest tuple mirror and local slot enables.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module isp_spatial
#(
	parameter integer C_H_TOTAL = 1650,
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_V_ACTIVE = 720,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320,
	parameter integer C_TOP_BAR = 40
)
(
	input i_clk,
	input i_rstn,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	input i_frame_boundary,
	input [63:0] i_algo_config,
	input [519:0] i_isp_config,
	input [63:0] i_effective_epoch,
	input i_isp_continuous_update,
	input [3:0] i_source_alive,
	input i_source_alive_valid,
	output [23:0] o_rgb,
	output [159:0] o_tag,
	output o_frame_boundary,
	output [63:0] o_algo_config,
	output [519:0] o_isp_config,
	output [63:0] o_effective_epoch,
	output o_isp_continuous_update,
	output [3:0] o_source_alive,
	output o_source_alive_valid
);
	localparam integer LAST_STAGE = 12;
	localparam integer FULL_HISTORY = 2 * C_H_TOTAL + 2;
	localparam integer DESCRIPTOR_BANK_WIDTH = 32;
	localparam integer DESCRIPTOR_BANK_COUNT = (761 + DESCRIPTOR_BANK_WIDTH - 1) / DESCRIPTOR_BANK_WIDTH;
	wire [760:0] descriptor_input;
	wire [760:0] descriptor_current;
	reg [760:0] reg_descriptor_latest;
	wire [95:0] descriptor_difference_byte /* synthesis syn_keep = 1 */;
	wire [23:0] descriptor_difference_group /* synthesis syn_keep = 1 */;
	wire [5:0] descriptor_difference_quarter /* synthesis syn_keep = 1 */;
	wire [1:0] descriptor_difference_half /* synthesis syn_keep = 1 */;
	wire descriptor_changed;
	wire [1:0] descriptor_write_index;
	reg [760:0] reg_descriptor [0:3];
	reg [1:0] reg_descriptor_index;
	reg flag_descriptor_valid;
	wire [760:0] descriptor_center;
	reg [127:0] config_center_tile;
	wire [519:0] config_center;
	wire [35:0] metadata_input;
	wire [15:0] luminance_input;
	wire [35:0] data_input;
	reg [10:0] cnt_address;
	reg [10:0] reg_write_address;
	reg [12:0] cnt_history;
	reg flag_write;
	reg [35:0] reg_data;
	reg [35:0] reg_metadata;
	wire [35:0] line_previous;
	wire [35:0] line_older;
	wire [35:0] line_metadata;
	reg [35:0] reg_top_d1;
	reg [35:0] reg_top_d2;
	reg [35:0] reg_middle_d1;
	reg [35:0] reg_middle_d2;
	reg [35:0] reg_bottom_d1;
	reg [35:0] reg_bottom_d2;
	reg [35:0] reg_metadata_d1;
	wire center_ready;
	wire center_geometry;
	wire [10:0] center_x;
	wire [9:0] center_y;
	wire [9:0] center_tile_x;
	wire [8:0] center_tile_y;
	wire center_border;
	wire center_window_valid;
	wire [35:0] window_data [0:8];
	wire [7:0] window_luma [0:8];
	wire [7:0] blur_luma [0:8];
	wire [7:0] difference_luma [0:8];
	wire [7:0] center_luma;
	wire spatial_enabled;
	reg [35:0] reg_meta_pipe [0:LAST_STAGE];
	reg [23:0] reg_rgb_pipe [0:LAST_STAGE];
	reg [7:0] reg_y_pipe [0:9];
	reg [39:0] reg_parameter_pipe [0:9];
	reg [2:0] reg_enable_pipe [0:9];
	reg flag_stream_pipe [0:LAST_STAGE];
	reg [7:0] reg_sorted [0:9][0:8];
	reg [9:0] reg_blur_row0;
	reg [9:0] reg_blur_row1;
	reg [9:0] reg_blur_row2;
	reg [11:0] reg_blur_sum;
	reg [7:0] reg_blur_pipe [2:9];
	reg [7:0] dec_denoised;
	reg [10:0] dec_blend;
	wire [7:0] median_difference;
	wire impulse_detected;
	wire signed [9:0] detail_difference;
	wire [9:0] detail_absolute;
	reg signed [11:0] dec_detail_scaled;
	reg signed [9:0] reg_base_delta;
	reg signed [11:0] reg_detail_delta;
	reg [7:0] reg_detail_limit;
	reg [7:0] reg_rgb_min;
	reg [7:0] reg_rgb_max;
	wire [7:0] center_min_rg;
	wire [7:0] center_max_rg;
	reg signed [10:0] dec_detail_limited;
	reg signed [11:0] dec_total_delta;
	wire signed [11:0] lower_color_limit;
	wire signed [11:0] upper_color_limit;
	reg signed [9:0] reg_color_delta;
	wire signed [10:0] result_r;
	wire signed [10:0] result_g;
	wire signed [10:0] result_b;
	wire [35:0] metadata_output;
	wire [760:0] descriptor_output;
	wire output_geometry;
	wire [10:0] output_x;
	wire [9:0] output_y;
	wire [9:0] output_tile_x;
	wire [8:0] output_tile_y;
	reg [23:0] rgb_o;
	integer pipeline_stage;
	integer sort_pixel;
	genvar neighbor;
	genvar sort_stage;
	genvar sort_pair;
	genvar descriptor_byte;
	genvar descriptor_group;
	genvar descriptor_quarter;
	genvar descriptor_half;
	genvar descriptor_bank;
	genvar descriptor_slot;

	assign descriptor_input = {i_isp_continuous_update,i_tag[111:0],i_algo_config,i_isp_config,i_effective_epoch};
	// 每沿镜像输入；有变化时槽与镜像同时捕获，无变化时输入本就等于当前槽。
	// 由此逐拍等价于当前槽，同时去掉全局二进制索引驱动761位比较读MUX的长网。
	assign descriptor_current = reg_descriptor_latest;
	// 原759:0布局不变，高位760保存更新原因；仍复用24组32位槽bank。
	// 将761位比较分成原95组8位及高位单bit，避免综合成串行进位链。
	// 保留每层组合边界；写槽、metadata索引与像素拍数保持原契约。
	generate
		for(descriptor_byte = 0; descriptor_byte < 95; descriptor_byte = descriptor_byte + 1)begin:gen_descriptor_byte
			assign descriptor_difference_byte[descriptor_byte] =
				|(descriptor_current[descriptor_byte * 8 +: 8] ^
					descriptor_input[descriptor_byte * 8 +: 8]);
		end
		for(descriptor_group = 0; descriptor_group < 24; descriptor_group = descriptor_group + 1)begin:gen_descriptor_group
			assign descriptor_difference_group[descriptor_group] =
				|descriptor_difference_byte[descriptor_group * 4 +: 4];
		end
		for(descriptor_quarter = 0; descriptor_quarter < 6; descriptor_quarter = descriptor_quarter + 1)begin:gen_descriptor_quarter
			assign descriptor_difference_quarter[descriptor_quarter] =
				|descriptor_difference_group[descriptor_quarter * 4 +: 4];
		end
		for(descriptor_half = 0; descriptor_half < 2; descriptor_half = descriptor_half + 1)begin:gen_descriptor_half
			assign descriptor_difference_half[descriptor_half] =
				|descriptor_difference_quarter[descriptor_half * 3 +: 3];
		end
	endgenerate
	assign descriptor_difference_byte[95] = descriptor_current[760] ^ descriptor_input[760];
	assign descriptor_changed = !flag_descriptor_valid ||
		(|descriptor_difference_half);
	assign descriptor_write_index = descriptor_changed ?
		reg_descriptor_index + 2'd1 : reg_descriptor_index;
	assign metadata_input = {i_tag[159:155],i_tag[154:144],i_tag[143:134],
		i_tag[133:132],i_tag[112],i_source_alive,i_source_alive_valid,descriptor_write_index};
	assign luminance_input = 16'd77 * i_rgb[23:16] + 16'd150 * i_rgb[15:8] +
		16'd29 * i_rgb[7:0] + 16'd128;
	assign data_input = {i_rgb,luminance_input[15:8],i_tag[80],i_tag[112],i_tag[133:132]};
	assign center_ready = cnt_history >= C_H_TOTAL + 2;
	assign center_x = reg_metadata_d1[30:20];
	assign center_y = reg_metadata_d1[19:10];
	assign center_geometry = reg_metadata_d1[33] && center_x < C_H_ACTIVE &&
		center_y >= C_TOP_BAR && center_y < C_TOP_BAR + 2 * C_TILE_HEIGHT;
	assign center_tile_x = center_geometry ?
		(center_x < C_TILE_WIDTH ? center_x : center_x - C_TILE_WIDTH) : 10'd0;
	assign center_tile_y = center_geometry ?
		(center_y < C_TOP_BAR + C_TILE_HEIGHT ? center_y - C_TOP_BAR :
		center_y - C_TOP_BAR - C_TILE_HEIGHT) : 9'd0;
	assign center_border = !center_geometry || center_tile_x == 0 ||
		center_tile_x == C_TILE_WIDTH - 1 || center_tile_y == 0 ||
		center_tile_y == C_TILE_HEIGHT - 1;
	assign center_window_valid = center_ready && cnt_history >= FULL_HISTORY &&
		reg_metadata_d1[7] && !center_border;
	assign descriptor_center = reg_descriptor[reg_metadata_d1[1:0]];
	assign config_center = descriptor_center[583:64];
	// Explicitly select the tile slice.  This keeps the RTL Verilog-2001
	// compatible and gives synthesis a bounded four-way mux instead of a
	// variable indexed part-select.
	always@(*)begin
		case(reg_metadata_d1[9:8])
			2'd0: config_center_tile = config_center[127:0];
			2'd1: config_center_tile = config_center[255:128];
			2'd2: config_center_tile = config_center[383:256];
			2'd3: config_center_tile = config_center[511:384];
			default: config_center_tile = 128'd0;
		endcase
	end
	assign spatial_enabled = center_window_valid && config_center[512] &&
		config_center[513 + reg_metadata_d1[9:8]];
	assign center_luma = center_ready ? reg_middle_d1[11:4] : 8'd0;
	assign window_data[0] = reg_top_d2;
	assign window_data[1] = reg_top_d1;
	assign window_data[2] = line_older;
	assign window_data[3] = reg_middle_d2;
	assign window_data[4] = reg_middle_d1;
	assign window_data[5] = line_previous;
	assign window_data[6] = reg_bottom_d2;
	assign window_data[7] = reg_bottom_d1;
	assign window_data[8] = reg_data;

	// Same tile, content and frame parity are required for every neighbor.
	// The history spans only two lines, so parity cannot alias a valid frame.
	generate
		for(neighbor = 0; neighbor < 9; neighbor = neighbor + 1)begin:gen_neighbor
			assign window_luma[neighbor] = center_window_valid &&
				window_data[neighbor][3:0] == reg_middle_d1[3:0] ?
				window_data[neighbor][11:4] : center_luma;
			assign difference_luma[neighbor] = window_luma[neighbor] >= center_luma ?
				window_luma[neighbor] - center_luma : center_luma - window_luma[neighbor];
			assign blur_luma[neighbor] = config_center_tile[65:64] == 2'd1 &&
				difference_luma[neighbor] > config_center_tile[60:53] ?
				center_luma : window_luma[neighbor];
		end
	endgenerate

	// 比较镜像没有CE，不进入像素流水；槽索引、metadata和全部输出仍在原沿切换。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_descriptor_latest <= 761'd0;
		end else begin
			reg_descriptor_latest <= descriptor_input;
		end
	end

	// 四槽保留SOF旧tag与新配置；每32位用本地同沿热码生成常量槽CE。
	// 热码表示当前槽，写入选择它的下一槽；与二进制索引同沿旋转，不增加拍数。
	generate
		for(descriptor_bank = 0; descriptor_bank < DESCRIPTOR_BANK_COUNT;
			descriptor_bank = descriptor_bank + 1)begin:gen_descriptor_bank
			localparam integer BANK_FIRST = descriptor_bank * DESCRIPTOR_BANK_WIDTH;
			localparam integer BANK_WIDTH = 761 - BANK_FIRST < DESCRIPTOR_BANK_WIDTH ?
				761 - BANK_FIRST : DESCRIPTOR_BANK_WIDTH;
			(* syn_preserve = 1 *) reg [3:0] reg_current_onehot;

			always@(posedge i_clk or negedge i_rstn)begin
				if(!i_rstn)begin
					reg_current_onehot <= 4'b0001;
				end else if(descriptor_changed)begin
					reg_current_onehot <= {reg_current_onehot[2:0],reg_current_onehot[3]};
				end
			end
			for(descriptor_slot = 0; descriptor_slot < 4;
				descriptor_slot = descriptor_slot + 1)begin:gen_descriptor_slot
				localparam integer PREVIOUS_SLOT = (descriptor_slot + 3) % 4;
				always@(posedge i_clk or negedge i_rstn)begin
					if(!i_rstn)begin
						reg_descriptor[descriptor_slot][BANK_FIRST +: BANK_WIDTH] <= {BANK_WIDTH{1'b0}};
					end else if(descriptor_changed && reg_current_onehot[PREVIOUS_SLOT])begin
						reg_descriptor[descriptor_slot][BANK_FIRST +: BANK_WIDTH] <=
							descriptor_input[BANK_FIRST +: BANK_WIDTH];
					end
				end
			end
		end
	endgenerate

	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_descriptor_index <= 2'd0;
			flag_descriptor_valid <= 1'b0;
		end else if(descriptor_changed)begin
			reg_descriptor_index <= descriptor_write_index;
			flag_descriptor_valid <= 1'b1;
		end
	end

	// Write the preceding address while reading the current address: no collisions.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_address <= 11'd0;
			reg_write_address <= 11'd0;
			cnt_history <= 13'd0;
			flag_write <= 1'b0;
			reg_data <= 36'd0;
			reg_metadata <= 36'd0;
		end else begin
			cnt_address <= cnt_address == C_H_TOTAL - 1 ? 11'd0 : cnt_address + 11'd1;
			reg_write_address <= cnt_address;
			if(cnt_history < FULL_HISTORY)begin
				cnt_history <= cnt_history + 13'd1;
			end
			flag_write <= 1'b1;
			reg_data <= data_input;
			reg_metadata <= metadata_input;
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_top_d1 <= 0;
			reg_top_d2 <= 0;
			reg_middle_d1 <= 0;
			reg_middle_d2 <= 0;
			reg_bottom_d1 <= 0;
			reg_bottom_d2 <= 0;
			reg_metadata_d1 <= 0;
		end else begin
			reg_top_d1 <= line_older;
			reg_top_d2 <= reg_top_d1;
			reg_middle_d1 <= line_previous;
			reg_middle_d2 <= reg_middle_d1;
			reg_bottom_d1 <= reg_data;
			reg_bottom_d2 <= reg_bottom_d1;
			reg_metadata_d1 <= line_metadata;
		end
	end

	// Stage 0 captures the centered window; rows sum independently for timing.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_blur_row0 <= 0;
			reg_blur_row1 <= 0;
			reg_blur_row2 <= 0;
			reg_blur_sum <= 0;
			for(pipeline_stage = 2; pipeline_stage <= 9; pipeline_stage = pipeline_stage + 1)begin
				reg_blur_pipe[pipeline_stage] <= 0;
			end
		end else begin
			reg_blur_row0 <= {2'd0,blur_luma[0]} + {1'd0,blur_luma[1],1'b0} + {2'd0,blur_luma[2]};
			reg_blur_row1 <= {2'd0,blur_luma[3]} + {1'd0,blur_luma[4],1'b0} + {2'd0,blur_luma[5]};
			reg_blur_row2 <= {2'd0,blur_luma[6]} + {1'd0,blur_luma[7],1'b0} + {2'd0,blur_luma[8]};
			reg_blur_sum <= {2'd0,reg_blur_row0} + {1'd0,reg_blur_row1,1'b0} + {2'd0,reg_blur_row2} + 12'd8;
			reg_blur_pipe[2] <= reg_blur_sum[11:4];
			for(pipeline_stage = 3; pipeline_stage <= 9; pipeline_stage = pipeline_stage + 1)begin
				reg_blur_pipe[pipeline_stage] <= reg_blur_pipe[pipeline_stage - 1];
			end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			for(sort_pixel = 0; sort_pixel < 9; sort_pixel = sort_pixel + 1)begin
				reg_sorted[0][sort_pixel] <= 0;
			end
		end else begin
			for(sort_pixel = 0; sort_pixel < 9; sort_pixel = sort_pixel + 1)begin
				reg_sorted[0][sort_pixel] <= window_luma[sort_pixel];
			end
		end
	end
	// Nine odd/even transposition stages have one comparator per register path.
	generate
		for(sort_stage = 1; sort_stage <= 9; sort_stage = sort_stage + 1)begin:gen_sort_stage
			for(sort_pair = 0; sort_pair < 9; sort_pair = sort_pair + 1)begin:gen_sort_pixel
				if((sort_stage % 2 == 1 && sort_pair == 8) ||
					(sort_stage % 2 == 0 && sort_pair == 0))begin:gen_unchanged
					always@(posedge i_clk or negedge i_rstn)begin
						if(!i_rstn)begin reg_sorted[sort_stage][sort_pair] <= 0; end
						else begin reg_sorted[sort_stage][sort_pair] <= reg_sorted[sort_stage - 1][sort_pair]; end
					end
				end else if((sort_stage + sort_pair) % 2 == 1)begin:gen_minimum
					always@(posedge i_clk or negedge i_rstn)begin
						if(!i_rstn)begin reg_sorted[sort_stage][sort_pair] <= 0; end
						else if(reg_sorted[sort_stage - 1][sort_pair] <=
							reg_sorted[sort_stage - 1][sort_pair + 1])begin
							reg_sorted[sort_stage][sort_pair] <= reg_sorted[sort_stage - 1][sort_pair];
						end else begin
							reg_sorted[sort_stage][sort_pair] <= reg_sorted[sort_stage - 1][sort_pair + 1];
						end
					end
				end else begin:gen_maximum
					always@(posedge i_clk or negedge i_rstn)begin
						if(!i_rstn)begin reg_sorted[sort_stage][sort_pair] <= 0; end
						else if(reg_sorted[sort_stage - 1][sort_pair] >=
							reg_sorted[sort_stage - 1][sort_pair - 1])begin
							reg_sorted[sort_stage][sort_pair] <= reg_sorted[sort_stage - 1][sort_pair];
						end else begin
							reg_sorted[sort_stage][sort_pair] <= reg_sorted[sort_stage - 1][sort_pair - 1];
						end
					end
				end
			end
		end
	endgenerate

	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			for(pipeline_stage = 0; pipeline_stage <= LAST_STAGE; pipeline_stage = pipeline_stage + 1)begin
				reg_meta_pipe[pipeline_stage] <= 0;
				reg_rgb_pipe[pipeline_stage] <= 0;
				flag_stream_pipe[pipeline_stage] <= 0;
			end
			for(pipeline_stage = 0; pipeline_stage <= 9; pipeline_stage = pipeline_stage + 1)begin
				reg_y_pipe[pipeline_stage] <= 0;
				reg_parameter_pipe[pipeline_stage] <= 0;
				reg_enable_pipe[pipeline_stage] <= 0;
			end
		end else begin
			reg_meta_pipe[0] <= center_ready ? reg_metadata_d1 : 36'd0;
			reg_rgb_pipe[0] <= center_ready ? reg_middle_d1[35:12] : 24'd0;
			flag_stream_pipe[0] <= center_ready;
			reg_y_pipe[0] <= center_luma;
			reg_parameter_pipe[0] <= config_center_tile[65:26];
			reg_enable_pipe[0] <= spatial_enabled ? config_center_tile[123:121] : 3'd0;
			for(pipeline_stage = 1; pipeline_stage <= LAST_STAGE; pipeline_stage = pipeline_stage + 1)begin
				reg_meta_pipe[pipeline_stage] <= reg_meta_pipe[pipeline_stage - 1];
				reg_rgb_pipe[pipeline_stage] <= reg_rgb_pipe[pipeline_stage - 1];
				flag_stream_pipe[pipeline_stage] <= flag_stream_pipe[pipeline_stage - 1];
			end
			for(pipeline_stage = 1; pipeline_stage <= 9; pipeline_stage = pipeline_stage + 1)begin
				reg_y_pipe[pipeline_stage] <= reg_y_pipe[pipeline_stage - 1];
				reg_parameter_pipe[pipeline_stage] <= reg_parameter_pipe[pipeline_stage - 1];
				reg_enable_pipe[pipeline_stage] <= reg_enable_pipe[pipeline_stage - 1];
			end
		end
	end

	// Parameter slice [65:26]: denoise strength[37:35], threshold[34:27],
	// median threshold[26:19], sharp strength[18:16], noise[15:8], limit[7:0].
	assign median_difference = reg_y_pipe[9] >= reg_sorted[9][4] ?
		reg_y_pipe[9] - reg_sorted[9][4] : reg_sorted[9][4] - reg_y_pipe[9];
	assign impulse_detected = reg_enable_pipe[9][1] &&
		median_difference > reg_parameter_pipe[9][26:19];
	assign detail_difference = $signed({2'd0,reg_y_pipe[9]}) - $signed({2'd0,reg_blur_pipe[9]});
	assign detail_absolute = detail_difference < 0 ? -detail_difference : detail_difference;
	always@(*)begin
		dec_blend = {3'd0,reg_y_pipe[9]};
		dec_denoised = reg_y_pipe[9];
		case(reg_parameter_pipe[9][37:35])
			3'd1:begin
				dec_blend = {3'd0,reg_y_pipe[9]} + {2'd0,reg_y_pipe[9],1'b0} + {3'd0,reg_blur_pipe[9]};
				dec_denoised = dec_blend[9:2];
			end
			3'd2:begin
				dec_blend = {3'd0,reg_y_pipe[9]} + {3'd0,reg_blur_pipe[9]};
				dec_denoised = dec_blend[8:1];
			end
			3'd3:begin
				dec_blend = {3'd0,reg_y_pipe[9]} + {3'd0,reg_blur_pipe[9]} + {2'd0,reg_blur_pipe[9],1'b0};
				dec_denoised = dec_blend[9:2];
			end
			3'd4:begin dec_denoised = reg_blur_pipe[9]; end
			default:begin end
		endcase
		dec_detail_scaled = 12'sd0;
		if(reg_enable_pipe[9][2] && !impulse_detected &&
			detail_absolute > {2'd0,reg_parameter_pipe[9][15:8]})begin
			case(reg_parameter_pipe[9][18:16])
				3'd1:begin dec_detail_scaled = $signed({{2{detail_difference[9]}},detail_difference}) >>> 2; end
				3'd2:begin dec_detail_scaled = $signed({{2{detail_difference[9]}},detail_difference}) >>> 1; end
				3'd3:begin dec_detail_scaled = ($signed({{2{detail_difference[9]}},detail_difference}) +
					($signed({{2{detail_difference[9]}},detail_difference}) <<< 1)) >>> 2; end
				3'd4:begin dec_detail_scaled = $signed({{2{detail_difference[9]}},detail_difference}); end
				default:begin end
			endcase
		end
	end
	assign center_min_rg = reg_rgb_pipe[9][23:16] <= reg_rgb_pipe[9][15:8] ?
		reg_rgb_pipe[9][23:16] : reg_rgb_pipe[9][15:8];
	assign center_max_rg = reg_rgb_pipe[9][23:16] >= reg_rgb_pipe[9][15:8] ?
		reg_rgb_pipe[9][23:16] : reg_rgb_pipe[9][15:8];
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_base_delta <= 0;
			reg_detail_delta <= 0;
			reg_detail_limit <= 0;
			reg_rgb_min <= 0;
			reg_rgb_max <= 0;
		end else begin
			if(impulse_detected)begin
				reg_base_delta <= $signed({2'd0,reg_sorted[9][4]}) - $signed({2'd0,reg_y_pipe[9]});
			end else if(reg_enable_pipe[9][0])begin
				reg_base_delta <= $signed({2'd0,dec_denoised}) - $signed({2'd0,reg_y_pipe[9]});
			end else begin
				reg_base_delta <= 10'sd0;
			end
			reg_detail_delta <= dec_detail_scaled;
			reg_detail_limit <= reg_parameter_pipe[9][7:0];
			reg_rgb_min <= center_min_rg <= reg_rgb_pipe[9][7:0] ? center_min_rg : reg_rgb_pipe[9][7:0];
			reg_rgb_max <= center_max_rg >= reg_rgb_pipe[9][7:0] ? center_max_rg : reg_rgb_pipe[9][7:0];
		end
	end
	assign lower_color_limit = -$signed({4'd0,reg_rgb_min});
	assign upper_color_limit = 12'sd255 - $signed({4'd0,reg_rgb_max});
	always@(*)begin
		dec_detail_limited = reg_detail_delta[10:0];
		if(reg_detail_delta > $signed({4'd0,reg_detail_limit}))begin
			dec_detail_limited = $signed({3'd0,reg_detail_limit});
		end else if(reg_detail_delta < -$signed({4'd0,reg_detail_limit}))begin
			dec_detail_limited = -$signed({3'd0,reg_detail_limit});
		end
		dec_total_delta = $signed({{2{reg_base_delta[9]}},reg_base_delta}) +
			$signed({{1{dec_detail_limited[10]}},dec_detail_limited});
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_color_delta <= 0;
		end else if(dec_total_delta < lower_color_limit)begin
			reg_color_delta <= lower_color_limit[9:0];
		end else if(dec_total_delta > upper_color_limit)begin
			reg_color_delta <= upper_color_limit[9:0];
		end else begin
			reg_color_delta <= dec_total_delta[9:0];
		end
	end
	assign result_r = $signed({3'd0,reg_rgb_pipe[11][23:16]}) + $signed({reg_color_delta[9],reg_color_delta});
	assign result_g = $signed({3'd0,reg_rgb_pipe[11][15:8]}) + $signed({reg_color_delta[9],reg_color_delta});
	assign result_b = $signed({3'd0,reg_rgb_pipe[11][7:0]}) + $signed({reg_color_delta[9],reg_color_delta});
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			rgb_o <= 24'd0;
		end else begin
			rgb_o <= {result_r[7:0],result_g[7:0],result_b[7:0]};
		end
	end

	assign metadata_output = reg_meta_pipe[LAST_STAGE];
	assign descriptor_output = reg_descriptor[metadata_output[1:0]];
	assign output_x = metadata_output[30:20];
	assign output_y = metadata_output[19:10];
	assign output_geometry = metadata_output[33] && output_x < C_H_ACTIVE &&
		output_y >= C_TOP_BAR && output_y < C_TOP_BAR + 2 * C_TILE_HEIGHT;
	assign output_tile_x = output_geometry ?
		(output_x < C_TILE_WIDTH ? output_x : output_x - C_TILE_WIDTH) : 10'd0;
	assign output_tile_y = output_geometry ?
		(output_y < C_TOP_BAR + C_TILE_HEIGHT ? output_y - C_TOP_BAR :
		output_y - C_TOP_BAR - C_TILE_HEIGHT) : 9'd0;
	assign o_rgb = flag_stream_pipe[LAST_STAGE] ? rgb_o : 24'd0;
	assign o_tag = flag_stream_pipe[LAST_STAGE] ? {metadata_output[35:31],
		output_x,output_y,metadata_output[9:8],output_tile_x,output_tile_y,
		metadata_output[7],descriptor_output[759:648]} : 160'd0;
	assign o_frame_boundary = flag_stream_pipe[LAST_STAGE] && metadata_output[32];
	assign o_algo_config = flag_stream_pipe[LAST_STAGE] ? descriptor_output[647:584] : 64'd0;
	assign o_isp_config = flag_stream_pipe[LAST_STAGE] ? descriptor_output[583:64] : 520'd0;
	assign o_effective_epoch = flag_stream_pipe[LAST_STAGE] ? descriptor_output[63:0] : 64'd0;
	assign o_isp_continuous_update = flag_stream_pipe[LAST_STAGE] && descriptor_output[760];
	assign o_source_alive = flag_stream_pipe[LAST_STAGE] ? metadata_output[6:3] : 4'd0;
	assign o_source_alive_valid = flag_stream_pipe[LAST_STAGE] && metadata_output[2];

	isp_line36x2048 isp_line36x2048_Inst_previous(
		.wr_data(reg_data),.wr_addr(reg_write_address),.wr_en(flag_write && i_rstn),
		.wr_clk(i_clk),.wr_rst(!i_rstn),.rd_addr(cnt_address),.rd_data(line_previous),
		.rd_clk(i_clk),.rd_clk_en(i_rstn),.rd_rst(!i_rstn));
	isp_line36x2048 isp_line36x2048_Inst_older(
		.wr_data(line_previous),.wr_addr(reg_write_address),.wr_en(flag_write && i_rstn),
		.wr_clk(i_clk),.wr_rst(!i_rstn),.rd_addr(cnt_address),.rd_data(line_older),
		.rd_clk(i_clk),.rd_clk_en(i_rstn),.rd_rst(!i_rstn));
	isp_line36x2048 isp_line36x2048_Inst_metadata(
		.wr_data(reg_metadata),.wr_addr(reg_write_address),.wr_en(flag_write && i_rstn),
		.wr_clk(i_clk),.wr_rst(!i_rstn),.rd_addr(cnt_address),.rd_data(line_metadata),
		.rd_clk(i_clk),.rd_clk_en(i_rstn),.rd_rst(!i_rstn));
endmodule
