//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 兼容手动逐通道 LUT 与自动亮度共同修正的色调、饱和度流水。
// Module Name: isp_tone_sat_stage
// File Name: source/video/isp/isp_tone_sat_stage.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// tile bit126=1 时按 Y601 查表，RGB 加同一受限 delta；bit126=0 保留旧逐通道行为。
// 正 delta 受 247-maxRGB 限制，负 delta 受 minRGB 限制，避免新增通道饱和。
// 消隐及无效内容旁路，d0..d5 六个采样级，像素和所有侧带保持一致。
//
// Dependencies: 官方 isp_tone_lut_2048x8；assets/isp/tone_profiles.dat 初始化数据。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-16 / v025 - 新增自然亮度查表与共同 delta，保持原接口、吞吐和延迟。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: ISP tone and saturation stage
// Module Name: isp_tone_sat_stage
// Project Name: PG2L100H_AVM_ISP
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Three parallel tone-LUT paths followed by bounded saturation.
//              The ROM uses {profile,pixel} addresses. Tone and saturation
//              are bypassed for blank/invalid pixels and all sideband signals
//              retain a fixed five-clock latency including the ROM read phase.
// Dependencies: Official isp_tone_lut_2048x8 ROM IP initialized by tone_profiles.dat.
// Revision: 0.01 - 2026-10-19, initial implementation.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module isp_tone_sat_stage
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
	reg [23:0] reg_rgb_d0;
	reg [7:0] reg_luma_d0;
	reg reg_natural_tone_d0;
	reg [159:0] reg_tag_d0;
	reg reg_frame_d0;
	reg [63:0] reg_algo_d0;
	reg [519:0] reg_isp_d0;
	reg [63:0] reg_epoch_d0;
	reg reg_continuous_d0;
	reg [3:0] reg_alive_d0;
	reg reg_alive_valid_d0;
	reg reg_saturation_enable_d0;
	reg reg_tone_enable_d0;
	reg [2:0] reg_profile_d0;
	reg [2:0] reg_saturation_d0;
	reg reg_stage_enable_d0;
	reg [23:0] reg_raw_rgb_d1;
	reg [7:0] reg_luma_d1;
	reg [7:0] reg_min_rgb_d1;
	reg [7:0] reg_max_rgb_d1;
	reg reg_natural_tone_d1;
	reg reg_tone_enable_d1;
	reg reg_saturation_enable_d1;
	reg reg_stage_enable_d1;
	reg [2:0] reg_saturation_d1;
	reg [159:0] reg_tag_d1;
	reg reg_frame_d1;
	reg [63:0] reg_algo_d1;
	reg [519:0] reg_isp_d1;
	reg [63:0] reg_epoch_d1;
	reg reg_continuous_d1;
	reg [3:0] reg_alive_d1;
	reg reg_alive_valid_d1;
	reg [23:0] reg_tone_rgb_d2;
	reg reg_saturation_enable_d2;
	reg reg_stage_enable_d2;
	reg [2:0] reg_saturation_d2;
	reg [159:0] reg_tag_d2;
	reg reg_frame_d2;
	reg [63:0] reg_algo_d2;
	reg [519:0] reg_isp_d2;
	reg [63:0] reg_epoch_d2;
	reg reg_continuous_d2;
	reg [3:0] reg_alive_d2;
	reg reg_alive_valid_d2;
	// d3 registers isolate the three constant luma terms from the RGB/LUT path.
	reg [23:0] reg_tone_rgb_d3;
	reg [15:0] reg_luma_term_r_d3;
	reg [15:0] reg_luma_term_g_d3;
	reg [15:0] reg_luma_term_b_d3;
	reg reg_saturation_enable_d3;
	reg reg_stage_enable_d3;
	reg [2:0] reg_saturation_d3;
	reg [159:0] reg_tag_d3;
	reg reg_frame_d3;
	reg [63:0] reg_algo_d3;
	reg [519:0] reg_isp_d3;
	reg [63:0] reg_epoch_d3;
	reg reg_continuous_d3;
	reg [3:0] reg_alive_d3;
	reg reg_alive_valid_d3;
	// d4 registers isolate luma/chroma arithmetic from the final clamp.
	reg [23:0] reg_tone_rgb_d4;
	reg [7:0] reg_luma_value_d4;
	reg signed [9:0] reg_chroma_r_d4;
	reg signed [9:0] reg_chroma_g_d4;
	reg signed [9:0] reg_chroma_b_d4;
	reg reg_saturation_enable_d4;
	reg reg_stage_enable_d4;
	reg [2:0] reg_saturation_d4;
	reg [159:0] reg_tag_d4;
	reg reg_frame_d4;
	reg [63:0] reg_algo_d4;
	reg [519:0] reg_isp_d4;
	reg [63:0] reg_epoch_d4;
	reg reg_continuous_d4;
	reg [3:0] reg_alive_d4;
	reg reg_alive_valid_d4;
	reg [159:0] reg_tag_d5;
	reg reg_frame_d5;
	reg [63:0] reg_algo_d5;
	reg [519:0] reg_isp_d5;
	reg [63:0] reg_epoch_d5;
	reg reg_continuous_d5;
	reg [3:0] reg_alive_d5;
	reg reg_alive_valid_d5;
	// Debug-visible arithmetic values retained for the existing smoke test.
	reg [15:0] luma_sum;
	reg [7:0] luma_value;
	reg signed [9:0] chroma_r;
	reg signed [9:0] chroma_g;
	reg signed [9:0] chroma_b;
	reg signed [10:0] saturation_coeff;
	reg signed [21:0] saturation_product_r;
	reg signed [21:0] saturation_product_g;
	reg signed [21:0] saturation_product_b;
	reg signed [19:0] saturation_value_r;
	reg signed [19:0] saturation_value_g;
	reg signed [19:0] saturation_value_b;
	reg signed [13:0] scaled_chroma_r;
	reg signed [13:0] scaled_chroma_g;
	reg signed [13:0] scaled_chroma_b;
	reg signed [13:0] chroma_ext_r;
	reg signed [13:0] chroma_ext_g;
	reg signed [13:0] chroma_ext_b;
	reg signed [19:0] rounded_chroma_r;
	reg signed [19:0] rounded_chroma_g;
	reg signed [19:0] rounded_chroma_b;
	reg [7:0] saturated_r;
	reg [7:0] saturated_g;
	reg [7:0] saturated_b;
	reg [23:0] rgb_o;

	wire [1:0] input_tile = i_tag[133:132];
	reg [127:0] input_tile_config;
	wire input_content_valid = i_tag[157] && i_tag[112];
	wire input_stage_enable = input_content_valid && i_isp_config[512] &&
		i_isp_config[513 + input_tile];
	always@(*)begin
		case(input_tile)
			2'd0: input_tile_config = i_isp_config[127:0];
			2'd1: input_tile_config = i_isp_config[255:128];
			2'd2: input_tile_config = i_isp_config[383:256];
			2'd3: input_tile_config = i_isp_config[511:384];
			default: input_tile_config = 128'd0;
		endcase
	end
	wire [15:0] input_luma_sum = 16'd77 * i_rgb[23:16] +
		16'd150 * i_rgb[15:8] + 16'd29 * i_rgb[7:0] + 16'd128;
	wire [7:0] raw_min_rg = (reg_rgb_d0[23:16] < reg_rgb_d0[15:8]) ?
		reg_rgb_d0[23:16] : reg_rgb_d0[15:8];
	wire [7:0] raw_max_rg = (reg_rgb_d0[23:16] > reg_rgb_d0[15:8]) ?
		reg_rgb_d0[23:16] : reg_rgb_d0[15:8];
	wire [10:0] tone_addr_r = {reg_profile_d0,
		(reg_natural_tone_d0 ? reg_luma_d0 : reg_rgb_d0[23:16])};
	wire [10:0] tone_addr_g = {reg_profile_d0,reg_rgb_d0[15:8]};
	wire [10:0] tone_addr_b = {reg_profile_d0,reg_rgb_d0[7:0]};
	wire [7:0] tone_data_r;
	wire [7:0] tone_data_g;
	wire [7:0] tone_data_b;
	reg signed [8:0] natural_delta;
	reg signed [8:0] natural_lower_bound;
	reg signed [8:0] natural_upper_bound;
	reg signed [9:0] natural_r;
	reg signed [9:0] natural_g;
	reg signed [9:0] natural_b;

	// 自动模式保持三通道差值；输入已接近饱和时不继续向白端提升。
	always@(*)begin
		natural_delta = $signed({1'b0,tone_data_r}) - $signed({1'b0,reg_luma_d1});
		natural_lower_bound = -$signed({1'b0,reg_min_rgb_d1});
		if(reg_max_rgb_d1 < 8'd247)begin
			natural_upper_bound = $signed(9'd247) - $signed({1'b0,reg_max_rgb_d1});
		end else begin
			natural_upper_bound = 9'sd0;
		end
		if(natural_delta < natural_lower_bound)begin
			natural_delta = natural_lower_bound;
		end else if(natural_delta > natural_upper_bound)begin
			natural_delta = natural_upper_bound;
		end
		natural_r = $signed({2'b00,reg_raw_rgb_d1[23:16]}) +
			$signed({natural_delta[8],natural_delta});
		natural_g = $signed({2'b00,reg_raw_rgb_d1[15:8]}) +
			$signed({natural_delta[8],natural_delta});
		natural_b = $signed({2'b00,reg_raw_rgb_d1[7:0]}) +
			$signed({natural_delta[8],natural_delta});
	end

	isp_tone_lut_2048x8 isp_tone_lut_2048x8_Inst_r(
		.addr(tone_addr_r), .rd_data(tone_data_r), .clk(i_clk), .rst(!i_rstn));
	isp_tone_lut_2048x8 isp_tone_lut_2048x8_Inst_g(
		.addr(tone_addr_g), .rd_data(tone_data_g), .clk(i_clk), .rst(!i_rstn));
	isp_tone_lut_2048x8 isp_tone_lut_2048x8_Inst_b(
		.addr(tone_addr_b), .rd_data(tone_data_b), .clk(i_clk), .rst(!i_rstn));

	// Stage d3: constant luma products are registered independently.  This
	// removes the three-channel multiply/add tree from the saturation path.
	always@(*)begin
		luma_sum = {4'd0,reg_luma_term_r_d3} +
			{4'd0,reg_luma_term_g_d3} + {4'd0,reg_luma_term_b_d3} + 16'd128;
		luma_value = reg_luma_value_d4;
		chroma_r = reg_chroma_r_d4;
		chroma_g = reg_chroma_g_d4;
		chroma_b = reg_chroma_b_d4;
		chroma_ext_r = $signed({{4{chroma_r[9]}},chroma_r});
		chroma_ext_g = $signed({{4{chroma_g[9]}},chroma_g});
		chroma_ext_b = $signed({{4{chroma_b[9]}},chroma_b});
		saturation_coeff = 11'sd256;
		if(reg_saturation_enable_d4 && reg_stage_enable_d4)begin
			case(reg_saturation_d4)
				3'd1: saturation_coeff = 11'sd192;
				3'd2: saturation_coeff = 11'sd224;
				3'd3: saturation_coeff = 11'sd288;
				3'd4: saturation_coeff = 11'sd320;
				default: saturation_coeff = 11'sd256;
			endcase
		end
		// For k={192,224,256,288,320}/256, the exact old expression
		// Y + ((C-Y)*k + 128) >>> 8 is represented by a small shift/add.
		// The added 2 or 4 is the same +128 rounding term after reduction.
		case(reg_saturation_d4)
			3'd1:begin
				scaled_chroma_r = chroma_ext_r + (chroma_ext_r <<< 1);
				scaled_chroma_g = chroma_ext_g + (chroma_ext_g <<< 1);
				scaled_chroma_b = chroma_ext_b + (chroma_ext_b <<< 1);
				rounded_chroma_r = (scaled_chroma_r + 14'sd2) >>> 2;
				rounded_chroma_g = (scaled_chroma_g + 14'sd2) >>> 2;
				rounded_chroma_b = (scaled_chroma_b + 14'sd2) >>> 2;
			end
			3'd2:begin
				scaled_chroma_r = chroma_ext_r + (chroma_ext_r <<< 1) + (chroma_ext_r <<< 2);
				scaled_chroma_g = chroma_ext_g + (chroma_ext_g <<< 1) + (chroma_ext_g <<< 2);
				scaled_chroma_b = chroma_ext_b + (chroma_ext_b <<< 1) + (chroma_ext_b <<< 2);
				rounded_chroma_r = (scaled_chroma_r + 14'sd4) >>> 3;
				rounded_chroma_g = (scaled_chroma_g + 14'sd4) >>> 3;
				rounded_chroma_b = (scaled_chroma_b + 14'sd4) >>> 3;
			end
			3'd3:begin
				scaled_chroma_r = chroma_ext_r + (chroma_ext_r <<< 3);
				scaled_chroma_g = chroma_ext_g + (chroma_ext_g <<< 3);
				scaled_chroma_b = chroma_ext_b + (chroma_ext_b <<< 3);
				rounded_chroma_r = (scaled_chroma_r + 14'sd4) >>> 3;
				rounded_chroma_g = (scaled_chroma_g + 14'sd4) >>> 3;
				rounded_chroma_b = (scaled_chroma_b + 14'sd4) >>> 3;
			end
			3'd4:begin
				scaled_chroma_r = chroma_ext_r + (chroma_ext_r <<< 2);
				scaled_chroma_g = chroma_ext_g + (chroma_ext_g <<< 2);
				scaled_chroma_b = chroma_ext_b + (chroma_ext_b <<< 2);
				rounded_chroma_r = (scaled_chroma_r + 14'sd2) >>> 2;
				rounded_chroma_g = (scaled_chroma_g + 14'sd2) >>> 2;
				rounded_chroma_b = (scaled_chroma_b + 14'sd2) >>> 2;
			end
			default:begin
				scaled_chroma_r = chroma_ext_r;
				scaled_chroma_g = chroma_ext_g;
				scaled_chroma_b = chroma_ext_b;
				rounded_chroma_r = chroma_ext_r;
				rounded_chroma_g = chroma_ext_g;
				rounded_chroma_b = chroma_ext_b;
			end
		endcase
		saturation_product_r = scaled_chroma_r <<< 6;
		saturation_product_g = scaled_chroma_g <<< 6;
		saturation_product_b = scaled_chroma_b <<< 6;
		saturation_value_r = $signed({1'b0,reg_luma_value_d4}) + rounded_chroma_r;
		saturation_value_g = $signed({1'b0,reg_luma_value_d4}) + rounded_chroma_g;
		saturation_value_b = $signed({1'b0,reg_luma_value_d4}) + rounded_chroma_b;
		if(saturation_value_r < 0)begin saturated_r = 8'd0; end
		else if(saturation_value_r > 255)begin saturated_r = 8'd255; end
		else begin saturated_r = saturation_value_r[7:0]; end
		if(saturation_value_g < 0)begin saturated_g = 8'd0; end
		else if(saturation_value_g > 255)begin saturated_g = 8'd255; end
		else begin saturated_g = saturation_value_g[7:0]; end
		if(saturation_value_b < 0)begin saturated_b = 8'd0; end
		else if(saturation_value_b > 255)begin saturated_b = 8'd255; end
		else begin saturated_b = saturation_value_b[7:0]; end
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_rgb_d0 <= 24'd0;
			reg_luma_d0 <= 8'd0;
			reg_natural_tone_d0 <= 1'b0;
			reg_tag_d0 <= 160'd0;
			reg_frame_d0 <= 1'b0;
			reg_algo_d0 <= 64'd0;
			reg_isp_d0 <= 520'd0;
			reg_epoch_d0 <= 64'd0;
			reg_continuous_d0 <= 1'b0;
			reg_alive_d0 <= 4'd0;
			reg_alive_valid_d0 <= 1'b0;
			reg_saturation_enable_d0 <= 1'b0;
			reg_tone_enable_d0 <= 1'b0;
			reg_profile_d0 <= 3'd0;
			reg_saturation_d0 <= 3'd0;
			reg_stage_enable_d0 <= 1'b0;
			reg_raw_rgb_d1 <= 24'd0;
			reg_luma_d1 <= 8'd0;
			reg_min_rgb_d1 <= 8'd0;
			reg_max_rgb_d1 <= 8'd0;
			reg_natural_tone_d1 <= 1'b0;
			reg_tone_enable_d1 <= 1'b0;
			reg_saturation_enable_d1 <= 1'b0;
			reg_stage_enable_d1 <= 1'b0;
			reg_saturation_d1 <= 3'd0;
			reg_tag_d1 <= 160'd0;
			reg_frame_d1 <= 1'b0;
			reg_algo_d1 <= 64'd0;
			reg_isp_d1 <= 520'd0;
			reg_epoch_d1 <= 64'd0;
			reg_continuous_d1 <= 1'b0;
			reg_alive_d1 <= 4'd0;
			reg_alive_valid_d1 <= 1'b0;
			reg_tone_rgb_d2 <= 24'd0;
			reg_saturation_enable_d2 <= 1'b0;
			reg_stage_enable_d2 <= 1'b0;
			reg_saturation_d2 <= 3'd0;
			reg_tag_d2 <= 160'd0;
			reg_frame_d2 <= 1'b0;
			reg_algo_d2 <= 64'd0;
			reg_isp_d2 <= 520'd0;
			reg_epoch_d2 <= 64'd0;
			reg_continuous_d2 <= 1'b0;
			reg_alive_d2 <= 4'd0;
			reg_alive_valid_d2 <= 1'b0;
			reg_tone_rgb_d3 <= 24'd0;
			reg_luma_term_r_d3 <= 16'd0;
			reg_luma_term_g_d3 <= 16'd0;
			reg_luma_term_b_d3 <= 16'd0;
			reg_saturation_enable_d3 <= 1'b0;
			reg_stage_enable_d3 <= 1'b0;
			reg_saturation_d3 <= 3'd0;
			reg_tag_d3 <= 160'd0;
			reg_frame_d3 <= 1'b0;
			reg_algo_d3 <= 64'd0;
			reg_isp_d3 <= 520'd0;
			reg_epoch_d3 <= 64'd0;
			reg_continuous_d3 <= 1'b0;
			reg_alive_d3 <= 4'd0;
			reg_alive_valid_d3 <= 1'b0;
			reg_tone_rgb_d4 <= 24'd0;
			reg_luma_value_d4 <= 8'd0;
			reg_chroma_r_d4 <= 10'sd0;
			reg_chroma_g_d4 <= 10'sd0;
			reg_chroma_b_d4 <= 10'sd0;
			reg_saturation_enable_d4 <= 1'b0;
			reg_stage_enable_d4 <= 1'b0;
			reg_saturation_d4 <= 3'd0;
			reg_tag_d4 <= 160'd0;
			reg_frame_d4 <= 1'b0;
			reg_algo_d4 <= 64'd0;
			reg_isp_d4 <= 520'd0;
			reg_epoch_d4 <= 64'd0;
			reg_continuous_d4 <= 1'b0;
			reg_alive_d4 <= 4'd0;
			reg_alive_valid_d4 <= 1'b0;
			reg_tag_d5 <= 160'd0;
			reg_frame_d5 <= 1'b0;
			reg_algo_d5 <= 64'd0;
			reg_isp_d5 <= 520'd0;
			reg_epoch_d5 <= 64'd0;
			reg_continuous_d5 <= 1'b0;
			reg_alive_d5 <= 4'd0;
			reg_alive_valid_d5 <= 1'b0;
			rgb_o <= 24'd0;
		end else begin
			reg_rgb_d0 <= i_rgb;
			reg_luma_d0 <= input_luma_sum[15:8];
			reg_natural_tone_d0 <= input_tile_config[126];
			reg_tag_d0 <= i_tag;
			reg_frame_d0 <= i_frame_boundary;
			reg_algo_d0 <= i_algo_config;
			reg_isp_d0 <= i_isp_config;
			reg_epoch_d0 <= i_effective_epoch;
			reg_continuous_d0 <= i_isp_continuous_update;
			reg_alive_d0 <= i_source_alive;
			reg_alive_valid_d0 <= i_source_alive_valid;
			// The CDC canonical map reserves tile bit124 for tone and bit125
			// for saturation, matching the host command mask bit4/bit5.
			reg_saturation_enable_d0 <= input_tile_config[125];
			reg_tone_enable_d0 <= input_tile_config[124];
			reg_profile_d0 <= input_tile_config[25:23];
			reg_saturation_d0 <= input_tile_config[22:20];
			reg_stage_enable_d0 <= input_stage_enable;
			reg_raw_rgb_d1 <= reg_rgb_d0;
			reg_luma_d1 <= reg_luma_d0;
			reg_natural_tone_d1 <= reg_natural_tone_d0;
			reg_min_rgb_d1 <= (raw_min_rg < reg_rgb_d0[7:0]) ? raw_min_rg : reg_rgb_d0[7:0];
			reg_max_rgb_d1 <= (raw_max_rg > reg_rgb_d0[7:0]) ? raw_max_rg : reg_rgb_d0[7:0];
			reg_tone_enable_d1 <= reg_tone_enable_d0;
			reg_saturation_enable_d1 <= reg_saturation_enable_d0;
			reg_stage_enable_d1 <= reg_stage_enable_d0;
			reg_saturation_d1 <= reg_saturation_d0;
			reg_tag_d1 <= reg_tag_d0;
			reg_frame_d1 <= reg_frame_d0;
			reg_algo_d1 <= reg_algo_d0;
			reg_isp_d1 <= reg_isp_d0;
			reg_epoch_d1 <= reg_epoch_d0;
			reg_continuous_d1 <= reg_continuous_d0;
			reg_alive_d1 <= reg_alive_d0;
			reg_alive_valid_d1 <= reg_alive_valid_d0;
			// The official ROM captures its address on this edge. Its data for
			// the d0 pixel is therefore sampled on the following edge.
			if(reg_stage_enable_d1 && reg_tone_enable_d1)begin
				if(reg_natural_tone_d1)begin
					reg_tone_rgb_d2 <= {natural_r[7:0],natural_g[7:0],natural_b[7:0]};
				end else begin
					reg_tone_rgb_d2 <= {tone_data_r,tone_data_g,tone_data_b};
				end
			end else begin
				reg_tone_rgb_d2 <= reg_raw_rgb_d1;
			end
			reg_saturation_enable_d2 <= reg_saturation_enable_d1;
			reg_stage_enable_d2 <= reg_stage_enable_d1;
			reg_saturation_d2 <= reg_saturation_d1;
			reg_tag_d2 <= reg_tag_d1;
			reg_frame_d2 <= reg_frame_d1;
			reg_algo_d2 <= reg_algo_d1;
			reg_isp_d2 <= reg_isp_d1;
			reg_epoch_d2 <= reg_epoch_d1;
			reg_continuous_d2 <= reg_continuous_d1;
			reg_alive_d2 <= reg_alive_d1;
			reg_alive_valid_d2 <= reg_alive_valid_d1;
			reg_tone_rgb_d3 <= reg_tone_rgb_d2;
			reg_luma_term_r_d3 <= 16'd77 * reg_tone_rgb_d2[23:16];
			reg_luma_term_g_d3 <= 16'd150 * reg_tone_rgb_d2[15:8];
			reg_luma_term_b_d3 <= 16'd29 * reg_tone_rgb_d2[7:0];
			reg_saturation_enable_d3 <= reg_saturation_enable_d2;
			reg_stage_enable_d3 <= reg_stage_enable_d2;
			reg_saturation_d3 <= reg_saturation_d2;
			reg_tag_d3 <= reg_tag_d2;
			reg_frame_d3 <= reg_frame_d2;
			reg_algo_d3 <= reg_algo_d2;
			reg_isp_d3 <= reg_isp_d2;
			reg_epoch_d3 <= reg_epoch_d2;
			reg_continuous_d3 <= reg_continuous_d2;
			reg_alive_d3 <= reg_alive_d2;
			reg_alive_valid_d3 <= reg_alive_valid_d2;
			reg_tone_rgb_d4 <= reg_tone_rgb_d3;
			reg_luma_value_d4 <= luma_sum[15:8];
			reg_chroma_r_d4 <= $signed({1'b0,reg_tone_rgb_d3[23:16]}) - $signed({1'b0,luma_sum[15:8]});
			reg_chroma_g_d4 <= $signed({1'b0,reg_tone_rgb_d3[15:8]}) - $signed({1'b0,luma_sum[15:8]});
			reg_chroma_b_d4 <= $signed({1'b0,reg_tone_rgb_d3[7:0]}) - $signed({1'b0,luma_sum[15:8]});
			reg_saturation_enable_d4 <= reg_saturation_enable_d3;
			reg_stage_enable_d4 <= reg_stage_enable_d3;
			reg_saturation_d4 <= reg_saturation_d3;
			reg_tag_d4 <= reg_tag_d3;
			reg_frame_d4 <= reg_frame_d3;
			reg_algo_d4 <= reg_algo_d3;
			reg_isp_d4 <= reg_isp_d3;
			reg_epoch_d4 <= reg_epoch_d3;
			reg_continuous_d4 <= reg_continuous_d3;
			reg_alive_d4 <= reg_alive_d3;
			reg_alive_valid_d4 <= reg_alive_valid_d3;
			reg_tag_d5 <= reg_tag_d4;
			reg_frame_d5 <= reg_frame_d4;
			reg_algo_d5 <= reg_algo_d4;
			reg_isp_d5 <= reg_isp_d4;
			reg_epoch_d5 <= reg_epoch_d4;
			reg_continuous_d5 <= reg_continuous_d4;
			reg_alive_d5 <= reg_alive_d4;
			reg_alive_valid_d5 <= reg_alive_valid_d4;
			if(reg_stage_enable_d4 && reg_saturation_enable_d4)begin
				rgb_o <= {saturated_r,saturated_g,saturated_b};
			end else begin
				rgb_o <= reg_tone_rgb_d4;
			end
		end
	end

	assign o_rgb = rgb_o;
	assign o_tag = reg_tag_d5;
	assign o_frame_boundary = reg_frame_d5;
	assign o_algo_config = reg_algo_d5;
	assign o_isp_config = reg_isp_d5;
	assign o_effective_epoch = reg_epoch_d5;
	assign o_isp_continuous_update = reg_continuous_d5;
	assign o_source_alive = reg_alive_d5;
	assign o_source_alive_valid = reg_alive_valid_d5;
endmodule
