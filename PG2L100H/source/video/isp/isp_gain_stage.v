//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 对 RGB 三通道并行执行无符号 8×10 增益乘法、Q8 舍入、带符号偏移与 8 位饱和。
// Module Name: isp_gain_stage
// File Name: source/video/isp/isp_gain_stage.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对 RGB 三通道并行执行无符号 8×10 增益乘法、Q8 舍入、带符号偏移与 8 位饱和。
// 空白及无效内容走同样固定六拍旁路，所有标签与像素严格等延迟。
//
// Dependencies: 官方 isp_gain8x10 乘法 IP。
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
// Design Name: ISP per-channel gain and offset stage
// Module Name: isp_gain_stage
// Project Name: PG2L100H_AVM_ISP
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Three parallel unsigned 8x10 gain paths with Q8 rounding,
//              signed offset and saturation. Blanking and invalid content are
//              delayed through the same fixed six-clock path and bypassed.
// Dependencies: Official isp_gain8x10 multiplier IP.
// Revision: 0.01 - 2026-10-19, initial implementation.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module isp_gain_stage
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
	localparam integer PIPE_DEPTH = 6;
	reg [23:0] reg_rgb_pipe [0:PIPE_DEPTH - 1];
	reg [159:0] reg_tag_pipe [0:PIPE_DEPTH - 1];
	reg reg_frame_pipe [0:PIPE_DEPTH - 1];
	reg [63:0] reg_algo_pipe [0:PIPE_DEPTH - 1];
	reg [519:0] reg_isp_pipe [0:PIPE_DEPTH - 1];
	reg [63:0] reg_epoch_pipe [0:PIPE_DEPTH - 1];
	reg reg_continuous_pipe [0:PIPE_DEPTH - 1];
	reg [3:0] reg_alive_pipe [0:PIPE_DEPTH - 1];
	reg reg_alive_valid_pipe [0:PIPE_DEPTH - 1];
	reg reg_enable_pipe [0:PIPE_DEPTH - 1];
	reg signed [7:0] reg_offset_r_pipe [0:PIPE_DEPTH - 1];
	reg signed [7:0] reg_offset_g_pipe [0:PIPE_DEPTH - 1];
	reg signed [7:0] reg_offset_b_pipe [0:PIPE_DEPTH - 1];
	wire [17:0] product_r;
	wire [17:0] product_g;
	wire [17:0] product_b;
	wire [18:0] product_round_r = {1'b0,product_r} + 19'd128;
	wire [18:0] product_round_g = {1'b0,product_g} + 19'd128;
	wire [18:0] product_round_b = {1'b0,product_b} + 19'd128;
	reg [10:0] reg_round_r;
	reg [10:0] reg_round_g;
	reg [10:0] reg_round_b;
	reg signed [11:0] corrected_r;
	reg signed [11:0] corrected_g;
	reg signed [11:0] corrected_b;
	reg [7:0] rgb_r_o;
	reg [7:0] rgb_g_o;
	reg [7:0] rgb_b_o;
	reg [23:0] rgb_o;
	integer pipe_index;

	wire [1:0] input_tile = i_tag[133:132];
	reg [127:0] input_tile_config;
	wire input_content_valid = i_tag[157] && i_tag[112];
	wire input_stage_enable = input_content_valid && i_isp_config[512] &&
		i_isp_config[513 + input_tile] && input_tile_config[120];
	always@(*)begin
		case(input_tile)
			2'd0: input_tile_config = i_isp_config[127:0];
			2'd1: input_tile_config = i_isp_config[255:128];
			2'd2: input_tile_config = i_isp_config[383:256];
			2'd3: input_tile_config = i_isp_config[511:384];
			default: input_tile_config = 128'd0;
		endcase
	end

	isp_gain8x10 isp_gain8x10_Inst_r(
		.ce(i_rstn), .rst(!i_rstn), .clk(i_clk),
		.a(i_rgb[23:16]), .b(input_tile_config[119:110]), .p(product_r));
	isp_gain8x10 isp_gain8x10_Inst_g(
		.ce(i_rstn), .rst(!i_rstn), .clk(i_clk),
		.a(i_rgb[15:8]), .b(input_tile_config[109:100]), .p(product_g));
	isp_gain8x10 isp_gain8x10_Inst_b(
		.ce(i_rstn), .rst(!i_rstn), .clk(i_clk),
		.a(i_rgb[7:0]), .b(input_tile_config[99:90]), .p(product_b));

	always@(*)begin
		if(corrected_r < 0)begin rgb_r_o = 8'd0; end
		else if(corrected_r > 255)begin rgb_r_o = 8'd255; end
		else begin rgb_r_o = corrected_r[7:0]; end
		if(corrected_g < 0)begin rgb_g_o = 8'd0; end
		else if(corrected_g > 255)begin rgb_g_o = 8'd255; end
		else begin rgb_g_o = corrected_g[7:0]; end
		if(corrected_b < 0)begin rgb_b_o = 8'd0; end
		else if(corrected_b > 255)begin rgb_b_o = 8'd255; end
		else begin rgb_b_o = corrected_b[7:0]; end
	end

	// Three IP registers precede rounding, offset and final saturation.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_round_r <= 11'd0;
			reg_round_g <= 11'd0;
			reg_round_b <= 11'd0;
			corrected_r <= 12'sd0;
			corrected_g <= 12'sd0;
			corrected_b <= 12'sd0;
		end else begin
			reg_round_r <= product_round_r[18:8];
			reg_round_g <= product_round_g[18:8];
			reg_round_b <= product_round_b[18:8];
			corrected_r <= $signed({1'b0,reg_round_r}) +
				$signed({{4{reg_offset_r_pipe[3][7]}},reg_offset_r_pipe[3]});
			corrected_g <= $signed({1'b0,reg_round_g}) +
				$signed({{4{reg_offset_g_pipe[3][7]}},reg_offset_g_pipe[3]});
			corrected_b <= $signed({1'b0,reg_round_b}) +
				$signed({{4{reg_offset_b_pipe[3][7]}},reg_offset_b_pipe[3]});
		end
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			for(pipe_index = 0; pipe_index < PIPE_DEPTH; pipe_index = pipe_index + 1)begin
				reg_rgb_pipe[pipe_index] <= 24'd0;
				reg_tag_pipe[pipe_index] <= 160'd0;
				reg_frame_pipe[pipe_index] <= 1'b0;
				reg_algo_pipe[pipe_index] <= 64'd0;
				reg_isp_pipe[pipe_index] <= 520'd0;
				reg_epoch_pipe[pipe_index] <= 64'd0;
				reg_continuous_pipe[pipe_index] <= 1'b0;
				reg_alive_pipe[pipe_index] <= 4'd0;
				reg_alive_valid_pipe[pipe_index] <= 1'b0;
				reg_enable_pipe[pipe_index] <= 1'b0;
				reg_offset_r_pipe[pipe_index] <= 8'sd0;
				reg_offset_g_pipe[pipe_index] <= 8'sd0;
				reg_offset_b_pipe[pipe_index] <= 8'sd0;
			end
			rgb_o <= 24'd0;
		end else begin
			reg_rgb_pipe[0] <= i_rgb;
			reg_tag_pipe[0] <= i_tag;
			reg_frame_pipe[0] <= i_frame_boundary;
			reg_algo_pipe[0] <= i_algo_config;
			reg_isp_pipe[0] <= i_isp_config;
			reg_epoch_pipe[0] <= i_effective_epoch;
			reg_continuous_pipe[0] <= i_isp_continuous_update;
			reg_alive_pipe[0] <= i_source_alive;
			reg_alive_valid_pipe[0] <= i_source_alive_valid;
			reg_enable_pipe[0] <= input_stage_enable;
			reg_offset_r_pipe[0] <= input_tile_config[89:82];
			reg_offset_g_pipe[0] <= input_tile_config[81:74];
			reg_offset_b_pipe[0] <= input_tile_config[73:66];
			for(pipe_index = 1; pipe_index < PIPE_DEPTH; pipe_index = pipe_index + 1)begin
				reg_rgb_pipe[pipe_index] <= reg_rgb_pipe[pipe_index - 1];
				reg_tag_pipe[pipe_index] <= reg_tag_pipe[pipe_index - 1];
				reg_frame_pipe[pipe_index] <= reg_frame_pipe[pipe_index - 1];
				reg_algo_pipe[pipe_index] <= reg_algo_pipe[pipe_index - 1];
				reg_isp_pipe[pipe_index] <= reg_isp_pipe[pipe_index - 1];
				reg_epoch_pipe[pipe_index] <= reg_epoch_pipe[pipe_index - 1];
				reg_continuous_pipe[pipe_index] <= reg_continuous_pipe[pipe_index - 1];
				reg_alive_pipe[pipe_index] <= reg_alive_pipe[pipe_index - 1];
				reg_alive_valid_pipe[pipe_index] <= reg_alive_valid_pipe[pipe_index - 1];
				reg_enable_pipe[pipe_index] <= reg_enable_pipe[pipe_index - 1];
				reg_offset_r_pipe[pipe_index] <= reg_offset_r_pipe[pipe_index - 1];
				reg_offset_g_pipe[pipe_index] <= reg_offset_g_pipe[pipe_index - 1];
				reg_offset_b_pipe[pipe_index] <= reg_offset_b_pipe[pipe_index - 1];
			end
			if(reg_enable_pipe[PIPE_DEPTH - 2])begin
				rgb_o <= {rgb_r_o,rgb_g_o,rgb_b_o};
			end else begin
				rgb_o <= reg_rgb_pipe[PIPE_DEPTH - 2];
			end
		end
	end

	assign o_rgb = rgb_o;
	assign o_tag = reg_tag_pipe[PIPE_DEPTH - 1];
	assign o_frame_boundary = reg_frame_pipe[PIPE_DEPTH - 1];
	assign o_algo_config = reg_algo_pipe[PIPE_DEPTH - 1];
	assign o_isp_config = reg_isp_pipe[PIPE_DEPTH - 1];
	assign o_effective_epoch = reg_epoch_pipe[PIPE_DEPTH - 1];
	assign o_isp_continuous_update = reg_continuous_pipe[PIPE_DEPTH - 1];
	assign o_source_alive = reg_alive_pipe[PIPE_DEPTH - 1];
	assign o_source_alive_valid = reg_alive_valid_pipe[PIPE_DEPTH - 1];
endmodule
