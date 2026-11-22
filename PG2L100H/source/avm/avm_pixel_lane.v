//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 单像素 AVM 算术流水：八个 RGB565 邻域先展开 RGB888，对两路像素分别双线性插值，再按 alpha 加权融合并输出。
// Module Name: avm_pixel_lane, avm_scale_u12_q4, avm_scale_u8_q6, avm_bilinear_rgb888
// File Name: source/avm/avm_pixel_lane.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 单像素 AVM 算术流水：八个 RGB565 邻域先展开 RGB888，对两路像素分别双线性插值，再按 alpha 加权融合并输出。
// 双线性采用 (sum+128)>>8，alpha 混合采用 (sum+32)>>6；每拍可接受一个输入，无背压，输出在第四个采样沿更新。
// 同文件包含 avm_scale_u12_q4、avm_scale_u8_q6、avm_bilinear_rgb888 小位宽移位加法及双线性子模块；本模块不自行访问 DDR/LUT。
//
// Dependencies: 同文件三个算术子模块；输入邻域须属于同一 frame_epoch，无 RAM/FIFO/乘法 IP。
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
// Create Date: 2026-10-13
// Design Name: AVM golden arithmetic
// Module Name: avm_pixel_lane
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: RGB565先展开RGB888；双线性(sum+128)>>8，alpha(sum+32)>>6。
//              输入八个邻域已经来自同一frame_epoch；本模块不读取DDR/LUT。
//              每拍可接收一个像素；输出在第四个采样沿更新，无背压。
// Dependencies: 本文件的小位宽移位加法器；无RAM/FIFO/乘法IP依赖。
// Revision 0.01 - 算术独立交付，不能代表AVM显示通路已接入。
// Revision 0.02 - 2026-10-14 纵向乘积寄存，切断fy到舍入结果组合路径。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_pixel_lane
(
	input i_clk,
	input i_rstn,
	input i_valid,
	input [95:0] i_lut,
	input [63:0] i_neighbors_a,
	input [63:0] i_neighbors_b,
	output o_valid,
	output [15:0] o_rgb565
);
	wire [23:0] sample_a;
	wire [23:0] sample_b;
	reg [6:0] reg_alpha_d1;
	reg [6:0] reg_alpha_d2;
	reg [6:0] reg_alpha_d3;
	reg [3:0] reg_valid;
	reg [3:0] reg_enabled;
	reg [15:0] rgb565_o;
	wire [23:0] mixed_rgb;
	wire lut_legal;
	assign lut_legal = (i_lut[95:87] == 9'd0) && (i_lut[78:68] <= 11'd64) &&
		(i_lut[43:40] == 4'd0) && (i_lut[51:48] == 4'd0) &&
		(i_lut[59:56] == 4'd0) && (i_lut[67:64] == 4'd0) &&
		(i_lut[81:79] < 3'd4) && (!i_lut[86] || i_lut[84:82] < 3'd4);
	assign o_valid = reg_valid[3];
	assign o_rgb565 = rgb565_o;
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_alpha_d1 <= 7'd0;
			reg_alpha_d2 <= 7'd0;
			reg_alpha_d3 <= 7'd0;
			reg_valid <= 4'd0;
			reg_enabled <= 4'd0;
			rgb565_o <= 16'd0;
		end else begin
			reg_alpha_d1 <= i_lut[86] ? i_lut[74:68] : 7'd64;
			reg_alpha_d2 <= reg_alpha_d1;
			reg_alpha_d3 <= reg_alpha_d2;
			reg_valid <= {reg_valid[2:0], i_valid};
			reg_enabled <= {reg_enabled[2:0], i_lut[85] && lut_legal};
			if(reg_enabled[2])begin
				rgb565_o <= {mixed_rgb[23:19], mixed_rgb[15:10], mixed_rgb[7:3]};
			end else begin
				rgb565_o <= 16'd0;
			end
		end
	end
	avm_bilinear_rgb888 avm_bilinear_rgb888_Inst_a(
		.i_clk(i_clk), .i_rstn(i_rstn), .i_pixels(i_neighbors_a),
		.i_fx(i_lut[39:36]), .i_fy(i_lut[47:44]), .o_rgb(sample_a));
	avm_bilinear_rgb888 avm_bilinear_rgb888_Inst_b(
		.i_clk(i_clk), .i_rstn(i_rstn), .i_pixels(i_neighbors_b),
		.i_fx(i_lut[55:52]), .i_fy(i_lut[63:60]), .o_rgb(sample_b));
	genvar channel;
	generate
		for(channel = 0; channel < 3; channel = channel + 1)begin:gen_mix
			wire [14:0] product_a;
			wire [14:0] product_b;
			wire [15:0] rounded_sum;
			avm_scale_u8_q6 avm_scale_u8_q6_Inst_a(
				.i_value(sample_a[channel * 8 +: 8]), .i_weight(reg_alpha_d3), .o_product(product_a));
			avm_scale_u8_q6 avm_scale_u8_q6_Inst_b(
				.i_value(sample_b[channel * 8 +: 8]), .i_weight(7'd64 - reg_alpha_d3), .o_product(product_b));
			assign rounded_sum = {1'b0, product_a} + {1'b0, product_b} + 16'd32;
			assign mixed_rgb[channel * 8 +: 8] = rounded_sum[13:6];
		end
	endgenerate
endmodule

// 小系数由五个条件移位相加实现；不声明存储或推断复杂乘法资源。
module avm_scale_u12_q4
(
	input [11:0] i_value,
	input [4:0] i_weight,
	output [16:0] o_product
);
	wire [16:0] value_extended;
	assign value_extended = {5'd0, i_value};
	assign o_product = (i_weight[0] ? value_extended : 17'd0) +
		(i_weight[1] ? value_extended << 1 : 17'd0) +
		(i_weight[2] ? value_extended << 2 : 17'd0) +
		(i_weight[3] ? value_extended << 3 : 17'd0) +
		(i_weight[4] ? value_extended << 4 : 17'd0);
endmodule

module avm_scale_u8_q6
(
	input [7:0] i_value,
	input [6:0] i_weight,
	output [14:0] o_product
);
	wire [14:0] value_extended;
	assign value_extended = {7'd0, i_value};
	assign o_product = (i_weight[0] ? value_extended : 15'd0) +
		(i_weight[1] ? value_extended << 1 : 15'd0) +
		(i_weight[2] ? value_extended << 2 : 15'd0) +
		(i_weight[3] ? value_extended << 3 : 15'd0) +
		(i_weight[4] ? value_extended << 4 : 15'd0) +
		(i_weight[5] ? value_extended << 5 : 15'd0) +
		(i_weight[6] ? value_extended << 6 : 15'd0);
endmodule

module avm_bilinear_rgb888
(
	input i_clk,
	input i_rstn,
	input [63:0] i_pixels,
	input [3:0] i_fx,
	input [3:0] i_fy,
	output [23:0] o_rgb
);
	wire [95:0] expanded;
	reg [3:0] reg_fy;
	genvar pixel;
	genvar channel;
	generate
		for(pixel = 0; pixel < 4; pixel = pixel + 1)begin:gen_expand
			wire [15:0] rgb565;
			assign rgb565 = i_pixels[pixel * 16 +: 16];
			assign expanded[pixel * 24 +: 24] = {
				rgb565[15:11], rgb565[15:13], rgb565[10:5], rgb565[10:9], rgb565[4:0], rgb565[4:2]};
		end
		for(channel = 0; channel < 3; channel = channel + 1)begin:gen_channel
			wire [16:0] h00, h01, h10, h11;
			wire [16:0] v0, v1;
			wire [17:0] rounded;
			reg [11:0] reg_h0, reg_h1;
			reg [16:0] reg_v0;
			reg [16:0] reg_v1;
			reg [7:0] rgb_o;
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_h00(.i_value({4'd0, expanded[channel * 8 +: 8]}), .i_weight(5'd16 - {1'b0, i_fx}), .o_product(h00));
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_h01(.i_value({4'd0, expanded[24 + channel * 8 +: 8]}), .i_weight({1'b0, i_fx}), .o_product(h01));
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_h10(.i_value({4'd0, expanded[48 + channel * 8 +: 8]}), .i_weight(5'd16 - {1'b0, i_fx}), .o_product(h10));
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_h11(.i_value({4'd0, expanded[72 + channel * 8 +: 8]}), .i_weight({1'b0, i_fx}), .o_product(h11));
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_v0(.i_value(reg_h0), .i_weight(5'd16 - {1'b0, reg_fy}), .o_product(v0));
			avm_scale_u12_q4 avm_scale_u12_q4_Inst_v1(.i_value(reg_h1), .i_weight({1'b0, reg_fy}), .o_product(v1));
			assign rounded = {1'b0, reg_v0} + {1'b0, reg_v1} + 18'd128;
			assign o_rgb[channel * 8 +: 8] = rgb_o;
			always@(posedge i_clk or negedge i_rstn)begin
				if(!i_rstn)begin
					reg_h0 <= 12'd0;
					reg_h1 <= 12'd0;
					reg_v0 <= 17'd0;
					reg_v1 <= 17'd0;
					rgb_o <= 8'd0;
				end else begin
					reg_h0 <= h00 + h01;
					reg_h1 <= h10 + h11;
					reg_v0 <= v0;
					reg_v1 <= v1;
					rgb_o <= rounded[15:8];
				end
			end
		end
	endgenerate
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin reg_fy <= 4'd0; end
		else begin reg_fy <= i_fy; end
	end
endmodule
