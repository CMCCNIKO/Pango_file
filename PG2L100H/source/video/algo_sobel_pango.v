//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 对整数灰度执行 3×3 Sobel 梯度计算，以 Gx²+Gy² 与 threshold² 比较生成黑边白底图像。
// Module Name: algo_sobel_pango
// File Name: source/video/algo_sobel_pango.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对整数灰度执行 3×3 Sobel 梯度计算，以 Gx²+Gy² 与 threshold² 比较生成黑边白底图像。
// 零门限映射为 128，门限随像素传播；固定 12 拍且边界置黑，不依赖 CORDIC。
//
// Dependencies: video_window_3x3、其内部 pango_line_ram_1280x8。
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
// Create Date: 2026-10-09
// Design Name: Sobel HDMI Algorithm
// Module Name: algo_sobel_pango
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Integer Sobel on Y=(77R+150G+29B)>>8. Edge is black, other pixels white.
//              Edge iff Gx^2+Gy^2 >= threshold^2; zero threshold maps to 128.
//              Fixed 12 clocks: Y1+window2+gradient1+square1+compare1+padding6.
//              Causal border is black; threshold follows its input pixel.
// Dependencies: video_window_3x3, pango_line_ram_1280x8; no CORDIC or Xilinx IP.
// Revision: 0.01 - Integer squared-magnitude implementation.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module algo_sobel_pango
#(
	parameter integer C_FRAME_WIDTH = 1280,
	parameter integer C_FRAME_HEIGHT = 720
)
(
	input i_clk,
	input i_rst_n,
	input i_vs,
	input i_hs,
	input i_de,
	input i_sof,
	input i_eol,
	input [23:0] i_rgb,
	input [10:0] i_x,
	input [9:0] i_y,
	input [10:0] i_threshold,
	output o_vs,
	output o_hs,
	output o_de,
	output o_sof,
	output o_eol,
	output [23:0] o_rgb
);
	wire [15:0] gray_sum;
	reg [7:0] reg_gray;
	reg [4:0] reg_gray_timing;
	reg [10:0] reg_x;
	reg [9:0] reg_y;
	wire [7:0] window_pixels [0:8];
	wire [4:0] window_timing;
	wire window_border;
	wire [9:0] gradient_left;
	wire [9:0] gradient_right;
	wire [9:0] gradient_top;
	wire [9:0] gradient_bottom;
	reg [9:0] reg_gx;
	reg [9:0] reg_gy;
	reg [19:0] reg_gx_square;
	reg [19:0] reg_gy_square;
	wire [20:0] gradient_square_sum;
	reg [10:0] reg_threshold [0:3];
	reg [21:0] reg_threshold_square;
	reg [5:0] reg_gradient_timing;
	reg [5:0] reg_square_timing;
	reg [5:0] reg_compare_timing;
	reg [23:0] reg_compare_rgb;
	reg [23:0] reg_rgb_pad [0:5];
	reg [5:0] reg_timing_pad [0:5];
	integer cnt_threshold;
	integer cnt_padding;

	assign gray_sum = i_rgb[23:16] * 16'd77 + i_rgb[15:8] * 16'd150 + i_rgb[7:0] * 16'd29;
	assign gradient_left = {2'd0, window_pixels[0]} + {1'b0, window_pixels[3], 1'b0} + {2'd0, window_pixels[6]};
	assign gradient_right = {2'd0, window_pixels[2]} + {1'b0, window_pixels[5], 1'b0} + {2'd0, window_pixels[8]};
	assign gradient_top = {2'd0, window_pixels[0]} + {1'b0, window_pixels[1], 1'b0} + {2'd0, window_pixels[2]};
	assign gradient_bottom = {2'd0, window_pixels[6]} + {1'b0, window_pixels[7], 1'b0} + {2'd0, window_pixels[8]};
	assign gradient_square_sum = {1'b0, reg_gx_square} + {1'b0, reg_gy_square};
	assign {o_vs, o_hs, o_de, o_sof, o_eol} = reg_timing_pad[5][4:0];
	assign o_rgb = reg_rgb_pad[5];

	// Gray data and canonical tags enter the shared two-line window together.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_gray <= 8'd0;
			reg_gray_timing <= 5'd0;
			reg_x <= 11'd0;
			reg_y <= 10'd0;
		end else begin
			reg_gray <= gray_sum[15:8];
			reg_gray_timing <= {i_vs, i_hs, i_de, i_sof, i_eol};
			reg_x <= i_x;
			reg_y <= i_y;
		end
	end

	// Threshold gets the same four clocks as Y/window/gradient before squaring.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			for(cnt_threshold = 0; cnt_threshold < 4; cnt_threshold = cnt_threshold + 1)begin
				reg_threshold[cnt_threshold] <= 11'd128;
			end
		end else begin
			reg_threshold[0] <= (i_threshold == 11'd0) ? 11'd128 : i_threshold;
			for(cnt_threshold = 1; cnt_threshold < 4; cnt_threshold = cnt_threshold + 1)begin
				reg_threshold[cnt_threshold] <= reg_threshold[cnt_threshold - 1];
			end
		end
	end

	// Unsigned absolute gradients avoid signed subtraction and retain full 0..1020 range.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_gx <= 10'd0;
			reg_gy <= 10'd0;
			reg_gradient_timing <= 6'd0;
		end else begin
			reg_gx <= (gradient_right >= gradient_left) ? gradient_right - gradient_left : gradient_left - gradient_right;
			reg_gy <= (gradient_bottom >= gradient_top) ? gradient_bottom - gradient_top : gradient_top - gradient_bottom;
			reg_gradient_timing <= {window_border, window_timing};
		end
	end

	// Twenty bits cover each gradient square; threshold's full eleven-bit square needs 22.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_gx_square <= 20'd0;
			reg_gy_square <= 20'd0;
			reg_threshold_square <= 22'd16384;
			reg_square_timing <= 6'd0;
		end else begin
			reg_gx_square <= reg_gx * reg_gx;
			reg_gy_square <= reg_gy * reg_gy;
			reg_threshold_square <= reg_threshold[3] * reg_threshold[3];
			reg_square_timing <= reg_gradient_timing;
		end
	end

	// Squared comparison exactly implements the new integer threshold contract.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_compare_rgb <= 24'd0;
			reg_compare_timing <= 6'd0;
		end else begin
			reg_compare_timing <= reg_square_timing;
			if(reg_square_timing[5] || !reg_square_timing[2] || ({1'b0, gradient_square_sum} >= reg_threshold_square))begin
				reg_compare_rgb <= 24'h000000;
			end else begin
				reg_compare_rgb <= 24'hffffff;
			end
		end
	end

	// Six padding registers match the twelve-clock latency of the other engines.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			for(cnt_padding = 0; cnt_padding < 6; cnt_padding = cnt_padding + 1)begin
				reg_rgb_pad[cnt_padding] <= 24'd0;
				reg_timing_pad[cnt_padding] <= 6'd0;
			end
		end else begin
			reg_rgb_pad[0] <= reg_compare_rgb;
			reg_timing_pad[0] <= reg_compare_timing;
			for(cnt_padding = 1; cnt_padding < 6; cnt_padding = cnt_padding + 1)begin
				reg_rgb_pad[cnt_padding] <= reg_rgb_pad[cnt_padding - 1];
				reg_timing_pad[cnt_padding] <= reg_timing_pad[cnt_padding - 1];
			end
		end
	end

	video_window_3x3 #(
		.C_DATA_WIDTH(8), .C_FRAME_WIDTH(C_FRAME_WIDTH), .C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_window_3x3_Inst_gray(
		.i_clk(i_clk), .i_rst_n(i_rst_n),
		.i_vs(reg_gray_timing[4]), .i_hs(reg_gray_timing[3]), .i_de(reg_gray_timing[2]),
		.i_sof(reg_gray_timing[1]), .i_eol(reg_gray_timing[0]), .i_data(reg_gray), .i_x(reg_x), .i_y(reg_y),
		.o_vs(window_timing[4]), .o_hs(window_timing[3]), .o_de(window_timing[2]),
		.o_sof(window_timing[1]), .o_eol(window_timing[0]), .o_border(window_border),
		.o_p11(window_pixels[0]), .o_p12(window_pixels[1]), .o_p13(window_pixels[2]),
		.o_p21(window_pixels[3]), .o_p22(window_pixels[4]), .o_p23(window_pixels[5]),
		.o_p31(window_pixels[6]), .o_p32(window_pixels[7]), .o_p33(window_pixels[8])
	);
endmodule
