//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 分别对 RGB 三通道执行因果 3×3 高斯滤波，核为 [1 2 1;2 4 2;1 2 1]/16 并截断。
// Module Name: algo_gaussian_pango
// File Name: source/video/algo_gaussian_pango.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 分别对 RGB 三通道执行因果 3×3 高斯滤波，核为 [1 2 1;2 4 2;1 2 1]/16 并截断。
// 固定 12 拍输出，与旁路保持等延迟；边界按掩码输出黑色，窗口有效中心对应输入坐标减一。
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
// Design Name: RGB Gaussian HDMI Algorithm
// Module Name: algo_gaussian_pango
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: RGB channels independently use [1 2 1;2 4 2;1 2 1]/16 with truncation.
//              Fixed 12 clocks: window(2), horizontal(1), vertical(1), padding(8).
//              Causal center (x-1,y-1); black first two rows/columns and final row/column.
// Dependencies: video_window_3x3, pango_line_ram_1280x8
// Revision: 0.01 - Pango-oriented RTL; no Xilinx generated IP imported.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module algo_gaussian_pango
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
	output o_vs,
	output o_hs,
	output o_de,
	output o_sof,
	output o_eol,
	output [23:0] o_rgb
);
	wire [23:0] window_pixels [0:8];
	wire [4:0] window_timing;
	wire window_border;
	wire [9:0] row_sums [0:8];
	wire [23:0] normalized_rgb;
	wire [23:0] masked_rgb;
	reg [5:0] reg_row_timing;
	reg [5:0] reg_sum_timing;
	reg [23:0] reg_rgb_pad [0:7];
	reg [5:0] reg_timing_pad [0:7];
	integer cnt_padding;
	genvar channel_index;
	genvar row_index;

	assign masked_rgb = (reg_sum_timing[5] || !reg_sum_timing[2]) ? 24'd0 : normalized_rgb;
	assign o_rgb = reg_rgb_pad[7];
	assign {o_vs, o_hs, o_de, o_sof, o_eol} = reg_timing_pad[7][4:0];

	// Two arithmetic stages travel with the border bit and canonical timing.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_row_timing <= 6'd0;
		end else begin
			reg_row_timing <= {window_border, window_timing};
		end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_sum_timing <= 6'd0;
		end else begin
			reg_sum_timing <= reg_row_timing;
		end
	end

	// Delay arithmetic output to the common twelve-clock algorithm boundary.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			for(cnt_padding = 0; cnt_padding < 8; cnt_padding = cnt_padding + 1)begin
				reg_rgb_pad[cnt_padding] <= 24'd0;
				reg_timing_pad[cnt_padding] <= 6'd0;
			end
		end else begin
			reg_rgb_pad[0] <= masked_rgb;
			reg_timing_pad[0] <= reg_sum_timing;
			for(cnt_padding = 1; cnt_padding < 8; cnt_padding = cnt_padding + 1)begin
				reg_rgb_pad[cnt_padding] <= reg_rgb_pad[cnt_padding - 1];
				reg_timing_pad[cnt_padding] <= reg_timing_pad[cnt_padding - 1];
			end
		end
	end

	// Ten-bit horizontal sums max at 1020; twelve-bit vertical sum max is 4080.
	generate
		for(channel_index = 0; channel_index < 3; channel_index = channel_index + 1)begin:gen_rgb_channel
			reg [11:0] reg_weighted_sum;
			for(row_index = 0; row_index < 3; row_index = row_index + 1)begin:gen_row
				reg [9:0] reg_row_sum;
				always@(posedge i_clk or negedge i_rst_n)begin
					if(!i_rst_n)begin
						reg_row_sum <= 10'd0;
					end else begin
						reg_row_sum <= {2'd0, window_pixels[row_index * 3][channel_index * 8 +: 8]} +
							{1'b0, window_pixels[row_index * 3 + 1][channel_index * 8 +: 8], 1'b0} +
							{2'd0, window_pixels[row_index * 3 + 2][channel_index * 8 +: 8]};
					end
				end
				assign row_sums[channel_index * 3 + row_index] = reg_row_sum;
			end
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					reg_weighted_sum <= 12'd0;
				end else begin
					reg_weighted_sum <= {2'd0, row_sums[channel_index * 3]} +
						{1'b0, row_sums[channel_index * 3 + 1], 1'b0} +
						{2'd0, row_sums[channel_index * 3 + 2]};
				end
			end
			assign normalized_rgb[channel_index * 8 +: 8] = reg_weighted_sum[11:4];
		end
	endgenerate

	video_window_3x3 #(
		.C_DATA_WIDTH(24), .C_FRAME_WIDTH(C_FRAME_WIDTH), .C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_window_3x3_Inst_rgb(
		.i_clk(i_clk), .i_rst_n(i_rst_n),
		.i_vs(i_vs), .i_hs(i_hs), .i_de(i_de), .i_sof(i_sof), .i_eol(i_eol),
		.i_data(i_rgb), .i_x(i_x), .i_y(i_y),
		.o_vs(window_timing[4]), .o_hs(window_timing[3]), .o_de(window_timing[2]),
		.o_sof(window_timing[1]), .o_eol(window_timing[0]), .o_border(window_border),
		.o_p11(window_pixels[0]), .o_p12(window_pixels[1]), .o_p13(window_pixels[2]),
		.o_p21(window_pixels[3]), .o_p22(window_pixels[4]), .o_p23(window_pixels[5]),
		.o_p31(window_pixels[6]), .o_p32(window_pixels[7]), .o_p33(window_pixels[8])
	);
endmodule
