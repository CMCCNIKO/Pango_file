//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 按参数选择高斯、灰度中值或 Sobel 算法实例，为一段视频算法提供处理分支与旁路选择。
// Module Name: video_stage_tagged
// File Name: source/video/video_stage_tagged.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 按参数选择高斯、灰度中值或 Sobel 算法实例，为一段视频算法提供处理分支与旁路选择。
// RGB 旁路和完整 tag 均固定 12 拍；移位数据不复位，复位有效链屏蔽流水填充气泡。
//
// Dependencies: algo_gaussian_pango、algo_gray_median_pango、algo_sobel_pango。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 旧头“尚未接入顶层”已过时，当前由 video_algorithm_tail 实际调用。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Design Name: HDMI Tagged Algorithm Stage
// Module Name: video_stage_tagged
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 复用已有算法，RGB旁路和完整像素标签均固定12拍；不改变已有存储实现。
// Dependencies: algo_gaussian_pango, algo_gray_median_pango, algo_sobel_pango
// Revision: 0.01 - 创建独立串联适配层，尚未登记/接入HDMI顶层。
// Revision: 0.02 - 标签/RGB旁路无复位移位，12拍有效链恢复完整零气泡。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module video_stage_tagged
#(
	parameter integer C_KIND = 0,
	parameter integer C_FRAME_WIDTH = 1280,
	parameter integer C_FRAME_HEIGHT = 720
)
(
	input i_clk,
	input i_rst_n,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	output [23:0] o_rgb,
	output [159:0] o_tag
);
	// tag={VS,HS,DE,SOF,EOL,x11,y10,tile2,tile_x10,tile_y9,valid,frame32,epoch16,config64}
	// config={stage8,tile8,W8,T_MIN8,A8,flags8,threshold16}；旧版算法位在stage[3:1]，
	// 扩展配置也接受flags[3:1]，两组位在下方作兼容OR。
	// 兼容旧版 v1 stage_mask[63:56]，同时接受扩展配置
	// flags[23:16] 的对应算法位。
	localparam integer LEGACY_ENABLE_BIT = 57 + C_KIND;
	localparam integer FLAGS_ENABLE_BIT = 17 + C_KIND;
	wire [23:0] algorithm_rgb;
	reg [23:0] reg_bypass [0:11];
	reg [159:0] reg_tag [0:11];
	reg [11:0] reg_tag_filled;
	integer cnt_stage;
	wire stage_filled;
	wire algorithm_enable;
	assign stage_filled = i_rst_n && reg_tag_filled[11];
	assign o_tag = stage_filled ? reg_tag[11] : 160'd0;
	assign algorithm_enable = o_tag[LEGACY_ENABLE_BIT] || o_tag[FLAGS_ENABLE_BIT];
	assign o_rgb = stage_filled ? (algorithm_enable ? algorithm_rgb : reg_bypass[11]) : 24'd0;

	// 消隐也推进，选通信号读取与输出像素同标签的配置。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_tag_filled <= 12'd0;
		end else begin
			reg_tag_filled <= {reg_tag_filled[10:0],1'b1};
		end
	end
	// 只卸载纯数据链复位，完整RGB/tag以相同12拍有效链选通。
	always@(posedge i_clk)begin
		reg_bypass[0] <= i_rgb;
		reg_tag[0] <= i_tag;
		for(cnt_stage = 1; cnt_stage < 12; cnt_stage = cnt_stage + 1)begin
			reg_bypass[cnt_stage] <= reg_bypass[cnt_stage - 1];
			reg_tag[cnt_stage] <= reg_tag[cnt_stage - 1];
		end
	end

	generate
		if(C_KIND == 0)begin:gen_gaussian
			algo_gaussian_pango #(
				.C_FRAME_WIDTH(C_FRAME_WIDTH),
				.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
			) algo_gaussian_pango_Inst_stage(
				.i_clk(i_clk),
				.i_rst_n(i_rst_n),
				.i_vs(i_tag[159]),
				.i_hs(i_tag[158]),
				.i_de(i_tag[157]),
				.i_sof(i_tag[156]),
				.i_eol(i_tag[155]),
				.i_rgb(i_rgb),
				.i_x(i_tag[154:144]),
				.i_y(i_tag[143:134]),
				.o_vs(),
				.o_hs(),
				.o_de(),
				.o_sof(),
				.o_eol(),
				.o_rgb(algorithm_rgb)
			);
		end else if(C_KIND == 1)begin:gen_median
			algo_gray_median_pango #(
				.C_FRAME_WIDTH(C_FRAME_WIDTH),
				.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
			) algo_gray_median_pango_Inst_stage(
				.i_clk(i_clk),
				.i_rst_n(i_rst_n),
				.i_vs(i_tag[159]),
				.i_hs(i_tag[158]),
				.i_de(i_tag[157]),
				.i_sof(i_tag[156]),
				.i_eol(i_tag[155]),
				.i_rgb(i_rgb),
				.i_x(i_tag[154:144]),
				.i_y(i_tag[143:134]),
				.o_vs(),
				.o_hs(),
				.o_de(),
				.o_sof(),
				.o_eol(),
				.o_rgb(algorithm_rgb)
			);
		end else begin:gen_sobel
			algo_sobel_pango #(
				.C_FRAME_WIDTH(C_FRAME_WIDTH),
				.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
			) algo_sobel_pango_Inst_stage(
				.i_clk(i_clk),
				.i_rst_n(i_rst_n),
				.i_vs(i_tag[159]),
				.i_hs(i_tag[158]),
				.i_de(i_tag[157]),
				.i_sof(i_tag[156]),
				.i_eol(i_tag[155]),
				.i_rgb(i_rgb),
				.i_x(i_tag[154:144]),
				.i_y(i_tag[143:134]),
				.i_threshold(i_tag[10:0]),
				.o_vs(),
				.o_hs(),
				.o_de(),
				.o_sof(),
				.o_eol(),
				.o_rgb(algorithm_rgb)
			);
		end
	endgenerate
endmodule
