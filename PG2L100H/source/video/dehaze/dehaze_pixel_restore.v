//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 根据暗通道、大气光及用户 W/Tmin 计算透射率，经局部导向细化后对 RGB888 执行精确整数去雾恢复。
// Module Name: dehaze_pixel_restore
// File Name: source/video/dehaze/dehaze_pixel_restore.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 根据暗通道、大气光及用户 W/Tmin 计算透射率，经局部导向细化后对 RGB888 执行精确整数去雾恢复。
// 固定 66 拍、II=1；恢复增益、各旁路与输入 tag 等拍传播，保留未启用像素的原始颜色。
//
// Dependencies: dehaze_guided_filter、dehaze_divide_exact、官方 dehaze_mul8x8 及相关 ROM/乘法 IP。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Module: dehaze_pixel_restore
// Project: PG2L100H HDMI dehaze; PDS 2022.2-SP6.4
// Device: PG2L100H-6FBG676
// Description: RGB888精确整数恢复，66拍II=1；所有旁路携带同拍tag。
// Dependencies: 本次官方乘法/ROM；dehaze_divide_exact。
// Revision: 1.0 初始实现。
// Revision: 1.1 正则项参数透传；保留用户W/Tmin及原恢复增益配置。
//////////////////////////////////////////////////////////////////////////////
module dehaze_pixel_restore
#(
	parameter [15:0] C_GUIDED_EPSILON = 16'd256
)
(
	input wire i_clk,
	input wire i_rst_n,
	input wire [23:0] i_rgb,
	input wire [159:0] i_tag,
	input wire [7:0] i_dark,
	input wire [7:0] i_atmosphere,
	input wire i_a_valid,
	output wire [23:0] o_rgb,
	output wire [159:0] o_tag
);
	// metadata={apply,A,RGB,tag}。预热/未选择/黑条同延迟原图旁路。
	wire [1:0] tile = i_tag[133:132];
	// stage_mask[0] 是协议定义的去雾总开关；flags[0] 仅控制自动大气光估计。
	// 保持默认配置的去雾关闭语义，避免把 auto-A 误当成算法使能。
	wire apply_pixel = i_tag[157] && i_tag[112] &&
		i_tag[56] && i_tag[48+tile] && i_a_valid;
	wire [7:0] atmosphere_safe = i_atmosphere==0 ? 8'd1 : i_atmosphere;
	wire [192:0] metadata_in = {apply_pixel,atmosphere_safe,i_rgb,i_tag};
	reg [192:0] wd_meta [0:2];
	wire [15:0] wd;
	wire [16:0] q_dark;
	wire [192:0] q_meta;
	reg [7:0] transmission;
	reg [192:0] transmission_meta;
	wire [7:0] refined_t;
	wire [192:0] refined_meta;
	wire [7:0] refined_min = refined_meta[39:32] == 0 ? 8'd1 : refined_meta[39:32];
	wire [7:0] refined_safe = refined_t < refined_min ? refined_min : refined_t;
	wire [7:0] t_min = q_meta[39:32]==0 ? 8'd1 : q_meta[39:32];
	wire [7:0] t_candidate = q_dark>=255 ? 8'd0 : 8'd255-q_dark[7:0];
	wire [15:0] attenuation;
	reg [7:0] t_pipe [0:2];
	reg [192:0] restore_meta [0:2];
	reg [15:0] numerator_r;
	reg [15:0] numerator_g;
	reg [15:0] numerator_b;
	reg [7:0] restore_t;
	reg [192:0] numerator_meta;
	wire [23:0] delayed_rgb = restore_meta[2][183:160];
	wire [15:0] scaled_r = {delayed_rgb[23:16],8'd0} - {8'd0,delayed_rgb[23:16]};
	wire [15:0] scaled_g = {delayed_rgb[15:8],8'd0} - {8'd0,delayed_rgb[15:8]};
	wire [15:0] scaled_b = {delayed_rgb[7:0],8'd0} - {8'd0,delayed_rgb[7:0]};
	wire [16:0] q_r;
	wire [16:0] q_g;
	wire [16:0] q_b;
	wire [192:0] final_meta;
	reg [23:0] rgb_o;
	reg [159:0] tag_o;
	integer k;
	dehaze_mul8x8 u_wd(.a(i_tag[47:40]),.b(i_dark),.p(wd),.clk(i_clk),.rst(~i_rst_n),.ce(1'b1));
	dehaze_divide_exact #(.C_TAG_WIDTH(193)) u_dark_divide(
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_numerator(wd),.i_denominator(wd_meta[2][191:184]),
		.i_tag(wd_meta[2]),.o_quotient(q_dark),.o_tag(q_meta)
	);
	dehaze_guided_filter #(.C_GUIDED_EPSILON(C_GUIDED_EPSILON)) u_guided(
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_t(transmission),
		.i_rgb(transmission_meta[183:160]),.i_meta(transmission_meta),.o_t(refined_t),.o_meta(refined_meta));
	dehaze_mul8x8 u_attenuation(.a(refined_meta[191:184]),.b(8'd255-refined_safe),
		.p(attenuation),.clk(i_clk),.rst(~i_rst_n),.ce(1'b1));
	dehaze_divide_exact #(.C_TAG_WIDTH(193)) u_restore_r(
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_numerator(numerator_r),.i_denominator(restore_t),
		.i_tag(numerator_meta),.o_quotient(q_r),.o_tag(final_meta));
	dehaze_divide_exact #(.C_TAG_WIDTH(193)) u_restore_g(
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_numerator(numerator_g),.i_denominator(restore_t),
		.i_tag(numerator_meta),.o_quotient(q_g),.o_tag());
	dehaze_divide_exact #(.C_TAG_WIDTH(193)) u_restore_b(
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_numerator(numerator_b),.i_denominator(restore_t),
		.i_tag(numerator_meta),.o_quotient(q_b),.o_tag());
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			for(k=0;k<3;k=k+1) begin wd_meta[k]<=0; restore_meta[k]<=0; t_pipe[k]<=1; end
			transmission<=1; transmission_meta<=0;
			numerator_r<=0; numerator_g<=0; numerator_b<=0; restore_t<=1; numerator_meta<=0;
			rgb_o<=0; tag_o<=0;
		end else begin
			wd_meta[0]<=metadata_in;
			for(k=1;k<3;k=k+1) wd_meta[k]<=wd_meta[k-1];
			transmission <= t_candidate<t_min ? t_min : t_candidate;
			transmission_meta <= q_meta;
			t_pipe[0]<=refined_safe; restore_meta[0]<=refined_meta;
			for(k=1;k<3;k=k+1) begin t_pipe[k]<=t_pipe[k-1]; restore_meta[k]<=restore_meta[k-1]; end
			numerator_r <= scaled_r>=attenuation ? scaled_r-attenuation : 16'd0;
			numerator_g <= scaled_g>=attenuation ? scaled_g-attenuation : 16'd0;
			numerator_b <= scaled_b>=attenuation ? scaled_b-attenuation : 16'd0;
			restore_t<=t_pipe[2]; numerator_meta<=restore_meta[2];
			tag_o<=final_meta[159:0];
			if(final_meta[192]) rgb_o <= {q_r>=255 ? 8'd255 : q_r[7:0],q_g>=255 ? 8'd255 : q_g[7:0],q_b>=255 ? 8'd255 : q_b[7:0]};
			else rgb_o<=final_meta[183:160];
		end
	end
	assign o_rgb=rgb_o;
	assign o_tag=tag_o;
endmodule
