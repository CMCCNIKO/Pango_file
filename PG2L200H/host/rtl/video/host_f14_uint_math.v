//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 共享逐拍无符号数学单元：对 40 位数执行恢复除法或整数平方根，分别迭代 40/20 次后给出 done 与结果。
// Module Name: host_f14_uint_math
// File Name: host/rtl/video/host_f14_uint_math.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 共享逐拍无符号数学单元：对 40 位数执行恢复除法或整数平方根，分别迭代 40/20 次后给出 done 与结果。
// 请求只在空闲时接收，忙时忽略新 start；输出调试状态包含忙标志、运算类型及剩余步数。
//
// Dependencies: i_clk/i_rst_n；由调用者保证除法分母有效；不使用动态组合除法。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 逐拍无符号除法/平方根；禁止动态组合除法。忙时不接收新请求。
`timescale 1ns/1ps
`default_nettype none
module host_f14_uint_math (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_sqrt,
	input wire [39:0] i_numerator,
	input wire [39:0] i_denominator,
	output reg o_done,
	output reg [39:0] o_result,
	output wire [7:0] o_debug_state
);
	reg busy_q, sqrt_q;
	reg [5:0] left_q;
	assign o_debug_state={busy_q,sqrt_q,left_q};
	reg [39:0] work_q, quotient_q, denominator_q;
	reg [40:0] remainder_q;
	wire [40:0] shifted = sqrt_q ? {remainder_q[38:0],work_q[39:38]} : {remainder_q[39:0],work_q[39]};
	wire [40:0] trial = sqrt_q ? {quotient_q[38:0],2'b01} : {1'b0,denominator_q};
	wire take = shifted >= trial;
	wire [39:0] next_quotient = {quotient_q[38:0],take};
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			busy_q<=0; sqrt_q<=0; left_q<=0; work_q<=0; quotient_q<=0;
			denominator_q<=0; remainder_q<=0; o_done<=0; o_result<=0;
		end else begin
			o_done<=0;
			if (!busy_q && i_start) begin
				busy_q<=1; sqrt_q<=i_sqrt; left_q<=i_sqrt ? 6'd20 : 6'd40;
				work_q<=i_numerator; denominator_q<=i_denominator;
				quotient_q<=0; remainder_q<=0;
			end else if (busy_q) begin
				work_q<=sqrt_q ? {work_q[37:0],2'b00} : {work_q[38:0],1'b0};
				remainder_q<=take ? shifted-trial : shifted;
				quotient_q<=next_quotient; left_q<=left_q-1'b1;
				if (left_q==1) begin
					busy_q<=0; o_done<=1; o_result<=next_quotient;
				end
			end
		end
	end
endmodule
`default_nettype wire
