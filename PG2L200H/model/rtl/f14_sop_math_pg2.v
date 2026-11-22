//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 保留三个在用PG2可综合算术兼容模块：四项U8乘加，以及signed17/34×unsigned16量化乘法；signed18模块已归档。
// Module Name: npu_multadd4_u8, npu_mult17s16u_pq, npu_mult34s16u_pq
// File Name: model/rtl/f14_sop_math_pg2.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.01（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 保留三个在用PG2可综合算术兼容模块：四项U8乘加，以及signed17/34×unsigned16量化乘法；signed18模块已归档。
// 保留旧接口名、使能和三级寄存结构，以RTL推断PG2算术资源，供SOP MAC及PQ流水使用。
//
// Dependencies: npu_multadd4_u8、npu_mult17s16u_pq、npu_mult34s16u_pq
//
// Revision:
// 2026-10-30 / Header 1.01 - 未用模块移入other_file，其余模块实现及指令保持不变。
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 此文件包含3个模块且无复位端口；打包保留原运算符和固定寄存延迟，不把它描述成IP Compiler新建官方IP。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`default_nettype none

// PG2-compatible replacements for the legacy SOP altmult_add wrappers.
// These keep the original port names and registered latency while allowing
// PDS to infer the PG2 arithmetic resources.  They are deliberately separate
// from the ModelSim-only test stubs; the final DSP implementation can replace
// these modules without touching the SOP tile or bridge ABI.
module npu_multadd4_u8 (
    input  wire        clock0,
    input  wire [7:0]  dataa_0,
    input  wire [7:0]  dataa_1,
    input  wire [7:0]  dataa_2,
    input  wire [7:0]  dataa_3,
    input  wire [7:0]  datab_0,
    input  wire [7:0]  datab_1,
    input  wire [7:0]  datab_2,
    input  wire [7:0]  datab_3,
    input  wire        ena0,
    output reg  [17:0] result
);
    reg [17:0] p0_q, p1_q;
    always @(posedge clock0) begin
        if (ena0) begin
            p0_q  <= (dataa_0 * datab_0) + (dataa_1 * datab_1)
                    + (dataa_2 * datab_2) + (dataa_3 * datab_3);
            p1_q  <= p0_q;
            result <= p1_q;
        end
    end
endmodule

module npu_mult17s16u_pq (
    input  wire         clock0,
    input  wire [16:0]  dataa_0,
    input  wire [15:0]  datab_0,
    input  wire         ena0,
    output reg  [32:0]  result
);
    reg signed [32:0] p0_q, p1_q;
    wire signed [32:0] dataa_ext = {{16{dataa_0[16]}}, dataa_0};
    wire signed [32:0] datab_ext = {17'd0, datab_0};
    always @(posedge clock0) begin
        if (ena0) begin
            p0_q <= dataa_ext * datab_ext;
            p1_q <= p0_q;
            result <= p1_q;
        end
    end
endmodule


// 量化主乘法：signed34 x unsigned16，完整 50 位结果，保留原三拍延迟。
module npu_mult34s16u_pq (
	input wire clock0,
	input wire [33:0] dataa_0,
	input wire [15:0] datab_0,
	input wire ena0,
	output reg [49:0] result
);
	reg signed [49:0] p0_q;
	reg signed [49:0] p1_q;
	wire signed [49:0] dataa_ext = {{16{dataa_0[33]}}, dataa_0};
	wire signed [49:0] datab_ext = {34'd0, datab_0};
	always @(posedge clock0) begin
		if (ena0) begin
			p0_q <= dataa_ext * datab_ext;
			p1_q <= p0_q;
			result <= p1_q;
		end
	end
endmodule
`default_nettype wire
