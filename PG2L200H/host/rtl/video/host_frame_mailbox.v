//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 带确认的单项跨时钟邮箱：源端握手锁存任意宽度数据并翻转请求，目的端 valid/ready 接收后返回确认，期间多位数据保持稳定。
// Module Name: host_frame_mailbox
// File Name: host/rtl/video/host_frame_mailbox.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 带确认的单项跨时钟邮箱：源端握手锁存任意宽度数据并翻转请求，目的端 valid/ready 接收后返回确认，期间多位数据保持稳定。
//
// Dependencies: i_src_clk/i_src_rst_n 与 i_dst_clk/i_dst_rst_n；两端需共同断言复位，数据路径延迟与 toggle 同步器需匹配 CDC 约束。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 带确认的单项 CDC 邮箱；多位数据从请求发布到确认返回期间保持稳定。
// 两侧必须共同复位；物理约束需限制数据延迟，并仅豁免 toggle 同步器首级。
`timescale 1ns/1ps
module host_frame_mailbox #(parameter integer C_WIDTH=32) (
	input wire i_src_clk,input wire i_src_rst_n,
	input wire i_valid,output wire o_ready,input wire [C_WIDTH-1:0] i_data,
	input wire i_dst_clk,input wire i_dst_rst_n,
	output wire o_valid,input wire i_ready,output wire [C_WIDTH-1:0] o_data
);
	reg request_q,ack_q;
	reg [1:0] ack_sync_q,request_sync_q;
	reg [C_WIDTH-1:0] data_q;
	assign o_ready=i_src_rst_n && request_q==ack_sync_q[1];
	assign o_valid=i_dst_rst_n && request_sync_q[1]!=ack_q;
	assign o_data=data_q;
	always @(posedge i_src_clk or negedge i_src_rst_n) begin
		if(!i_src_rst_n) begin request_q<=0;ack_sync_q<=0;data_q<=0; end
		else begin
			ack_sync_q<={ack_sync_q[0],ack_q};
			if(i_valid && o_ready) begin data_q<=i_data;request_q<=~request_q; end
		end
	end
	always @(posedge i_dst_clk or negedge i_dst_rst_n) begin
		if(!i_dst_rst_n) begin ack_q<=0;request_sync_q<=0; end
		else begin
			request_sync_q<={request_sync_q[0],request_q};
			if(o_valid && i_ready) begin ack_q<=request_sync_q[1]; end
		end
	end
endmodule
