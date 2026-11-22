//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 按显示帧汇总四槽车道有效状态与类型，构造左右条数、实虚线类别及有效位 payload，并为每个输出统计帧分配递增序号。
// Module Name: host_lane_meta_collect
// File Name: host/rtl/video/host_lane_meta_collect.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 按显示帧汇总四槽车道有效状态与类型，构造左右条数、实虚线类别及有效位 payload，并为每个输出统计帧分配递增序号。
// 检查同帧结果一致性和源帧号新鲜度，源结果长期不更新则失效；pending 可由新帧覆盖，邮箱接收后的快照由下游保持。
//
// Dependencies: 像素域 i_clk/i_rst_n；曲线桥结果帧号/available；ready/valid 元数据出口，C_STALE_CYCLES 默认 14850000。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 输出接收器四字包 payload；pending 可被较新帧替换，CDC 接受后保持稳定。
// 结果帧号不前进则失效：148.5MHz 下100ms，74.25MHz下200ms。
`timescale 1ns/1ps
module host_lane_meta_collect #(
	parameter integer C_STALE_CYCLES=14850000
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_pixel_valid,
	input wire [8:0] i_row,
	input wire [9:0] i_col,
	input wire i_frame_done,
	input wire i_result_available,
	input wire [15:0] i_source_id,
	input wire [3:0] i_lane_valid,
	input wire [7:0] i_lane_type,
	output reg o_valid,
	input wire i_ready,
	output reg [31:0] o_seq,
	output reg [31:0] o_payload
);
	reg [3:0] seen_q,seen_next;
	reg [7:0] types_q,types_next;
	reg [15:0] source_q,last_source_q;
	reg available_q,changed_q,have_source_q;
	reg [23:0] age_q;
	reg [31:0] next_seq_q;
	reg [1:0] left_count,right_count,left_type,right_type;
	reg left_type_valid,right_type_valid;
	reg [31:0] payload;
	integer n;
	wire new_source=i_result_available && (!have_source_q || i_source_id!=last_source_q);
	wire fresh=i_result_available && (new_source || (have_source_q && age_q<C_STALE_CYCLES));
	wire coherent=available_q && fresh && !changed_q && source_q==i_source_id;
	always @* begin
		seen_next=seen_q;types_next=types_q;
		if(i_pixel_valid && i_col==0) begin
			if(i_row==0) begin seen_next=0;types_next=0;end
			for(n=0;n<4;n=n+1) begin
				if(i_lane_valid[n]) begin seen_next[n]=1;types_next[n*2+:2]=i_lane_type[n*2+:2];end
			end
		end
		left_count={1'b0,seen_next[0]}+{1'b0,seen_next[1]};
		right_count={1'b0,seen_next[2]}+{1'b0,seen_next[3]};
		left_type=seen_next[1] ? types_next[3:2] : types_next[1:0];
		right_type=seen_next[2] ? types_next[5:4] : types_next[7:6];
		left_type_valid=left_count!=0 && (left_type==1 || left_type==2);
		right_type_valid=right_count!=0 && (right_type==1 || right_type==2);
		payload=0;
		if(coherent) begin
			payload[7:0]={6'd0,left_count};payload[15:8]={6'd0,right_count};
			payload[17:16]=left_type_valid ? left_type : 2'd0;
			payload[19:18]=right_type_valid ? right_type : 2'd0;
			payload[23:20]={right_type_valid,left_type_valid,2'b11};
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			seen_q<=0;types_q<=0;source_q<=0;last_source_q<=0;have_source_q<=0;
			available_q<=0;changed_q<=0;age_q<=C_STALE_CYCLES;
			next_seq_q<=0;o_valid<=0;o_seq<=0;o_payload<=0;
		end else begin
			if(new_source) begin last_source_q<=i_source_id;have_source_q<=1;age_q<=0;end
			else if(age_q<C_STALE_CYCLES) age_q<=age_q+1'b1;
			if(!i_result_available) begin have_source_q<=0;age_q<=C_STALE_CYCLES;end
			if(o_valid && i_ready) o_valid<=0;
			if(i_pixel_valid && i_col==0) begin
				seen_q<=seen_next;types_q<=types_next;
				if(i_row==0) begin source_q<=i_source_id;available_q<=fresh;changed_q<=0;end
				else if(source_q!=i_source_id || !fresh) changed_q<=1;
			end
			if(i_frame_done) begin
				o_valid<=1;o_seq<=next_seq_q;next_seq_q<=next_seq_q+1'b1;o_payload<=payload;
			end
			// 等待邮箱期间也不得保留已经过期的有效识别。
			if(!fresh) o_payload<=0;
		end
	end
endmodule
