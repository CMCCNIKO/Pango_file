//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 管理两个 gather 输入槽与两个计算输出槽的预留、就绪和所有权，使 gather、compute、stream 三阶段可重叠运行。
// Module Name: f14_backbone_overlap_scheduler
// File Name: model/rtl/f14_backbone_overlap_scheduler.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 管理两个 gather 输入槽与两个计算输出槽的预留、就绪和所有权，使 gather、compute、stream 三阶段可重叠运行。
// 随槽保存 tile 几何元数据，严格按握手交接；阶段结束时等待全部在途事务排空后给出 done。
//
// Dependencies: 外部 gather、compute、stream 的 ready/done 接口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 同步低有效复位清空所有槽状态；工作中重复 start 及非法完成事件按原控制报错。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 两个输入槽、两个输出槽；阶段边界必须排空全部事务。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none
module f14_backbone_overlap_scheduler (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_geometry_valid,
	input wire [26:0] i_geometry,
	output wire o_geometry_take,
	output wire o_gather_valid,
	input wire i_gather_ready,
	input wire i_gather_done,
	output wire o_gather_slot,
	output wire o_compute_start,
	input wire i_compute_ready,
	input wire i_compute_done,
	output wire o_compute_input_slot,
	output wire o_compute_output_slot,
	output wire [26:0] o_compute_geometry,
	output wire o_stream_valid,
	input wire i_stream_ready,
	input wire i_stream_done,
	output wire o_stream_slot,
	output wire [26:0] o_stream_geometry,
	output wire o_done,
	output wire o_error
);
	reg active_q, gather_busy_q, compute_busy_q, stream_busy_q;
	reg gather_slot_q, compute_input_q, compute_output_q, stream_slot_q;
	reg [1:0] input_reserved_q, input_ready_q, output_reserved_q, output_ready_q;
	reg [26:0] input_meta0_q, input_meta1_q, output_meta0_q, output_meta1_q;
	reg done_q, error_q;
	wire gather_fire = o_gather_valid && i_gather_ready;
	wire compute_fire = o_compute_start;
	wire stream_fire = o_stream_valid && i_stream_ready;
	assign o_gather_valid = active_q && !gather_busy_q && i_geometry_valid && !input_reserved_q[gather_slot_q];
	assign o_geometry_take = gather_fire;
	assign o_gather_slot = gather_slot_q;
	assign o_compute_start = active_q && !compute_busy_q && input_ready_q[compute_input_q]
		&& !output_reserved_q[compute_output_q] && i_compute_ready;
	assign o_compute_input_slot = compute_input_q;
	assign o_compute_output_slot = compute_output_q;
	assign o_compute_geometry = compute_input_q ? input_meta1_q : input_meta0_q;
	assign o_stream_valid = active_q && !stream_busy_q && output_ready_q[stream_slot_q];
	assign o_stream_slot = stream_slot_q;
	assign o_stream_geometry = stream_slot_q ? output_meta1_q : output_meta0_q;
	assign o_done = done_q;
	assign o_error = error_q;
	integer k;
	always @(posedge i_clk) begin
		if (!i_rst_n) begin
			active_q<=0; gather_busy_q<=0; compute_busy_q<=0; stream_busy_q<=0;
			gather_slot_q<=0; compute_input_q<=0; compute_output_q<=0; stream_slot_q<=0;
			input_reserved_q<=0; input_ready_q<=0; output_reserved_q<=0; output_ready_q<=0;
			done_q<=0; error_q<=0;
		end else begin
			done_q<=0;
			if (i_start) begin
				if (active_q) error_q<=1;
				active_q<=1; gather_busy_q<=0; compute_busy_q<=0; stream_busy_q<=0;
				gather_slot_q<=0; compute_input_q<=0; compute_output_q<=0; stream_slot_q<=0;
				input_reserved_q<=0; input_ready_q<=0; output_reserved_q<=0; output_ready_q<=0;
			end else if (active_q) begin
				if (gather_fire) begin
					gather_busy_q<=1;
					if (gather_slot_q) input_meta1_q<=i_geometry; else input_meta0_q<=i_geometry;
				end
				if (i_gather_done) begin
					if (!gather_busy_q) error_q<=1;
					gather_busy_q<=0; gather_slot_q<=!gather_slot_q;
				end
				if (compute_fire) begin
					compute_busy_q<=1;
					if (compute_output_q) output_meta1_q<=o_compute_geometry; else output_meta0_q<=o_compute_geometry;
				end
				if (i_compute_done) begin
					if (!compute_busy_q) error_q<=1;
					compute_busy_q<=0; compute_input_q<=!compute_input_q; compute_output_q<=!compute_output_q;
				end
				if (stream_fire) stream_busy_q<=1;
				if (i_stream_done) begin
					if (!stream_busy_q) error_q<=1;
					stream_busy_q<=0; stream_slot_q<=!stream_slot_q;
					if (o_stream_geometry[26]) begin active_q<=0; done_q<=1; end
				end
				// 每个槽独立维护；允许三个阶段在同拍释放/接收不同槽。
				for (k=0;k<2;k=k+1) begin
					if (gather_fire && gather_slot_q==k) input_reserved_q[k]<=1;
					if (i_gather_done && gather_slot_q==k) input_ready_q[k]<=1;
					if (compute_fire && compute_input_q==k) input_ready_q[k]<=0;
					if (i_compute_done && compute_input_q==k) input_reserved_q[k]<=0;
					if (compute_fire && compute_output_q==k) output_reserved_q[k]<=1;
					if (i_compute_done && compute_output_q==k) output_ready_q[k]<=1;
					if (stream_fire && stream_slot_q==k) output_ready_q[k]<=0;
					if (i_stream_done && stream_slot_q==k) output_reserved_q[k]<=0;
				end
			end
		end
	end
endmodule
`default_nettype wire
