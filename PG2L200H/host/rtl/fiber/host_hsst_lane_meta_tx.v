//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 在完整视频帧结束后的空隙发送 BEGIN/SEQ/PAYLOAD/END 四字车道元数据包，冻结同包序号与负载并暂缓下一视频帧。
// Module Name: host_hsst_lane_meta_tx
// File Name: host/rtl/fiber/host_hsst_lane_meta_tx.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 在完整视频帧结束后的空隙发送 BEGIN/SEQ/PAYLOAD/END 四字车道元数据包，冻结同包序号与负载并暂缓下一视频帧。
// 忙期间仅更新待发送快照；无新快照时发布未知负载，检测重复帧结束或插包期间非 IDLE 视频冲突。
//
// Dependencies: 发送域 i_clk/i_rst_n；视频 packetizer 的 frame_done、64 位元数据快照与 i_abort。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 接收器四字契约。仅完整视频帧结束后插包；SEQ/PAYLOAD 同包冻结。
`timescale 1ns/1ps
module host_hsst_lane_meta_tx #(
	parameter integer C_WAIT_CYCLES=32
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_abort,
	input wire [31:0] i_video_data,
	input wire [3:0] i_video_k,
	input wire i_frame_done,
	input wire i_meta_valid,
	output wire o_meta_ready,
	input wire [31:0] i_meta_seq,
	input wire [31:0] i_meta_payload,
	output reg [31:0] o_data,
	output reg [3:0] o_k,
	output wire o_hold_next_frame,
	output reg o_error
);
	localparam [2:0] ST_IDLE=0, ST_WAIT=1, ST_BEGIN=2, ST_SEQ=3,
		ST_PAYLOAD=4, ST_END=5, ST_GAP=6;
	reg [2:0] state_q,state_next;
	reg [7:0] wait_q;
	reg pending_q;
	reg [31:0] pending_seq_q,pending_payload_q,seq_q,payload_q;
	wire take_meta=i_meta_valid && o_meta_ready;
	assign o_meta_ready=i_rst_n && !i_abort;
	assign o_hold_next_frame=i_frame_done || state_q!=ST_IDLE;
	always @* begin
		state_next=state_q;
		case(state_q)
			ST_IDLE: if(i_frame_done) state_next=ST_WAIT;
			ST_WAIT: if(wait_q==C_WAIT_CYCLES-1) state_next=ST_BEGIN;
			ST_BEGIN: state_next=ST_SEQ;
			ST_SEQ: state_next=ST_PAYLOAD;
			ST_PAYLOAD: state_next=ST_END;
			ST_END: state_next=ST_GAP;
			ST_GAP: state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
		if(i_abort) state_next=ST_IDLE;
	end
	always @* begin
		o_data=i_video_data;o_k=i_video_k;
		if(!i_abort) begin
			case(state_q)
				ST_BEGIN: begin o_data=32'h4c4d01bc;o_k=4'b0001;end
				ST_SEQ: begin o_data=seq_q;o_k=4'b0000;end
				ST_PAYLOAD: begin o_data=payload_q;o_k=4'b0000;end
				ST_END: begin o_data=32'h4c4dffbc;o_k=4'b0001;end
				ST_GAP: begin o_data=32'h000000bc;o_k=4'b0001;end
				default: begin end
			endcase
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_q<=ST_IDLE;
		else state_q<=state_next;
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			wait_q<=0;pending_q<=0;pending_seq_q<=0;pending_payload_q<=0;
			seq_q<=0;payload_q<=0;o_error<=0;
		end else if(i_abort) begin
			wait_q<=0;pending_q<=0;payload_q<=0;
		end else begin
			// 忙时仅替换待发快照，已冻结包不变。
			if(take_meta) begin
				pending_q<=1;pending_seq_q<=i_meta_seq;pending_payload_q<=i_meta_payload;
			end
			if(state_q==ST_IDLE) wait_q<=0;
			if(state_q==ST_WAIT) begin
				wait_q<=wait_q+1'b1;
				if(wait_q==C_WAIT_CYCLES-1) begin
					seq_q<=take_meta ? i_meta_seq : pending_seq_q;
					// 无新快照主动发布未知，不重复已消费的有效包。
					payload_q<=take_meta ? i_meta_payload : (pending_q ? pending_payload_q : 32'd0);
					pending_q<=0;
				end
			end
			if(i_frame_done && state_q!=ST_IDLE) o_error<=1;
			if(state_q>=ST_BEGIN && (i_video_k!=4'b0001 || i_video_data!=32'h000000bc)) o_error<=1;
		end
	end
endmodule
