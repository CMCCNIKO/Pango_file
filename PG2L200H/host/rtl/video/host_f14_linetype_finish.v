//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 复用一套线型策略和一套实线保护器，顺序处理左右内侧车道，分别输出基线决策和显示类别，外侧类别原样透传。
// Module Name: host_f14_linetype_finish
// File Name: host/rtl/video/host_f14_linetype_finish.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 复用一套线型策略和一套实线保护器，顺序处理左右内侧车道，分别输出基线决策和显示类别，外侧类别原样透传。
// 基线策略之后才施加同帧场景保护；保护输出不能回灌为原始历史证据。
//
// Dependencies: host_f14_linetype_policy、host_f14_solid_protect；i_clk/i_rst_n；frame_id 为 30 fps 单位，原始表在 start 至 done 保持稳定。
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
// 一套策略+一套有界保护，依次完成左右内侧线。外侧类型原样透传。
// frame_id为30fps单位；raw表必须保持至done。保护输出不回灌基线证据。
`timescale 1ns/1ps
`default_nettype none
module host_f14_linetype_finish (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_contiguous,
	input wire [15:0] i_frame_id,
	input wire [839:0] i_x_table,
	input wire [83:0] i_valid_table,
	input wire [7:0] i_raw_types,
	input wire [1:0] i_high,
	input wire [1:0] i_medium,
	input wire [1:0] i_supported,
	input wire [1:0] i_hard,
	input wire [1:0] i_visible,
	input wire [1:0] i_visible_track,
	input wire i_scene,
	output wire o_busy,
	output wire o_done,
	output wire [7:0] o_decision,
	output wire [7:0] o_display
);
	localparam [2:0] ST_IDLE=0,ST_POLICY=1,ST_POLICY_WAIT=2,ST_PROTECT=3,ST_PROTECT_WAIT=4,ST_DONE=5;
	reg [2:0] state_q,state_next;
	reg lane_q,done_q;
	reg [7:0] decision_q,display_q;
	wire policy_done,protect_done;
	wire [1:0] base_decision,base_display,decision,display;
	wire [1:0] candidate=i_supported[lane_q] ? i_raw_types[(lane_q ? 4 : 2) +: 2] : 2'd0;
	assign o_busy=state_q!=ST_IDLE;
	assign o_done=done_q;
	assign o_decision=decision_q;
	assign o_display=display_q;
	host_f14_linetype_policy u_policy(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(state_q==ST_POLICY),
		.i_discontinuity(!i_contiguous),.i_lane(lane_q),.i_frame_id(i_frame_id),.i_candidate(candidate),
		.i_conf_high(i_high[lane_q]),.i_conf_medium(i_medium[lane_q]),.i_hard(i_hard[lane_q]),
		.i_visible(i_visible[lane_q]),.i_visible_track(i_visible_track[lane_q]),
		.o_busy(),.o_done(policy_done),.o_gate_decision(),.o_decision(base_decision),.o_display(base_display));
	host_f14_solid_protect u_protect(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(state_q==ST_PROTECT),
		.i_lane(lane_q),.i_frame_id(i_frame_id),.i_x(i_x_table[(lane_q ? 420 : 210) +: 210]),
		.i_valid(i_valid_table[(lane_q ? 42 : 21) +: 21]),.i_supported(i_supported[lane_q]),
		.i_visible(i_visible[lane_q]),.i_candidate(candidate),.i_confident(i_high[lane_q]),
		.i_base_decision(base_decision),.i_base_display(base_display),.i_scene_active(i_scene),
		.o_busy(),.o_done(protect_done),.o_protected(),.o_decision(decision),.o_display(display));
	always @(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)state_q<=ST_IDLE;else state_q<=state_next;
	end
	always @(*)begin
		state_next=state_q;
		case(state_q)
			ST_IDLE:if(i_start)state_next=ST_POLICY;
			ST_POLICY:state_next=ST_POLICY_WAIT;
			ST_POLICY_WAIT:if(policy_done)state_next=ST_PROTECT;
			ST_PROTECT:state_next=ST_PROTECT_WAIT;
			ST_PROTECT_WAIT:if(protect_done)state_next=lane_q ? ST_DONE : ST_POLICY;
			ST_DONE:state_next=ST_IDLE;
			default:state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin lane_q<=0;done_q<=0;decision_q<=0;display_q<=0;end
		else begin
			done_q<=0;
			if(state_q==ST_IDLE && i_start)begin lane_q<=0;decision_q<=i_raw_types&8'hc3;display_q<=i_raw_types&8'hc3;end
			if(state_q==ST_PROTECT_WAIT && protect_done)begin
				decision_q[(lane_q ? 4 : 2) +: 2]<=decision;display_q[(lane_q ? 4 : 2) +: 2]<=display;lane_q<=1;
			end
			if(state_q==ST_DONE)done_q<=1;
		end
	end
endmodule
`default_nettype wire
