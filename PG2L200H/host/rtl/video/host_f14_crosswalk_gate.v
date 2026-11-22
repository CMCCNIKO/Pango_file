//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 对斑马线几何候选进行时间确认与保持：检查连续帧、横向重叠和底边运动，两帧确认后短时保持，近场场景另加宽限。
// Module Name: host_f14_crosswalk_gate
// File Name: host/rtl/video/host_f14_crosswalk_gate.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 对斑马线几何候选进行时间确认与保持：检查连续帧、横向重叠和底边运动，两帧确认后短时保持，近场场景另加宽限。
// frame_id 使用归一化 30 fps 单位，按模回绕差值计龄，跳帧不能冒充连续候选。
//
// Dependencies: 系统域 i_clk/i_rst_n；来自 host_f14_crosswalk_scan 的候选边界及帧号。
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
// 来源帧步长由调用方归一为30fps；帧号模回绕，跳帧不冒充连续确认。
// 图像门控2帧确认+6帧缺失；近场y>=240再宽限6帧。
`timescale 1ns/1ps
`default_nettype none
module host_f14_crosswalk_gate (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_valid,
	input wire [15:0] i_frame_id,
	input wire i_candidate,
	input wire [9:0] i_left,
	input wire [9:0] i_right,
	input wire [8:0] i_bottom,
	output wire o_valid,
	output wire [15:0] o_frame_id,
	output wire o_active
);
	reg seen_q,previous_q,confirmed_q,held_near_q;
	reg [15:0] previous_frame_q;
	reg [9:0] previous_left_q,previous_right_q;
	reg [8:0] previous_bottom_q;
	reg [2:0] gate_remaining_q,scene_remaining_q;
	reg valid_q,active_q;
	reg [15:0] frame_q;
	wire [15:0] delta=seen_q ? i_frame_id-previous_frame_q : 16'd1;
	wire [9:0] overlap_left=i_left>previous_left_q ? i_left : previous_left_q;
	wire [9:0] overlap_right=i_right<previous_right_q ? i_right : previous_right_q;
	wire [8:0] bottom_motion=i_bottom>=previous_bottom_q ? i_bottom-previous_bottom_q : previous_bottom_q-i_bottom;
	wire near_candidate=i_candidate && i_bottom>=205;
	wire fresh=near_candidate && previous_q && delta==1 && bottom_motion<=24 &&
		overlap_right>=overlap_left+10'd60;
	wire gate_active=fresh || (delta!=0 && delta<=gate_remaining_q);
	wire held_near=fresh ? i_bottom>=240 : held_near_q;
	wire scene_fresh=gate_active && held_near;
	assign o_valid=valid_q;
	assign o_frame_id=frame_q;
	assign o_active=active_q;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			seen_q<=0;previous_q<=0;confirmed_q<=0;held_near_q<=0;previous_frame_q<=0;
			previous_left_q<=0;previous_right_q<=0;previous_bottom_q<=0;
			gate_remaining_q<=0;scene_remaining_q<=0;valid_q<=0;active_q<=0;frame_q<=0;
		end else begin
			valid_q<=i_valid;
			if(i_valid) begin
				seen_q<=1;previous_frame_q<=i_frame_id;frame_q<=i_frame_id;
				previous_q<=near_candidate;confirmed_q<=fresh;
				if(near_candidate) begin previous_left_q<=i_left;previous_right_q<=i_right;previous_bottom_q<=i_bottom;end
				gate_remaining_q<=fresh ? 3'd6 : delta<gate_remaining_q ? gate_remaining_q-delta : 3'd0;
				if(fresh) held_near_q<=i_bottom>=240;
				else if(!gate_active) held_near_q<=0;
				scene_remaining_q<=scene_fresh ? 3'd6 : delta<scene_remaining_q ? scene_remaining_q-delta : 3'd0;
				active_q<=scene_fresh || (delta!=0 && delta<=scene_remaining_q);
			end
		end
	end
endmodule
`default_nettype wire
