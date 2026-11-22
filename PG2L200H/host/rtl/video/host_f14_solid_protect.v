//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-26
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 结合两条内侧线的原始候选、基线决策、几何连续性与同帧斑马线场景，对短时实线显示缺失施加有界保护。
// Module Name: host_f14_solid_protect
// File Name: host/rtl/video/host_f14_solid_protect.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 结合两条内侧线的原始候选、基线决策、几何连续性与同帧斑马线场景，对短时实线显示缺失施加有界保护。
// 每次事务处理一条线，串行比较共有锚点并按帧号限制证据年龄；仅原始/基线证据建立历史，保护结果禁止回灌。
//
// Dependencies: i_clk/i_rst_n；i_lane=0/1 对应槽 1/2；输入几何表在事务期间保持，frame_id 使用归一时间单位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-26
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 两条内侧线共用逐点连续性比较；每次事务处理一条线。
// i_lane=0对应槽1，=1对应槽2。scene已由同帧条纹门控产生。
// 仅基线决策/原始候选建立历史；保护后的输出禁止回灌。
`timescale 1ns/1ps
`default_nettype none
module host_f14_solid_protect #(
	parameter [15:0] C_MAX_AGE = 16'd22
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_lane,
	input wire [15:0] i_frame_id,
	input wire [209:0] i_x,
	input wire [20:0] i_valid,
	input wire i_supported,
	input wire i_visible,
	input wire [1:0] i_candidate,
	input wire i_confident,
	input wire [1:0] i_base_decision,
	input wire [1:0] i_base_display,
	input wire i_scene_active,
	output wire o_busy,
	output wire o_done,
	output wire o_protected,
	output wire [1:0] o_decision,
	output wire [1:0] o_display
);
	localparam [2:0] ST_IDLE=3'd0, ST_READ=3'd1, ST_CHECK=3'd2,
		ST_EVAL=3'd3, ST_WRITE=3'd4, ST_DONE=3'd5;
	reg [2:0] state_q, state_next;
	reg lane_q, supported_q, visible_q, confident_q, scene_q;
	reg [15:0] frame_q;
	reg [209:0] x_q;
	reg [20:0] valid_q;
	reg [1:0] candidate_q, base_decision_q, base_display_q;
	reg [4:0] point_q;
	reg [1:0] count_q, common_q;
	reg motion_q;
	reg [9:0] previous_data_q;
	reg [9:0] history_x [0:41] /* synthesis syn_ramstyle="lut_ram" */;
	reg [20:0] history_mask [0:1];
	reg [15:0] last_frame [0:1], solid_frame [0:1];
	reg [2:0] strong_history [0:1];
	reg [1:0] missing [0:1];
	reg [1:0] previous_known_q, solid_known_q;
	reg done_q, protected_q;
	reg [1:0] decision_q, display_q;
	wire [5:0] address;
	wire [9:0] dx;
	wire common_point;
	wire good, stable, strong, new_evidence, have_evidence;
	wire [15:0] frame_delta, evidence_age;
	wire [2:0] shifted_history, updated_history;
	assign o_busy = state_q != ST_IDLE;
	assign o_done = done_q;
	assign o_protected = protected_q;
	assign o_decision = decision_q;
	assign o_display = display_q;
	assign address = (lane_q ? 6'd21 : 6'd0) + {1'b0,point_q};
	assign dx = x_q[9:0] >= previous_data_q ? x_q[9:0]-previous_data_q : previous_data_q-x_q[9:0];
	assign common_point = valid_q[0] && history_mask[lane_q][point_q];
	assign good = supported_q && visible_q && count_q == 3;
	assign frame_delta = frame_q-last_frame[lane_q];
	assign stable = previous_known_q[lane_q] && frame_delta >= 1 && frame_delta <= 3 &&
		common_q == 3 && !motion_q;
	assign strong = candidate_q == 1 && confident_q && base_decision_q == 1;
	assign shifted_history = !stable || (candidate_q == 2 && confident_q) ? 3'd0 :
		frame_delta == 1 ? {strong_history[lane_q][1:0],1'b0} :
		frame_delta == 2 ? {strong_history[lane_q][0],2'b0} : 3'd0;
	assign updated_history = shifted_history | {2'd0,strong};
	assign new_evidence = strong && (updated_history[1] || updated_history[2]);
	assign evidence_age = frame_q-solid_frame[lane_q];
	assign have_evidence = new_evidence || (stable && solid_known_q[lane_q] && evidence_age <= C_MAX_AGE);
	always @(posedge i_clk) begin
		if (state_q == ST_READ) begin
			previous_data_q <= history_x[address];
		end
		if (state_q == ST_WRITE) begin
			history_x[address] <= x_q[9:0];
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin state_q <= ST_IDLE; end
		else begin state_q <= state_next; end
	end
	always @(*) begin
		state_next = state_q;
		case(state_q)
			ST_IDLE: if(i_start) begin state_next = ST_READ; end
			ST_READ: state_next = ST_CHECK;
			ST_CHECK: state_next = point_q == 20 ? ST_EVAL : ST_READ;
			ST_EVAL: state_next = good ? ST_WRITE : ST_DONE;
			ST_WRITE: if(point_q == 20) begin state_next = ST_DONE; end
			ST_DONE: state_next = ST_IDLE;
			default: state_next = ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			lane_q<=0; frame_q<=0; x_q<=0; valid_q<=0;
			supported_q<=0; visible_q<=0; confident_q<=0; scene_q<=0;
			candidate_q<=0; base_decision_q<=0; base_display_q<=0;
			point_q<=0; count_q<=0; common_q<=0; motion_q<=0;
			previous_known_q<=0; solid_known_q<=0;
			history_mask[0]<=0; history_mask[1]<=0;
			last_frame[0]<=0; last_frame[1]<=0; solid_frame[0]<=0; solid_frame[1]<=0;
			strong_history[0]<=0; strong_history[1]<=0; missing[0]<=0; missing[1]<=0;
			done_q<=0; protected_q<=0; decision_q<=0; display_q<=0;
		end else begin
			done_q<=0;
			case(state_q)
				ST_IDLE: if(i_start) begin
					lane_q<=i_lane; frame_q<=i_frame_id; x_q<=i_x; valid_q<=i_valid;
					supported_q<=i_supported; visible_q<=i_visible; confident_q<=i_confident;
					scene_q<=i_scene_active; candidate_q<=i_candidate;
					base_decision_q<=i_base_decision; base_display_q<=i_base_display;
					point_q<=0; count_q<=0; common_q<=0; motion_q<=0;
					protected_q<=0;
				end
				ST_CHECK: begin
					if(valid_q[0] && count_q != 3) begin count_q<=count_q+1'b1; end
					if(common_point) begin
						if(common_q != 3) begin common_q<=common_q+1'b1; end
						if(dx>27) begin motion_q<=1; end
					end
					x_q<={x_q[9:0],x_q[209:10]}; valid_q<={valid_q[0],valid_q[20:1]};
					point_q<=point_q == 20 ? 5'd0 : point_q+1'b1;
				end
				ST_EVAL: begin
					decision_q<=base_decision_q; display_q<=base_display_q;
					if(!good) begin
						if(missing[lane_q]<3) begin missing[lane_q]<=missing[lane_q]+1'b1; end
						if(missing[lane_q]>=2) begin
							previous_known_q[lane_q]<=0; solid_known_q[lane_q]<=0;
							strong_history[lane_q]<=0;
						end
					end else begin
						missing[lane_q]<=0; previous_known_q[lane_q]<=1;
						last_frame[lane_q]<=frame_q; history_mask[lane_q]<=valid_q;
						strong_history[lane_q]<=updated_history;
						if(!stable) begin solid_known_q[lane_q]<=0; end
						if(new_evidence) begin solid_known_q[lane_q]<=1; solid_frame[lane_q]<=frame_q; end
						if(scene_q && have_evidence) begin
							protected_q<=1; decision_q<=1; display_q<=1;
						end
					end
				end
				ST_WRITE: begin
					x_q<={x_q[9:0],x_q[209:10]}; point_q<=point_q+1'b1;
				end
				ST_DONE: done_q<=1;
				default: begin end
			endcase
		end
	end
endmodule
`default_nettype wire
