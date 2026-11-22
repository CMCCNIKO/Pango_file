//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-26
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 左右内侧线共享的时间策略：合并高/中置信度、原始几何可见性、实虚线证据、待确认、宽限与滞回规则，分别输出门控决策、基线决策及显示类别。
// Module Name: host_f14_linetype_policy
// File Name: host/rtl/video/host_f14_linetype_policy.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 左右内侧线共享的时间策略：合并高/中置信度、原始几何可见性、实虚线证据、待确认、宽限与滞回规则，分别输出门控决策、基线决策及显示类别。
// 每次事务处理一条线，分别保存两条线历史；不连续帧显式清理相关证据，实线/虚线维持时间按参数约束。
//
// Dependencies: i_clk/i_rst_n；frame_id 为归一 30 fps 单位；保护器位于本模块之后。
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
// 合并asymmetric/pending/dashed/grace/hysteresis/display-evidence规则。
// 逐线调用，hard由原始锚点质量支路产生；两个可见性输入保留离线区别。
// frame_id单位必须为30fps来源帧；保护器位于本模块之后。
`timescale 1ns/1ps
`default_nettype none
module host_f14_linetype_policy #(
	parameter [15:0] C_SOLID_HOLD = 16'd30,
	parameter [15:0] C_DASHED_SUSTAIN = 16'd15
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_discontinuity,
	input wire i_lane,
	input wire [15:0] i_frame_id,
	input wire [1:0] i_candidate,
	input wire i_conf_high,
	input wire i_conf_medium,
	input wire i_hard,
	input wire i_visible,
	input wire i_visible_track,
	output wire o_busy,
	output wire o_done,
	output wire [1:0] o_gate_decision,
	output wire [1:0] o_decision,
	output wire [1:0] o_display
);
	localparam [3:0] ST_IDLE=0,ST_PRE=1,ST_GATE=2,ST_DASH_PRE=3,
		ST_DASH=4,ST_GRACE=5,ST_HYST=6,ST_SHOW_PRE=7,ST_SHOW=8,ST_DONE=9;
	reg [3:0] state_q,state_next;
	reg lane_q,high_q,medium_q,hard_q,visible_q,track_q;
	reg discontinuity_q;
	reg [15:0] frame_q;
	reg [1:0] candidate_q,gate_q,decision_q,display_q;
	reg done_q;
	reg [1:0] confirmed [0:1],switch_candidate [0:1],streak [0:1];
	reg [15:0] refreshed [0:1],last_high [0:1];
	reg [4:0] solid_history [0:1];
	reg [1:0] yellow_gap [0:1],grace_gap [0:1],grace_last [0:1];
	reg [2:0] blue_gap [0:1],dashed_age [0:1];
	reg [1:0] dashed_pending [0:1],dashed_hold [0:1];
	reg [1:0] yellow_q,blue_q,hysteresis_q;
	wire [1:0] raw;
	wire [15:0] solid_age,high_age;
	wire [2:0] solid_count;
	integer k;
	assign o_busy=state_q!=ST_IDLE;
	assign o_done=done_q;
	assign o_gate_decision=gate_q;
	assign o_decision=decision_q;
	assign o_display=display_q;
	assign raw=high_q ? candidate_q : 2'd0;
	assign solid_age=frame_q-refreshed[lane_q];
	assign high_age=frame_q-last_high[lane_q];
	assign solid_count={2'd0,solid_history[lane_q][0]}+{2'd0,solid_history[lane_q][1]}+
		{2'd0,solid_history[lane_q][2]}+{2'd0,solid_history[lane_q][3]}+{2'd0,solid_history[lane_q][4]};
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin state_q<=ST_IDLE; end
		else begin state_q<=state_next; end
	end
	always @(*) begin
		state_next=state_q;
		case(state_q)
			ST_IDLE: if(i_start) begin state_next=ST_PRE; end
			ST_PRE: state_next=ST_GATE;
			ST_GATE: state_next=ST_DASH_PRE;
			ST_DASH_PRE: state_next=ST_DASH;
			ST_DASH: state_next=ST_GRACE;
			ST_GRACE: state_next=ST_HYST;
			ST_HYST: state_next=ST_SHOW_PRE;
			ST_SHOW_PRE: state_next=ST_SHOW;
			ST_SHOW: state_next=ST_DONE;
			ST_DONE: state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			lane_q<=0;frame_q<=0;candidate_q<=0;high_q<=0;medium_q<=0;discontinuity_q<=0;
			hard_q<=0;visible_q<=0;track_q<=0;gate_q<=0;decision_q<=0;display_q<=0;done_q<=0;
			yellow_q<=0;blue_q<=0;hysteresis_q<=0;
			for(k=0;k<2;k=k+1) begin
				confirmed[k]<=0;switch_candidate[k]<=0;streak[k]<=0;refreshed[k]<=0;
				last_high[k]<=0;solid_history[k]<=0;yellow_gap[k]<=0;blue_gap[k]<=0;
				grace_gap[k]<=0;grace_last[k]<=0;dashed_age[k]<=7;
				dashed_pending[k]<=0;dashed_hold[k]<=0;
			end
		end else begin
			done_q<=0;
			case(state_q)
				ST_IDLE: if(i_start) begin
					lane_q<=i_lane;frame_q<=i_frame_id;candidate_q<=i_candidate;
					discontinuity_q<=i_discontinuity;
					high_q<=i_conf_high;medium_q<=i_conf_medium;hard_q<=i_hard;
					visible_q<=i_visible;track_q<=i_visible_track;
				end
				ST_PRE: begin
					// 跳帧不累计连续确认，不用旧弱证据填补未观察的来源帧。
					if(discontinuity_q) begin
						switch_candidate[lane_q]<=0;streak[lane_q]<=0;solid_history[lane_q]<=0;
						yellow_gap[lane_q]<=0;blue_gap[lane_q]<=0;grace_gap[lane_q]<=0;grace_last[lane_q]<=0;
						dashed_age[lane_q]<=7;dashed_pending[lane_q]<=0;dashed_hold[lane_q]<=0;
						yellow_q[lane_q]<=0;blue_q[lane_q]<=0;hysteresis_q[lane_q]<=0;
					end
					if((confirmed[lane_q]==2 && raw!=2) ||
						(confirmed[lane_q]!=0 && solid_age>=C_SOLID_HOLD)) begin
						confirmed[lane_q]<=0;
					end
					if(confirmed[lane_q]==2 && raw!=2) begin
						switch_candidate[lane_q]<=0;streak[lane_q]<=0;
					end
				end
				ST_GATE: begin
					gate_q<=confirmed[lane_q];
					if(hard_q) begin
						confirmed[lane_q]<=0;switch_candidate[lane_q]<=0;streak[lane_q]<=0;gate_q<=0;
					end else if(raw!=0) begin
						if(confirmed[lane_q]==raw || confirmed[lane_q]==0 ||
							(switch_candidate[lane_q]==raw && streak[lane_q]>=2)) begin
							confirmed[lane_q]<=raw;gate_q<=raw;refreshed[lane_q]<=frame_q;
							switch_candidate[lane_q]<=0;streak[lane_q]<=0;
						end else begin
							switch_candidate[lane_q]<=raw;
							streak[lane_q]<=switch_candidate[lane_q]==raw ? streak[lane_q]+1'b1 : 2'd1;
						end
					end else begin
						switch_candidate[lane_q]<=0;streak[lane_q]<=0;
					end
				end
				ST_DASH_PRE: begin
					if(!visible_q) begin
						yellow_q[lane_q]<=0;blue_q[lane_q]<=0;yellow_gap[lane_q]<=0;
						blue_gap[lane_q]<=0;solid_history[lane_q]<=0;
					end else if(hard_q) begin
						solid_history[lane_q]<=0;blue_q[lane_q]<=0;blue_gap[lane_q]<=0;
					end else begin
						solid_history[lane_q]<={solid_history[lane_q][3:0],candidate_q==1 && !high_q};
					end
				end
				ST_DASH: begin
					decision_q<=gate_q;display_q<=gate_q;
					if(visible_q) begin
						if(gate_q==2) begin
							blue_q[lane_q]<=1;blue_gap[lane_q]<=0;yellow_q[lane_q]<=0;yellow_gap[lane_q]<=0;
						end else if(gate_q==1) begin
							blue_q[lane_q]<=0;blue_gap[lane_q]<=0;yellow_q[lane_q]<=1;yellow_gap[lane_q]<=0;
						end else if(blue_q[lane_q] && candidate_q==2 && !hard_q && blue_gap[lane_q]<6) begin
							blue_gap[lane_q]<=blue_gap[lane_q]+1'b1;decision_q<=2;display_q<=2;
						end else if(yellow_q[lane_q] && (!hard_q || yellow_gap[lane_q]<2)) begin
							if(yellow_gap[lane_q]!=3) begin yellow_gap[lane_q]<=yellow_gap[lane_q]+1'b1; end
							decision_q<=1;display_q<=1;
						end else if(solid_count>=3 && !hard_q) begin
							yellow_q[lane_q]<=1;yellow_gap[lane_q]<=0;decision_q<=1;display_q<=1;
						end else begin
							blue_q[lane_q]<=0;blue_gap[lane_q]<=0;
							if(hard_q) begin
								if(yellow_gap[lane_q]!=3) begin yellow_gap[lane_q]<=yellow_gap[lane_q]+1'b1; end
								if(yellow_gap[lane_q]>=2) begin yellow_q[lane_q]<=0; end
							end else begin yellow_gap[lane_q]<=0;yellow_q[lane_q]<=0; end
						end
					end
				end
				ST_GRACE: begin
					if(display_q!=0) begin grace_last[lane_q]<=display_q;grace_gap[lane_q]<=0; end
					else if(visible_q && hard_q && grace_last[lane_q]!=0 && grace_gap[lane_q]<1) begin
						decision_q<=grace_last[lane_q];display_q<=grace_last[lane_q];grace_gap[lane_q]<=1;
					end else if(!visible_q || !hard_q) begin grace_last[lane_q]<=0;grace_gap[lane_q]<=0; end
					else if(grace_gap[lane_q]!=3) begin grace_gap[lane_q]<=grace_gap[lane_q]+1'b1; end
				end
				ST_HYST: begin
					if(!track_q || hard_q || candidate_q!=2 || gate_q==1) begin
						hysteresis_q[lane_q]<=0;
						if(display_q==2 && gate_q!=2) begin decision_q<=0;display_q<=0; end
					end else if(gate_q==2 && high_q) begin
						hysteresis_q[lane_q]<=1;last_high[lane_q]<=frame_q;decision_q<=2;display_q<=2;
					end else if(hysteresis_q[lane_q] && medium_q && high_age<=C_DASHED_SUSTAIN) begin
						decision_q<=2;display_q<=2;
					end else begin
						hysteresis_q[lane_q]<=0;
						if(display_q==2) begin decision_q<=0;display_q<=0; end
					end
				end
				ST_SHOW_PRE: begin
					if(!visible_q) begin
						dashed_age[lane_q]<=7;dashed_pending[lane_q]<=0;dashed_hold[lane_q]<=0;
					end else begin
						if(display_q==2) begin dashed_age[lane_q]<=0; end
						else if(dashed_age[lane_q]<6) begin dashed_age[lane_q]<=dashed_age[lane_q]+1'b1; end
						else begin dashed_age[lane_q]<=7; end
						if(candidate_q==1 && medium_q) begin
							dashed_age[lane_q]<=7;dashed_pending[lane_q]<=0;dashed_hold[lane_q]<=0;
						end else if(candidate_q==2 && medium_q) begin
							if(dashed_pending[lane_q]<2) begin dashed_pending[lane_q]<=dashed_pending[lane_q]+1'b1; end
						end else begin dashed_pending[lane_q]<=0; end
					end
				end
				ST_SHOW: begin
					if(visible_q) begin
						if(candidate_q==1 && medium_q) begin
							display_q<=1;
						end else begin
							if(candidate_q==2 && medium_q && dashed_pending[lane_q]>=2 && display_q!=1 && display_q!=2) begin
								display_q<=2;dashed_hold[lane_q]<=1;dashed_pending[lane_q]<=0;
							end else if(display_q==0 && candidate_q!=1 &&
								(dashed_age[lane_q]<6 || dashed_hold[lane_q]!=0)) begin
								display_q<=2;
								if(dashed_hold[lane_q]!=0) begin dashed_hold[lane_q]<=dashed_hold[lane_q]-1'b1; end
							end
						end
					end
				end
				ST_DONE: done_q<=1;
				default: begin end
			endcase
		end
	end
endmodule
`default_nettype wire
