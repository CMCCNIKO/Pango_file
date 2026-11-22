//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 顺序评估两条内侧车道的支持度、原始几何硬证据和可见性；空间后处理点流写入小 RAM，再串行比较点数、跨度、相邻点及帧间运动。
// Module Name: host_f14_linetype_quality
// File Name: host/rtl/video/host_f14_linetype_quality.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 顺序评估两条内侧车道的支持度、原始几何硬证据和可见性；空间后处理点流写入小 RAM，再串行比较点数、跨度、相邻点及帧间运动。
// 独立影子跳变保护用于计算原基线可见性，与重捕获后的几何历史隔离，避免保护输出污染判断。
//
// Dependencies: host_f14_old_coordinates.vh；i_clk/i_rst_n；i_spatial_* 写入流及启动时稳定的锚点表。
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
// 两内侧线共享串行质量判断。空间点从后处理流写小RAM，无额外全表寄存器。
// 独立影子jump guard只计算原基线可见性；与重捕获后的几何历史分开。
`timescale 1ns/1ps
`default_nettype none
module host_f14_linetype_quality (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_spatial_valid,
	input wire [1:0] i_spatial_lane,
	input wire [4:0] i_spatial_row,
	input wire [9:0] i_spatial_x,
	input wire i_spatial_kept,
	input wire [4:0] i_spatial_segment,
	input wire i_start,
	input wire i_contiguous,
	input wire i_error,
	input wire [839:0] i_x_table,
	input wire [83:0] i_valid_table,
	output wire o_busy,
	output wire o_done,
	output wire [1:0] o_supported,
	output wire [1:0] o_hard,
	output wire [1:0] o_visible
);
	localparam [3:0] ST_IDLE=0,ST_READ=1,ST_GUARD=2,ST_CLEAN=3,ST_RAW=4,
		ST_PAIR=5,ST_FINAL=6,ST_DONE=7;
	reg [3:0] state_q,state_next;
	reg lane_q,contiguous_q,error_q,done_q;
	reg [4:0] row_q;
	reg [15:0] spatial[0:41] /* synthesis syn_ramstyle="lut_ram" */;
	reg [25:0] history[0:41] /* synthesis syn_ramstyle="lut_ram" */;
	reg [41:0] initialized_q;
	reg [15:0] spatial_q;
	reg [25:0] history_q;
	reg accepted_q;
	reg [4:0] last_segment_q;
	reg [1:0] run_q;
	reg [1:0] supported_q,hard_q,visible_q;
	reg [1:0] raw_count[0:1];
	reg [9:0] raw_last[0:1],previous_last[0:1];
	reg [8:0] raw_min[0:1],raw_max[0:1],previous_min[0:1],previous_max[0:1];
	reg [1:0] previous_valid_q;
	reg [20:0] common_q,previous_common_q;
	reg [19:0] pair_x[0:20] /* synthesis syn_ramstyle="lut_ram" */;
	reg [1:0] common_count_q,shared_count_q;
	reg [8:0] common_min_q,common_max_q;
	reg [9:0] near_width_q;
	reg widths_good_q,motion_good_q,previous_pair_q,pair_q;
	wire [5:0] address=lane_q ? 6'd21+{1'b0,row_q} : {1'b0,row_q};
	wire [5:0] write_address=i_spatial_lane==2 ? 6'd21+{1'b0,i_spatial_row} : {1'b0,i_spatial_row};
	wire [6:0] raw_index=(lane_q ? 7'd42 : 7'd21)+{2'd0,row_q};
	wire [9:0] raw_x=i_x_table[raw_index*10 +: 10];
	wire raw_valid=i_valid_table[raw_index] && raw_x<640;
	wire [9:0] pair_left=i_x_table[(7'd21+{2'd0,row_q})*10 +: 10];
	wire [9:0] pair_right=i_x_table[(7'd42+{2'd0,row_q})*10 +: 10];
	wire pair_common=i_valid_table[7'd21+{2'd0,row_q}] && i_valid_table[7'd42+{2'd0,row_q}] && pair_left<640 && pair_right<640;
	wire [19:0] old_pair=pair_x[row_q];
	wire [9:0] motion_left=pair_left>=old_pair[9:0] ? pair_left-old_pair[9:0] : old_pair[9:0]-pair_left;
	wire [9:0] motion_right=pair_right>=old_pair[19:10] ? pair_right-old_pair[19:10] : old_pair[19:10]-pair_right;
	wire pair_good=common_count_q>=3 && &visible_q && widths_good_q && common_max_q>=246 &&
		common_max_q-common_min_q>=22 && near_width_q>=40 && near_width_q<=400;
	wire old_known=initialized_q[address] && history_q[25];
	wire [2:0] old_missing=initialized_q[address] ? history_q[24:22] : 3'd0;
	wire [1:0] old_count=initialized_q[address] ? history_q[21:20] : 2'd0;
	wire [9:0] old_last=history_q[19:10],old_pending=history_q[9:0];
	wire [9:0] dx=spatial_q[9:0]>=old_last ? spatial_q[9:0]-old_last : old_last-spatial_q[9:0];
	wire [9:0] pdx=spatial_q[9:0]>=old_pending ? spatial_q[9:0]-old_pending : old_pending-spatial_q[9:0];
	wire [2:0] next_count=old_count!=0 && pdx<=8 ? {1'b0,old_count}+1'b1 : 3'd1;
	wire accept=spatial_q[15] && ((old_known && dx<=18) || next_count>=(old_known ? 3 : 2));
	wire [8:0] y;
	wire [8:0] overlap_low=raw_min[lane_q]>previous_min[lane_q] ? raw_min[lane_q] : previous_min[lane_q];
	wire [8:0] overlap_high=raw_max[lane_q]<previous_max[lane_q] ? raw_max[lane_q] : previous_max[lane_q];
	wire [9:0] last_motion=raw_last[lane_q]>=previous_last[lane_q] ? raw_last[lane_q]-previous_last[lane_q] : previous_last[lane_q]-raw_last[lane_q];
	wire support=raw_count[lane_q]>=3 && ((raw_last[lane_q]>=320)==lane_q || pair_q);
	`include "video/host_f14_old_coordinates.vh"
	assign y=old_anchor_y(row_q);
	assign o_busy=state_q!=ST_IDLE;
	assign o_done=done_q;
	assign o_supported=supported_q;
	assign o_hard=hard_q;
	assign o_visible=visible_q;
	always @(posedge i_clk) begin
		if(i_spatial_valid && (i_spatial_lane==1 || i_spatial_lane==2))
			spatial[write_address]<={i_spatial_kept,i_spatial_segment,i_spatial_x};
		if(state_q==ST_READ)begin spatial_q<=spatial[address];history_q<=history[address];end
		if(state_q==ST_GUARD)begin
			if(!spatial_q[15]) history[address]<={old_known && old_missing<3,old_missing>=4 ? 3'd4 : old_missing+3'd1,2'd0,old_last,old_pending};
			else history[address]<={old_known || accept,3'd0,accept ? 2'd0 : next_count[1:0],accept ? spatial_q[9:0] : old_last,spatial_q[9:0]};
		end
		if(state_q==ST_PAIR)pair_x[row_q]<={pair_right,pair_left};
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n)state_q<=ST_IDLE;else state_q<=state_next;
	end
	always @(*)begin
		state_next=state_q;
		case(state_q)
			ST_IDLE:if(i_start)state_next=ST_READ;
			ST_READ:state_next=ST_GUARD;
			ST_GUARD:state_next=ST_CLEAN;
			ST_CLEAN:state_next=row_q==20 ? (lane_q ? ST_PAIR : ST_READ) : ST_READ;
			ST_PAIR:if(row_q==20)state_next=ST_FINAL;
			ST_FINAL:if(lane_q)state_next=ST_DONE;
			ST_DONE:state_next=ST_IDLE;
			default:state_next=ST_IDLE;
		endcase
	end
	integer k;
	always @(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			lane_q<=0;contiguous_q<=0;error_q<=0;done_q<=0;row_q<=0;accepted_q<=0;last_segment_q<=0;run_q<=0;
			initialized_q<=0;supported_q<=0;hard_q<=0;visible_q<=0;previous_valid_q<=0;
			common_q<=0;previous_common_q<=0;common_count_q<=0;shared_count_q<=0;common_min_q<=0;common_max_q<=0;
			near_width_q<=0;widths_good_q<=1;motion_good_q<=1;previous_pair_q<=0;pair_q<=0;
			for(k=0;k<2;k=k+1)begin raw_count[k]<=0;raw_last[k]<=0;previous_last[k]<=0;raw_min[k]<=0;raw_max[k]<=0;previous_min[k]<=0;previous_max[k]<=0;end
		end else begin
			done_q<=0;
			case(state_q)
				ST_IDLE:if(i_start)begin
					contiguous_q<=i_contiguous;error_q<=i_error;lane_q<=0;row_q<=0;run_q<=0;visible_q<=0;
					raw_count[0]<=0;raw_count[1]<=0;common_q<=0;common_count_q<=0;shared_count_q<=0;
					widths_good_q<=1;motion_good_q<=1;common_min_q<=0;common_max_q<=0;near_width_q<=0;
				end
				ST_GUARD:begin initialized_q[address]<=1;accepted_q<=accept;end
				ST_CLEAN:begin
					if(spatial_q[15])begin
						last_segment_q<=spatial_q[14:10];
						if(!accepted_q)run_q<=0;
						else if(run_q==0 || last_segment_q!=spatial_q[14:10])run_q<=1;
						else begin if(run_q<3)run_q<=run_q+1'b1;if(run_q>=2)visible_q[lane_q]<=1;end
					end
					if(raw_valid)begin
						if(raw_count[lane_q]==0)raw_min[lane_q]<=y;
						if(raw_count[lane_q]<3)raw_count[lane_q]<=raw_count[lane_q]+1'b1;
						raw_max[lane_q]<=y;raw_last[lane_q]<=raw_x;
					end
					if(row_q==20)begin lane_q<=!lane_q;row_q<=0;run_q<=0;end else row_q<=row_q+1'b1;
				end
				ST_PAIR:begin
					common_q[row_q]<=pair_common;
					if(pair_common)begin
						if(common_count_q==0)common_min_q<=y;
						if(common_count_q<3)common_count_q<=common_count_q+1'b1;
						common_max_q<=y;near_width_q<=pair_right-pair_left;
						if(pair_right<pair_left+10'd16)widths_good_q<=0;
						if(previous_common_q[row_q])begin
							if(shared_count_q<3)shared_count_q<=shared_count_q+1'b1;
							if(motion_left>27 || motion_right>27)motion_good_q<=0;
						end
					end
					if(row_q==20)begin
						row_q<=0;lane_q<=0;
					end else row_q<=row_q+1'b1;
				end
				ST_FINAL:begin
					// 第一拍只锁存pair；第二拍更新第一条线，第三拍更新第二条线。
					if(row_q==0)begin
						pair_q<=pair_good && previous_pair_q && contiguous_q && shared_count_q>=3 && motion_good_q;
						previous_pair_q<=pair_good;previous_common_q<=pair_good ? common_q : 21'd0;row_q<=1;
					end else begin
						supported_q[lane_q]<=support && !error_q;
						hard_q[lane_q]<=error_q || !support || (previous_valid_q[lane_q] && contiguous_q &&
							(last_motion>27 || overlap_high<=overlap_low || overlap_high-overlap_low<17));
						previous_valid_q[lane_q]<=support && !error_q;
						previous_last[lane_q]<=raw_last[lane_q];previous_min[lane_q]<=raw_min[lane_q];previous_max[lane_q]<=raw_max[lane_q];
						lane_q<=1;
					end
				end
				ST_DONE:done_q<=1;
				default:begin end
			endcase
		end
	end
endmodule
`default_nettype wire
