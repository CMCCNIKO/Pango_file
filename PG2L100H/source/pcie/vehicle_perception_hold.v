//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-21
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 对交通灯、行人、障碍、限速、车道、警告及新鲜度等感知字段分别执行可重新触发的阳性保持。
// Module Name: vehicle_perception_hold
// File Name: source/pcie/vehicle_perception_hold.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对交通灯、行人、障碍、限速、车道、警告及新鲜度等感知字段分别执行可重新触发的阳性保持。
// 默认保持 501ms；各字段独立重新计时，scene_valid=0 时清除全部保持结果。
//
// Dependencies: vehicle_behavior_timebase 提供的同域毫秒时间与场景快照。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 2026-11-12 / RK场景字段为0/3时立即清除交通灯或车道保持，允许普通逻辑回退。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-21
// Design Name: Independently retriggered PCIe perception holds
// Module Name: vehicle_perception_hold
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Positive detections hold independently; valid=0 clears all holds.
// Dependencies: Registered PCIe-domain millisecond timestamp.
// Revision: 0.01 - 2026-10-21 Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module vehicle_perception_hold
#(
	parameter [31:0] C_HOLD_MS = 32'd501
)
(
	input i_clk,
	input i_rst_n,
	input i_time_ready,
	input [31:0] i_timestamp_ms,
	input i_scene_commit_valid,
	input [111:0] i_scene_commit_payload,
	output [31:0] o_held_state,
	output [31:0] o_time0,
	output [31:0] o_time1,
	output [15:0] o_sequence,
	output o_scene_ready,
	output o_scene_fresh
);
	// Index 0 traffic, 1 person, 2 obstacle, 3 speed, 4/5 lanes, 6 warning, 7 freshness.
	reg [7:0] dec_refresh;
	reg [7:0] dec_clear;
	reg [7:0] reg_active;
	reg [7:0] reg_age_ready;
	reg [7:0] reg_anchor_pending;
	reg [31:0] reg_last_ms [0:7];
	reg [31:0] reg_elapsed_ms [0:7];
	reg [1:0] reg_traffic;
	reg [1:0] reg_left_lane;
	reg [1:0] reg_right_lane;
	reg [20:0] reg_metadata;
	reg [31:0] time0_o;
	reg [31:0] time1_o;
	reg [15:0] sequence_o;
	reg scene_ready_o;
	reg [31:0] held_state_o;
	integer hold_index;

	assign o_held_state = held_state_o;
	assign o_time0 = time0_o;
	assign o_time1 = time1_o;
	assign o_sequence = sequence_o;
	assign o_scene_ready = scene_ready_o;
	assign o_scene_fresh = reg_active[7];

	always@(*)begin
		dec_refresh = 8'd0;
		dec_clear = 8'd0;
		if(i_scene_commit_valid && i_scene_commit_payload[0])begin
			dec_refresh[0] = i_scene_commit_payload[2:1] == 2'd1 ||
				i_scene_commit_payload[2:1] == 2'd2;
			dec_refresh[1] = i_scene_commit_payload[3];
			dec_refresh[2] = i_scene_commit_payload[4];
			dec_refresh[3] = i_scene_commit_payload[5];
			dec_refresh[4] = i_scene_commit_payload[7:6] == 2'd1 ||
				i_scene_commit_payload[7:6] == 2'd2;
			dec_refresh[5] = i_scene_commit_payload[9:8] == 2'd1 ||
				i_scene_commit_payload[9:8] == 2'd2;
			// 0/3 means RK has no usable recognition for that field; release the
			// corresponding hold immediately so the normal source can resume.
			dec_clear[0] = !(i_scene_commit_payload[2:1] == 2'd1 ||
				i_scene_commit_payload[2:1] == 2'd2);
			dec_clear[4] = !(i_scene_commit_payload[7:6] == 2'd1 ||
				i_scene_commit_payload[7:6] == 2'd2);
			dec_clear[5] = !(i_scene_commit_payload[9:8] == 2'd1 ||
				i_scene_commit_payload[9:8] == 2'd2);
			dec_refresh[6] = i_scene_commit_payload[10];
			dec_refresh[7] = 1'b1;
		end
	end

	// Age subtraction is registered separately from expiry comparison.
	// A refresh invalidates the old arithmetic for one cycle and always wins expiry.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_active <= 8'd0;
			reg_age_ready <= 8'd0;
			reg_anchor_pending <= 8'd0;
			for(hold_index = 0; hold_index < 8; hold_index = hold_index + 1)begin
				reg_last_ms[hold_index] <= 32'd0;
				reg_elapsed_ms[hold_index] <= 32'd0;
			end
		end else if(i_scene_commit_valid && !i_scene_commit_payload[0])begin
			reg_active <= 8'd0;
			reg_age_ready <= 8'd0;
			reg_anchor_pending <= 8'd0;
		end else begin
			for(hold_index = 0; hold_index < 8; hold_index = hold_index + 1)begin
				if(dec_refresh[hold_index])begin
					reg_last_ms[hold_index] <= i_timestamp_ms;
					reg_active[hold_index] <= 1'b1;
					reg_age_ready[hold_index] <= 1'b0;
					reg_anchor_pending[hold_index] <= !i_time_ready;
				end else if(dec_clear[hold_index])begin
					reg_active[hold_index] <= 1'b0;
					reg_age_ready[hold_index] <= 1'b0;
					reg_anchor_pending[hold_index] <= 1'b0;
				end else if(i_time_ready)begin
					// A startup event may precede the first synchronized source timestamp.
					if(reg_anchor_pending[hold_index])begin
						reg_last_ms[hold_index] <= i_timestamp_ms;
						reg_anchor_pending[hold_index] <= 1'b0;
						reg_age_ready[hold_index] <= 1'b0;
					end else begin
						reg_elapsed_ms[hold_index] <= i_timestamp_ms - reg_last_ms[hold_index];
						reg_age_ready[hold_index] <= 1'b1;
						if(reg_age_ready[hold_index] && reg_elapsed_ms[hold_index] >= C_HOLD_MS)begin
							reg_active[hold_index] <= 1'b0;
						end
					end
				end
			end
		end
	end

	// Time, sequence and non-perception flags belong to the same accepted commit.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_traffic <= 2'd0;
			reg_left_lane <= 2'd0;
			reg_right_lane <= 2'd0;
			reg_metadata <= 21'd0;
			time0_o <= 32'h10102026;
			time1_o <= 32'h00235716;
			sequence_o <= 16'd0;
			scene_ready_o <= 1'b0;
		end else if(i_scene_commit_valid)begin
			reg_metadata <= i_scene_commit_payload[31:11];
			time0_o <= i_scene_commit_payload[63:32];
			time1_o <= i_scene_commit_payload[95:64];
			sequence_o <= i_scene_commit_payload[111:96];
			scene_ready_o <= 1'b1;
			if(dec_refresh[0] || dec_clear[0])begin
				reg_traffic <= i_scene_commit_payload[2:1];
			end
			if(dec_refresh[4] || dec_clear[4])begin
				reg_left_lane <= i_scene_commit_payload[7:6];
			end
			if(dec_refresh[5] || dec_clear[5])begin
				reg_right_lane <= i_scene_commit_payload[9:8];
			end
		end
	end
	always@(*)begin
		held_state_o = {reg_metadata, 11'd0};
		held_state_o[0] = reg_active[7];
		held_state_o[2:1] = reg_active[0] ? reg_traffic : 2'd0;
		held_state_o[3] = reg_active[1];
		held_state_o[4] = reg_active[2];
		held_state_o[5] = reg_active[3];
		held_state_o[7:6] = reg_active[4] ? reg_left_lane : 2'd0;
		held_state_o[9:8] = reg_active[5] ? reg_right_lane : 2'd0;
		held_state_o[10] = reg_active[6];
		if(!scene_ready_o)begin
			held_state_o = 32'h00000a41;
		end
	end
endmodule
