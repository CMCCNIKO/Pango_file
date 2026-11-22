//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 独立双路监视画面行规划器，在 start 时冻结完整 source/slot/epoch 快照，依次读取上方 320 行与下方 320 行。
// Module Name: monitor_read_scheduler
// File Name: source/memory/monitor_read_scheduler.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 独立双路监视画面行规划器，在 start 时冻结完整 source/slot/epoch 快照，依次读取上方 320 行与下方 320 行。
// 只影响 monitor 读客户端，保持原四宫格与 PCIe 采集源；缺失源填黑并按下游容量发出读段。
//
// Dependencies: 已预留完整 burst 空间的 DDR 读适配器、monitor DDR 到 HDMI FIFO。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-13
// Module Name: monitor_read_scheduler
// Project Name: PG2L100H HDMI AVM Monitor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 独立双路监视行规划；不改变原quad读源与PCIe。
// Dependencies: 已保留完整burst返回空间的DDR读适配器；官方输出FIFO。
// Contract: DDR域；完整snapshot在start锁存；top 320行后bottom 320行。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module monitor_read_scheduler (
	input i_clk,
	input i_rst_n,
	input i_frame_start,
	input [55:0] i_source_base,
	input [1:0] i_source_present,
	input [15:0] i_fifo_free_words,
	input i_output_ready,
	input i_engine_busy,
	input i_burst_done,
	input i_burst_failed,
	input i_word_valid,
	input [255:0] i_word_data,
	output o_word_ready,
	output o_burst_req,
	output [27:0] o_burst_addr,
	output [3:0] o_burst_len,
	output o_output_valid,
	output [255:0] o_output_data,
	output o_busy,
	output o_frame_done,
	output o_frame_error
);
	localparam [2:0] ST_IDLE=0, ST_PREP=1, ST_REQUEST=2, ST_READ=3, ST_ZERO=4, ST_ADVANCE=5;
	reg [2:0] state_current, state_next;
	reg [55:0] base_snapshot;
	reg [1:0] present_snapshot;
	reg window_index;
	reg [8:0] row_index;
	reg [1:0] segment_index;
	reg [4:0] word_count;
	reg [27:0] burst_addr_o;
	reg [3:0] burst_len_o;
	reg frame_done_o, frame_error_o;
	wire [4:0] segment_beats = (segment_index==2) ? 5'd8 : 5'd16;
	wire [27:0] source_base = window_index ? base_snapshot[55:28] : base_snapshot[27:0];
	wire [27:0] row_offset = ({19'd0,row_index}<<8)+({19'd0,row_index}<<6);
	wire [27:0] segment_offset = {19'd0,segment_index,7'd0};
	wire output_take = o_output_valid && i_output_ready;
	assign o_word_ready=(state_current==ST_READ) && i_output_ready;
	assign o_burst_req=(state_current==ST_REQUEST) && !i_engine_busy;
	assign o_burst_addr=burst_addr_o;
	assign o_burst_len=burst_len_o;
	assign o_output_valid=(state_current==ST_ZERO) || ((state_current==ST_READ) && i_word_valid);
	assign o_output_data=(state_current==ST_ZERO) ? 256'd0 : i_word_data;
	assign o_busy=(state_current!=ST_IDLE);
	assign o_frame_done=frame_done_o;
	assign o_frame_error=frame_error_o;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_current<=ST_IDLE;
		else state_current<=state_next;
	end
	always @(*) begin
		state_next=state_current;
		case(state_current)
			ST_IDLE: if(i_frame_start) state_next=ST_PREP;
			ST_PREP: begin
				if(!present_snapshot[window_index]) state_next=ST_ZERO;
				else if(!i_engine_busy && i_fifo_free_words>={11'd0,segment_beats}) state_next=ST_REQUEST;
			end
			ST_REQUEST: if(i_engine_busy) state_next=ST_READ;
			ST_READ: begin
				if(i_burst_failed) state_next=ST_IDLE;
				else if(i_burst_done) state_next=(word_count+{4'd0,output_take}==segment_beats) ? ST_ADVANCE : ST_IDLE;
			end
			ST_ZERO: if(output_take && word_count==segment_beats-1'b1) state_next=ST_ADVANCE;
			ST_ADVANCE: state_next=(segment_index==2 && row_index==319 && window_index) ? ST_IDLE : ST_PREP;
			default: state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			base_snapshot<=0; present_snapshot<=0; window_index<=0; row_index<=0;
			segment_index<=0; word_count<=0; burst_addr_o<=0; burst_len_o<=0;
			frame_done_o<=0; frame_error_o<=0;
		end else begin
			frame_done_o<=0; frame_error_o<=0;
			if(state_current==ST_IDLE && i_frame_start) begin
				base_snapshot<=i_source_base; present_snapshot<=i_source_present;
				window_index<=0; row_index<=0; segment_index<=0;
			end
			if(state_current==ST_PREP) begin
				word_count<=0;
				burst_addr_o<=source_base+row_offset+segment_offset;
				burst_len_o<=segment_beats[3:0]-1'b1;
			end
			if(output_take) word_count<=word_count+1'b1;
			if(state_current==ST_READ && (i_burst_failed || (i_burst_done && word_count+{4'd0,output_take}!=segment_beats))) frame_error_o<=1;
			if(state_current==ST_ADVANCE) begin
				if(segment_index!=2) segment_index<=segment_index+1'b1;
				else begin
					segment_index<=0;
					if(row_index!=319) row_index<=row_index+1'b1;
					else if(!window_index) begin row_index<=0; window_index<=1; end
					else frame_done_o<=1;
				end
			end
		end
	end
endmodule
