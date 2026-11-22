//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四宫格行读取规划器，锁存各源显示槽基址与有效性，按左 tile 再右 tile 的行序组成四宫格流，缺失源输出黑色。
// Module Name: quad_mosaic_read_scheduler
// File Name: source/memory/quad_mosaic_read_scheduler.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四宫格行读取规划器，锁存各源显示槽基址与有效性，按左 tile 再右 tile 的行序组成四宫格流，缺失源输出黑色。
// 每 tile 行 640 像素对应 40 个 256-bit word，以 16+16+8 的读段安排 HMIC 请求；请求与返回在 DDR 域按 ready/valid 对接。
//
// Dependencies: ddr_read_burst_engine、quad_frame_address_map.vh、下游 quad DDR 到 HDMI FIFO。
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
//
// Create Date: 2026-09-13
// Design Name: Quad Video Mosaic
// Module Name: quad_mosaic_read_scheduler
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 单个 HMIC 读 master 的四宫格行规划。每 640 像素 tile 行按
//              16+16+8 个 256-bit word 读取；输出行按左 tile 再右 tile。
// Dependencies: ddr_read_burst_engine；下游 FIFO 必须按 ready/valid 收词。
// Revision:
// Revision 0.01 - File Created
// Additional Comments: i_frame_start/slot base/valid 均须在 DDR core_clk 域。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_mosaic_read_scheduler #(
	parameter integer C_URGENT_WATERMARK = 128
)(
	input i_clk,
	input i_rstn,
	input i_frame_start,
	input [111:0] i_display_base,
	input [3:0] i_display_valid,
	input [15:0] i_fifo_free_words,
	input [15:0] i_fifo_used_words,
	input i_output_ready,
	input i_read_engine_busy,
	input i_read_burst_done,
	input i_read_burst_failed,
	input i_read_word_valid,
	input [255:0] i_read_word_data,
	output o_read_word_ready,
	output o_read_burst_req,
	output [27:0] o_read_burst_addr,
	output [3:0] o_read_burst_len,
	output o_output_word_valid,
	output [255:0] o_output_word_data,
	output o_busy,
	output o_frame_done,
	output o_frame_overrun,
	output o_frame_error,
	output o_flush_output_fifo,
	output o_read_urgent,
	output [3:0] o_display_valid_snapshot
);
	localparam [2:0] ST_IDLE = 3'd0;
	localparam [2:0] ST_PREP = 3'd1;
	localparam [2:0] ST_READ = 3'd2;
	localparam [2:0] ST_ZERO = 3'd3;
	localparam [2:0] ST_ADVANCE = 3'd4;
	localparam [2:0] ST_FINISH = 3'd5;
	localparam [2:0] ST_REQUEST = 3'd6;

	reg [2:0] state_current;
	reg [111:0] display_base_snapshot;
	reg [3:0] display_valid_snapshot;
	reg [9:0] row_index;
	reg tile_side;
	reg [1:0] burst_segment;
	reg [4:0] zero_count;
	reg [4:0] read_word_count;
	reg [27:0] read_burst_addr_o;
	reg [3:0] read_burst_len_o;
	reg frame_done_o;
	reg frame_overrun_o;
	reg frame_error_o;
	reg flush_output_fifo_o;
	reg [1:0] source_index;
	reg [27:0] source_base;
	reg [27:0] line_offset;
	reg [27:0] segment_offset;
	reg [4:0] segment_beats;
	wire [9:0] local_row;
	wire source_ready;
	wire read_word_handshake;

	assign local_row = (row_index < 10'd320) ? row_index : (row_index - 10'd320);
	assign source_ready = display_valid_snapshot[source_index];
	// 仅导出实际读取快照，不修改原调度或数据。
	assign o_display_valid_snapshot = display_valid_snapshot;
	assign read_word_handshake = (state_current == ST_READ) &&
		i_read_word_valid && i_output_ready;
	assign o_read_word_ready = (state_current == ST_READ) && i_output_ready;
	// Hold the request until the engine raises busy; a one-cycle pulse can be lost
	// when its capacity guard changes on the accepting edge.
	assign o_read_burst_req = (state_current == ST_REQUEST) && !i_read_engine_busy;
	assign o_read_burst_addr = read_burst_addr_o;
	assign o_read_burst_len = read_burst_len_o;
	assign o_output_word_valid = ((state_current == ST_ZERO) ||
		((state_current == ST_READ) && i_read_word_valid));
	assign o_output_word_data = (state_current == ST_ZERO) ? 256'd0 : i_read_word_data;
	assign o_busy = (state_current != ST_IDLE);
	assign o_frame_done = frame_done_o;
	assign o_frame_overrun = frame_overrun_o;
	assign o_frame_error = frame_error_o;
	assign o_flush_output_fifo = flush_output_fifo_o;
	assign o_read_urgent = (state_current != ST_IDLE) &&
		(i_fifo_used_words < C_URGENT_WATERMARK);

	// 当前输出行只在 top/bottom 二选一，左右 tile 顺序固定。
	always@(*)begin
		source_index = (row_index < 10'd320) ?
			(tile_side ? 2'd1 : 2'd0) :
			(tile_side ? 2'd3 : 2'd2);
		case(source_index)
			2'd0:begin
				source_base = display_base_snapshot[27:0];
			end
			2'd1:begin
				source_base = display_base_snapshot[55:28];
			end
			2'd2:begin
				source_base = display_base_snapshot[83:56];
			end
			default:begin
				source_base = display_base_snapshot[111:84];
			end
		endcase
		// 40 word/line * 8 HMIC 地址单位 = 320 = 256+64。
		line_offset = ({18'd0, local_row} << 8) + ({18'd0, local_row} << 6);
		case(burst_segment)
			2'd0:begin
				segment_offset = 28'd0;
				segment_beats = 5'd16;
			end
			2'd1:begin
				segment_offset = 28'd128;
				segment_beats = 5'd16;
			end
			default:begin
				segment_offset = 28'd256;
				segment_beats = 5'd8;
			end
		endcase
	end

	// 只在 HDMI 输出帧边界接纳并锁存四路显示槽；忙时记录 overrun。
	always@(posedge i_clk or negedge i_rstn)begin
		if(i_rstn == 1'b0)begin
			state_current <= ST_IDLE;
			display_base_snapshot <= 112'd0;
			display_valid_snapshot <= 4'd0;
			row_index <= 10'd0;
			tile_side <= 1'b0;
			burst_segment <= 2'd0;
			zero_count <= 5'd0;
			read_word_count <= 5'd0;
			read_burst_addr_o <= 28'd0;
			read_burst_len_o <= 4'd0;
			frame_done_o <= 1'b0;
			frame_overrun_o <= 1'b0;
			frame_error_o <= 1'b0;
			flush_output_fifo_o <= 1'b0;
		end else begin
			frame_done_o <= 1'b0;
			frame_overrun_o <= 1'b0;
			frame_error_o <= 1'b0;
			flush_output_fifo_o <= 1'b0;
			if((i_frame_start == 1'b1) && (state_current != ST_IDLE))begin
				frame_overrun_o <= 1'b1;
			end
			case(state_current)
				ST_IDLE:begin
					if(i_frame_start == 1'b1)begin
						display_base_snapshot <= i_display_base;
						display_valid_snapshot <= i_display_valid;
						row_index <= 10'd0;
						tile_side <= 1'b0;
						burst_segment <= 2'd0;
						state_current <= ST_PREP;
					end
				end
				ST_PREP:begin
					if(source_ready == 1'b0)begin
						zero_count <= 5'd0;
						state_current <= ST_ZERO;
					end else if((i_read_engine_busy == 1'b0) &&
						(i_fifo_free_words >= {11'd0, segment_beats}))begin
						read_burst_addr_o <= source_base + line_offset + segment_offset;
						read_burst_len_o <= segment_beats[3:0] - 4'd1;
						read_word_count <= 5'd0;
						state_current <= ST_REQUEST;
					end
				end
				ST_REQUEST:begin
					if(i_read_engine_busy == 1'b1)begin
						state_current <= ST_READ;
					end
				end
				ST_READ:begin
					if(read_word_handshake)begin
						read_word_count <= read_word_count + 5'd1;
					end
					if(i_read_burst_failed || i_read_burst_done)begin
						if(i_read_burst_failed ||
							(read_word_count + {4'd0, read_word_handshake} != segment_beats))begin
							// The output FIFO may already hold an incomplete frame.
							frame_error_o <= 1'b1;
							flush_output_fifo_o <= 1'b1;
							state_current <= ST_IDLE;
						end else begin
							state_current <= ST_ADVANCE;
						end
					end
				end
				ST_ZERO:begin
					if(i_output_ready == 1'b1)begin
						if(zero_count == segment_beats - 5'd1)begin
							state_current <= ST_ADVANCE;
						end else begin
							zero_count <= zero_count + 5'd1;
						end
					end
				end
				ST_ADVANCE:begin
					if(burst_segment < 2'd2)begin
						burst_segment <= burst_segment + 2'd1;
						state_current <= ST_PREP;
					end else begin
						burst_segment <= 2'd0;
						if(tile_side == 1'b0)begin
							tile_side <= 1'b1;
							state_current <= ST_PREP;
						end else if(row_index == 10'd639)begin
							state_current <= ST_FINISH;
						end else begin
							tile_side <= 1'b0;
							row_index <= row_index + 10'd1;
							state_current <= ST_PREP;
						end
					end
				end
				ST_FINISH:begin
					frame_done_o <= 1'b1;
					state_current <= ST_IDLE;
				end
				default:begin
					state_current <= ST_IDLE;
				end
			endcase
		end
	end
endmodule
