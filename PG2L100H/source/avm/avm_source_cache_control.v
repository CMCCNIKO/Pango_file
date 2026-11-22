//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四源 64KiB 直映像素缓存控制器，将 LUT 邻域像素请求转换为 data/tag 查找、未命中 DDR 单 beat 填充与重放。
// Module Name: avm_source_cache_control
// File Name: source/avm/avm_source_cache_control.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四源 64KiB 直映像素缓存控制器，将 LUT 邻域像素请求转换为 data/tag 查找、未命中 DDR 单 beat 填充与重放。
// 同步 RAM 读延迟为一拍，tag 包含 valid、frame_epoch 与像素高地址；连续命中可一拍一请求，cancel 只排空业务，不清已接受的 DDR 事务。
//
// Dependencies: avm_source_cache_ram、avm_source_cache_tag_ram 的真实 RAM 端口、ddr_read_burst_engine。
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
// Module Name: avm_source_cache_control
// Project Name: PG2L100H HDMI AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 64KiB四source直映缓存控制；纯控制，不实例化或替代尚未生成RAM。
// Contract: tag/data普通同步读1拍；lookup并行，连续hit一拍一请求。
//           tag={10'd0,valid,frame_epoch[15:0],pixel_addr[17:13]}。
//           全局DDR reset外，cancel不清DDR adapter，已接受burst必须排空。
// Dependencies: 真实RAM wrapper + ddr_read_burst_engine，生成后独立接入。
// Revision: 2026-10-23 仅REPLAY_READ选冻结地址，正常预取不经过ready/valid组合路径。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module avm_source_cache_control (
	input i_clk,
	input i_rst_n,
	input i_cancel,
	input i_invalidate,
	input i_request_valid,
	output o_request_ready,
	input [1:0] i_request_source,
	input [17:0] i_request_pixel_addr,
	input [15:0] i_request_frame_epoch,
	input [27:0] i_request_source_base,
	output o_response_valid,
	input i_response_ready,
	output [15:0] o_response_rgb565,
	output o_response_error,
	output o_initialized,
	output o_idle,
	output o_tag_read_enable,
	output [10:0] o_tag_read_addr,
	input [31:0] i_tag_read_data,
	output o_data_read_enable,
	output [14:0] o_data_read_addr,
	input [15:0] i_data_read_data,
	output o_tag_write_enable,
	output [10:0] o_tag_write_addr,
	output [31:0] o_tag_write_data,
	output o_data_write_enable,
	output [10:0] o_data_write_addr,
	output [255:0] o_data_write_data,
	output o_burst_request,
	output [27:0] o_burst_addr,
	output [3:0] o_burst_len,
	input i_engine_busy,
	input i_burst_done,
	input i_burst_failed,
	input i_word_valid,
	input [255:0] i_word_data,
	output o_word_ready,
	output [31:0] o_hit_count,
	output [31:0] o_miss_count
);
	localparam [2:0] ST_CLEAR=0, ST_IDLE=1, ST_LOOKUP=2, ST_REQUEST=3,
		ST_FILL=4, ST_REPLAY_READ=5, ST_REPLAY_REPLY=6;
	reg [2:0] state_current,state_next;
	reg [10:0] clear_index;
	reg [1:0] request_source;
	reg [17:0] request_pixel_addr;
	reg [15:0] request_epoch;
	reg [27:0] request_base;
	reg cancel_pending,fill_received,failed_pending;
	reg response_valid_o,response_error_o;
	reg [15:0] response_rgb_o;
	reg [31:0] hit_count_o,miss_count_o;
	wire response_space=!response_valid_o || i_response_ready;
	wire tag_hit=i_tag_read_data[21] && i_tag_read_data[20:5]==request_epoch &&
		i_tag_read_data[4:0]==request_pixel_addr[17:13];
	wire accept_request=i_request_valid && o_request_ready;
	wire fill_take=(state_current==ST_FILL) && i_word_valid;
	assign o_request_ready=!i_cancel && !i_invalidate && response_space && ((state_current==ST_IDLE) ||
		(state_current==ST_LOOKUP && tag_hit && !cancel_pending) ||
		(state_current==ST_REPLAY_REPLY && !cancel_pending));
	assign o_response_valid=response_valid_o;
	assign o_response_rgb565=response_rgb_o;
	assign o_response_error=response_error_o;
	assign o_initialized=state_current!=ST_CLEAR;
	assign o_idle=state_current==ST_IDLE && !response_valid_o;
	assign o_tag_read_enable=accept_request;
	assign o_tag_read_addr={i_request_source,i_request_pixel_addr[12:4]};
	assign o_data_read_enable=accept_request || state_current==ST_REPLAY_READ;
	assign o_data_read_addr=(state_current==ST_REPLAY_READ) ?
		{request_source,request_pixel_addr[12:0]} : {i_request_source,i_request_pixel_addr[12:0]};
	assign o_tag_write_enable=(state_current==ST_CLEAR) || (fill_take && !cancel_pending && !i_cancel);
	assign o_tag_write_addr=(state_current==ST_CLEAR) ? clear_index : {request_source,request_pixel_addr[12:4]};
	assign o_tag_write_data=(state_current==ST_CLEAR) ? 32'd0 : {10'd0,1'b1,request_epoch,request_pixel_addr[17:13]};
	assign o_data_write_enable=fill_take && !cancel_pending && !i_cancel;
	assign o_data_write_addr={request_source,request_pixel_addr[12:4]};
	assign o_data_write_data=i_word_data;
	assign o_burst_request=(state_current==ST_REQUEST) && !i_engine_busy && !cancel_pending && !i_cancel;
	assign o_burst_addr=request_base+{11'd0,request_pixel_addr[17:4],3'd0};
	assign o_burst_len=4'd0;
	assign o_word_ready=state_current==ST_FILL;
	assign o_hit_count=hit_count_o;
	assign o_miss_count=miss_count_o;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_current<=ST_CLEAR;
		else state_current<=state_next;
	end
	always @(*) begin
		state_next=state_current;
		case(state_current)
			ST_CLEAR: if(clear_index==2047) state_next=ST_IDLE;
			ST_IDLE: begin
				if(i_invalidate && !response_valid_o) state_next=ST_CLEAR;
				else if(accept_request) state_next=ST_LOOKUP;
			end
			ST_LOOKUP: begin
				if(cancel_pending || i_cancel) begin if(response_space) state_next=ST_IDLE; end
				else if(tag_hit) begin
					if(response_space) state_next=accept_request ? ST_LOOKUP : ST_IDLE;
				end else state_next=ST_REQUEST;
			end
			ST_REQUEST: begin
				if(i_engine_busy) state_next=ST_FILL;
				else if((cancel_pending || i_cancel) && response_space) state_next=ST_IDLE;
			end
			ST_FILL: begin
				if(i_burst_failed || failed_pending) begin if(response_space) state_next=ST_IDLE; end
				else if(i_burst_done && (fill_received || fill_take)) state_next=ST_REPLAY_READ;
			end
			ST_REPLAY_READ: state_next=ST_REPLAY_REPLY;
			ST_REPLAY_REPLY: if(response_space) state_next=accept_request ? ST_LOOKUP : ST_IDLE;
			default: state_next=ST_CLEAR;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			clear_index<=0; request_source<=0; request_pixel_addr<=0; request_epoch<=0; request_base<=0;
			cancel_pending<=0; fill_received<=0; failed_pending<=0; response_valid_o<=0; response_error_o<=0;
			response_rgb_o<=0; hit_count_o<=0; miss_count_o<=0;
		end else begin
			if(state_current==ST_CLEAR && clear_index!=2047) clear_index<=clear_index+1'b1;
			if((i_invalidate && o_idle) || state_current==3'd7) clear_index<=0;
			if(response_valid_o && i_response_ready) response_valid_o<=0;
			if(i_cancel && state_current!=ST_IDLE && state_current!=ST_CLEAR) cancel_pending<=1;
			if(accept_request) begin
				request_source<=i_request_source; request_pixel_addr<=i_request_pixel_addr;
				request_epoch<=i_request_frame_epoch; request_base<=i_request_source_base;
				cancel_pending<=0; fill_received<=0; failed_pending<=0;
			end
			if(state_current==ST_LOOKUP && !cancel_pending && !i_cancel) begin
				if(tag_hit && response_space) hit_count_o<=hit_count_o+1'b1;
				else if(!tag_hit) miss_count_o<=miss_count_o+1'b1;
			end
			if(fill_take) fill_received<=1;
			if(state_current==ST_FILL && i_burst_failed) failed_pending<=1;
			if(state_current==ST_LOOKUP && response_space && (tag_hit || cancel_pending || i_cancel)) begin
				response_valid_o<=1; response_rgb_o<=(cancel_pending || i_cancel) ? 16'd0 : i_data_read_data;
				response_error_o<=cancel_pending || i_cancel;
			end
			if(state_current==ST_REPLAY_REPLY && response_space) begin
				response_valid_o<=1; response_rgb_o<=(cancel_pending || i_cancel) ? 16'd0 : i_data_read_data;
				response_error_o<=cancel_pending || i_cancel;
			end
			if(state_current==ST_FILL && (i_burst_failed || failed_pending) && response_space) begin
				response_valid_o<=1; response_rgb_o<=0; response_error_o<=1;
			end
			if(state_current==ST_REQUEST && !i_engine_busy && (cancel_pending || i_cancel) && response_space) begin
				response_valid_o<=1; response_rgb_o<=0; response_error_o<=1;
			end
		end
	end
endmodule
