//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 冻结一个 LUT entry 后按序请求两路相机共八个双线性邻域像素，收齐后送入 avm_pixel_lane，并使算术流水与下一条 LUT 邻域采集重叠。
// Module Name: avm_neighbor_gather
// File Name: source/avm/avm_neighbor_gather.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 冻结一个 LUT entry 后按序请求两路相机共八个双线性邻域像素，收齐后送入 avm_pixel_lane，并使算术流水与下一条 LUT 邻域采集重叠。
// 渲染期间 source base/present/epoch 必须由 lease 固定，cache 响应须按请求顺序；相机角色映射由独立固定参数决定。
//
// Dependencies: avm_pixel_lane、avm_source_cache 请求/响应接口。
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
// Module Name: avm_neighbor_gather
// Project Name: PG2L100H HDMI AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 冻结LUT→8邻域有序请求→已golden验证单lane；算术与下个LUT gather重叠。
// Dependencies: avm_pixel_lane；cache wrapper生成后接cache request/response。
// Contract: 每render的source base/present/epoch必须已lease冻结；cache返回按序。
//           摄像机角色映射独立固定参数，不使用monitor route。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module avm_neighbor_gather #(
	parameter [1:0] C_SOURCE_SLOT0=2'd0,
	parameter [1:0] C_SOURCE_SLOT1=2'd1,
	parameter [1:0] C_SOURCE_SLOT2=2'd2,
	parameter [1:0] C_SOURCE_SLOT3=2'd3
)(
	input i_clk,
	input i_rst_n,
	input i_lut_valid,
	output o_lut_ready,
	input [95:0] i_lut,
	input [18:0] i_lut_index,
	input [111:0] i_source_base,
	input [63:0] i_source_epoch,
	input [3:0] i_source_present,
	output o_cache_request_valid,
	input i_cache_request_ready,
	output [1:0] o_cache_source,
	output [17:0] o_cache_pixel_addr,
	output [15:0] o_cache_frame_epoch,
	output [27:0] o_cache_source_base,
	input i_cache_response_valid,
	output o_cache_response_ready,
	input [15:0] i_cache_response_rgb565,
	input i_cache_response_error,
	output o_pixel_valid,
	input i_pixel_ready,
	output [15:0] o_pixel_rgb565,
	output [18:0] o_pixel_index,
	output o_pixel_error,
	output o_idle
);
	localparam [1:0] ST_IDLE=0, ST_GATHER=1, ST_LAUNCH=2;
	reg [1:0] state_current,state_next;
	reg [95:0] lut_hold;
	reg [18:0] lut_index_hold,lane_index_hold;
	reg [63:0] neighbors_a,neighbors_b;
	reg [3:0] request_count,response_count;
	reg gather_error,lane_error_hold,lane_inflight;
	reg gather_missing;
	reg pixel_valid_o,pixel_error_o;
	reg [15:0] pixel_rgb_o;
	reg [18:0] pixel_index_o;
	reg [1:0] source_a,source_b;
	reg [17:0] neighbor_offset;
	wire [3:0] neighbor_count=lut_hold[86] ? 4'd8 : 4'd4;
	wire request_take=o_cache_request_valid && i_cache_request_ready;
	wire response_take=i_cache_response_valid && o_cache_response_ready;
	wire response_space=!pixel_valid_o || i_pixel_ready;
	wire lane_launch=state_current==ST_LAUNCH && !lane_inflight && response_space;
	wire lane_valid;
	wire [15:0] lane_rgb;
	function [1:0] map_source;
		input [2:0] slot;
		begin
			case(slot)
				0: map_source=C_SOURCE_SLOT0;
				1: map_source=C_SOURCE_SLOT1;
				2: map_source=C_SOURCE_SLOT2;
				default: map_source=C_SOURCE_SLOT3;
			endcase
		end
	endfunction
	wire lut_source_invalid=i_lut[81:79]>3 || (i_lut[86] && i_lut[84:82]>3);
	wire lut_source_missing=!i_source_present[map_source(i_lut[81:79])] ||
		(i_lut[86] && !i_source_present[map_source(i_lut[84:82])]);
	wire [95:0] lane_lut={lut_hold[95:86],lut_hold[85] && !gather_error && !gather_missing,lut_hold[84:0]};
	assign o_lut_ready=state_current==ST_IDLE;
	assign o_cache_request_valid=state_current==ST_GATHER && request_count<neighbor_count;
	assign o_cache_source=request_count<4 ? source_a : source_b;
	assign o_cache_pixel_addr=(request_count<4 ? lut_hold[17:0] : lut_hold[35:18])+neighbor_offset;
	assign o_cache_frame_epoch=i_source_epoch[o_cache_source*16+:16];
	assign o_cache_source_base=i_source_base[o_cache_source*28+:28];
	assign o_cache_response_ready=state_current==ST_GATHER && response_count<neighbor_count;
	assign o_pixel_valid=pixel_valid_o;
	assign o_pixel_rgb565=pixel_rgb_o;
	assign o_pixel_index=pixel_index_o;
	assign o_pixel_error=pixel_error_o;
	assign o_idle=state_current==ST_IDLE && !lane_inflight && !pixel_valid_o;
	always @(*) begin
		case(lut_hold[81:79])
			0: source_a=C_SOURCE_SLOT0;
			1: source_a=C_SOURCE_SLOT1;
			2: source_a=C_SOURCE_SLOT2;
			default: source_a=C_SOURCE_SLOT3;
		endcase
		case(lut_hold[84:82])
			0: source_b=C_SOURCE_SLOT0;
			1: source_b=C_SOURCE_SLOT1;
			2: source_b=C_SOURCE_SLOT2;
			default: source_b=C_SOURCE_SLOT3;
		endcase
		case(request_count[1:0])
			0: neighbor_offset=18'd0;
			1: neighbor_offset=18'd1;
			2: neighbor_offset=18'd640;
			default: neighbor_offset=18'd641;
		endcase
		state_next=state_current;
		case(state_current)
			ST_IDLE: if(i_lut_valid) state_next=(i_lut[85] && !lut_source_missing && !lut_source_invalid) ? ST_GATHER : ST_LAUNCH;
			ST_GATHER: if(response_count+{3'd0,response_take}==neighbor_count) state_next=ST_LAUNCH;
			ST_LAUNCH: if(lane_launch) state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_current<=ST_IDLE;
		else state_current<=state_next;
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			lut_hold<=0; lut_index_hold<=0; neighbors_a<=0; neighbors_b<=0;
			request_count<=0; response_count<=0; gather_error<=0;
			gather_missing<=0;
			lane_index_hold<=0; lane_error_hold<=0; lane_inflight<=0;
			pixel_valid_o<=0; pixel_rgb_o<=0; pixel_index_o<=0; pixel_error_o<=0;
		end else begin
			if(pixel_valid_o && i_pixel_ready) pixel_valid_o<=0;
			if(i_lut_valid && o_lut_ready) begin
				lut_hold<=i_lut; lut_index_hold<=i_lut_index; neighbors_a<=0; neighbors_b<=0;
				request_count<=0; response_count<=0;
				// 已冻结的present/lease决定是否可读；缺源黑像素不触发整帧取消。
				gather_missing<=i_lut[85] && lut_source_missing;
				gather_error<=i_lut[85] && lut_source_invalid;
			end
			if(request_take) begin
				request_count<=request_count+1'b1;
			end
			if(response_take) begin
				if(response_count<4) neighbors_a[response_count*16+:16]<=i_cache_response_rgb565;
				else neighbors_b[(response_count-4)*16+:16]<=i_cache_response_rgb565;
				response_count<=response_count+1'b1;
				if(i_cache_response_error) gather_error<=1;
			end
			if(lane_valid) begin
				pixel_valid_o<=1; pixel_rgb_o<=lane_rgb; pixel_index_o<=lane_index_hold;
				pixel_error_o<=lane_error_hold; lane_inflight<=0;
			end
			if(lane_launch) begin
				lane_index_hold<=lut_index_hold; lane_error_hold<=gather_error; lane_inflight<=1;
			end
		end
	end
	avm_pixel_lane u_lane(.i_clk(i_clk),.i_rstn(i_rst_n),.i_valid(lane_launch),
		.i_lut(lane_lut),.i_neighbors_a(neighbors_a),.i_neighbors_b(neighbors_b),.o_valid(lane_valid),.o_rgb565(lane_rgb));
endmodule
