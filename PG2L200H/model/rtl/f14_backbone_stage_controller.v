//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 串行调度 Stem、13 个 bottleneck 与 adapter 的整层处理顺序，依次取控制记录、装资产、下发配置/几何并协调 tile 与写回。
// Module Name: f14_backbone_stage_controller
// File Name: model/rtl/f14_backbone_stage_controller.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 串行调度 Stem、13 个 bottleneck 与 adapter 的整层处理顺序，依次取控制记录、装资产、下发配置/几何并协调 tile 与写回。
// 每次只保存当前层几何和四段资产信息，支持 stage_valid 观察暂停与整层 watchdog，并累计周期和错误码。
//
// Dependencies: record_fetch、asset_loader、geometry、gather、tile、streamer 的握手接口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 本模块不负责 alignment、head 或最终 result；子模块 done 按单拍契约，复位须同步清空子模块在途事务。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 single-engine backbone stage controller
// Module Name: f14_backbone_stage_controller
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: On-demand controls; assets -> gather -> shared tile -> writeback.
// Dependencies: Existing record fetch/loader/geometry/gather/tile/streamer
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 仅控制stem+13个bottleneck+adapter，不包含alignment、head或最终result。
// 每次只保存当前层的几何和4个资产段；cfg的10个字直接送原tile寄存器。
// stage_valid允许验证端暂停回读；生产侧stage_ready可恒为1。
// 子模块done必须单拍，复位同时清空所有子模块在途事务。
module f14_backbone_stage_controller
#(
	parameter [31:0] C_TIMEOUT_CYCLES = 32'd20000000
)
(
	input wire i_clk,
	input wire i_rst_n,
	input wire i_frame_valid,
	input wire [31:0] i_frame_id,
	output wire o_frame_accept,
	output wire o_busy,
	output wire o_backbone_done,
	output wire [31:0] o_frame_id,
	output wire [31:0] o_cycles,
	output wire [9:0] o_error_code,
	output wire o_stage_valid,
	input wire i_stage_ready,
	output wire [3:0] o_stage_index,
	output wire o_record_valid,
	input wire i_record_ready,
	output wire [4:0] o_record_index,
	input wire i_record_done,
	input wire i_record_error,
	input wire i_control_valid,
	output wire o_control_ready,
	input wire [4:0] i_control_index,
	input wire [31:0] i_control_data,
	output wire o_asset_valid,
	input wire i_asset_ready,
	output wire [1:0] o_asset_kind,
	output wire [16:0] o_asset_base,
	output wire [10:0] o_asset_count,
	input wire i_asset_done,
	input wire i_asset_error,
	output wire o_cfg_wr_en,
	output wire [3:0] o_cfg_wr_addr,
	output wire [31:0] o_cfg_wr_data,
	input wire i_cfg_ready,
	output wire o_cfg_commit,
	output wire [31:0] o_cfg_geometry,
	output wire o_geometry_valid,
	input wire i_geometry_ready,
	input wire i_geometry_tile_valid,
	output wire o_geometry_tile_ready,
	input wire i_geometry_done,
	input wire i_geometry_error,
	output wire o_gather_valid,
	input wire i_gather_ready,
	input wire i_gather_done,
	input wire i_gather_error,
	output wire o_tile_start,
	input wire i_tile_start_ready,
	input wire i_tile_done,
	input wire i_tile_error,
	output wire o_stream_valid,
	input wire i_stream_ready,
	input wire i_stream_done,
	input wire i_stream_error,
	output wire [1:0] o_kind,
	output wire [7:0] o_input_h,
	output wire [7:0] o_input_w,
	output wire [6:0] o_output_h,
	output wire [6:0] o_output_w,
	output wire [9:0] o_output_c,
	output wire [3:0] o_input_groups,
	output wire [7:0] o_input_zero,
	output wire o_source_bank_b,
	output wire [31:0] o_destination,
	output wire [16:0] o_image_base
);
	localparam [4:0] ST_IDLE = 5'd0;
	localparam [4:0] ST_RECORD_REQ = 5'd1;
	localparam [4:0] ST_RECORD_WAIT = 5'd2;
	localparam [4:0] ST_ASSET_CHECK = 5'd3;
	localparam [4:0] ST_ASSET_REQ = 5'd4;
	localparam [4:0] ST_ASSET_WAIT = 5'd5;
	localparam [4:0] ST_ASSET_NEXT = 5'd6;
	localparam [4:0] ST_COMMIT = 5'd7;
	localparam [4:0] ST_GEOMETRY_REQ = 5'd8;
	localparam [4:0] ST_TILE_WAIT = 5'd9;
	localparam [4:0] ST_GATHER_REQ = 5'd10;
	localparam [4:0] ST_GATHER_WAIT = 5'd11;
	localparam [4:0] ST_COMPUTE_REQ = 5'd12;
	localparam [4:0] ST_COMPUTE_WAIT = 5'd13;
	localparam [4:0] ST_STREAM_REQ = 5'd14;
	localparam [4:0] ST_STREAM_WAIT = 5'd15;
	localparam [4:0] ST_TILE_ACK = 5'd16;
	localparam [4:0] ST_REPORT = 5'd17;
	localparam [4:0] ST_DONE = 5'd18;
	localparam [4:0] ST_ERROR = 5'd19;
	reg [4:0] state_current;
	reg [4:0] state_next;
	reg active_q;
	reg [3:0] stage_q;
	reg [31:0] frame_q;
	reg [31:0] cycles_q;
	reg [9:0] error_q;
	reg [5:0] record_words_q;
	reg record_bad_q;
	reg [1:0] kind_q;
	reg [7:0] input_h_q;
	reg [7:0] input_w_q;
	reg [6:0] output_h_q;
	reg [6:0] output_w_q;
	reg [9:0] output_c_q;
	reg [31:0] geometry_q;
	reg [7:0] input_zero_q;
	reg source_bank_q;
	reg [31:0] destination_q;
	reg [16:0] image_base_q;
	reg [111:0] sections_q;
	reg [1:0] section_q;
	wire cfg_word;
	wire control_fire;
	wire control_bad;
	wire [27:0] current_section;
	wire sub_error;
	wire timeout_error;

	assign o_busy = active_q && (state_current != ST_IDLE);
	assign o_frame_accept = active_q && (state_current == ST_IDLE) && i_frame_valid;
	assign o_backbone_done = active_q && ((state_current == ST_DONE) || (state_current == ST_ERROR));
	assign o_frame_id = frame_q;
	assign o_cycles = cycles_q;
	assign o_error_code = error_q;
	assign o_stage_valid = active_q && (state_current == ST_REPORT);
	assign o_stage_index = stage_q;
	assign o_record_valid = active_q && (state_current == ST_RECORD_REQ);
	assign o_record_index = {1'b0,stage_q};
	assign cfg_word = (i_control_index >= 5'd12) && (i_control_index <= 5'd21);
	assign o_control_ready = active_q && (state_current == ST_RECORD_WAIT) && (!cfg_word || i_cfg_ready);
	assign control_fire = i_control_valid && o_control_ready;
	assign o_cfg_wr_en = control_fire && cfg_word;
	assign o_cfg_wr_addr = i_control_index[3:0] - 4'd12;
	assign o_cfg_wr_data = i_control_data;
	assign o_cfg_commit = active_q && (state_current == ST_COMMIT);
	assign o_cfg_geometry = geometry_q;
	assign current_section = sections_q[section_q * 28 +: 28];
	assign o_asset_valid = active_q && (state_current == ST_ASSET_REQ);
	assign o_asset_kind = section_q;
	assign o_asset_base = current_section[16:0];
	assign o_asset_count = current_section[27:17];
	assign o_geometry_valid = active_q && (state_current == ST_GEOMETRY_REQ);
	assign o_geometry_tile_ready = active_q && (state_current == ST_TILE_ACK);
	assign o_gather_valid = active_q && (state_current == ST_GATHER_REQ);
	assign o_tile_start = active_q && (state_current == ST_COMPUTE_REQ) && i_tile_start_ready;
	assign o_stream_valid = active_q && (state_current == ST_STREAM_REQ);
	assign o_kind = kind_q;
	assign o_input_h = input_h_q;
	assign o_input_w = input_w_q;
	assign o_output_h = output_h_q;
	assign o_output_w = output_w_q;
	assign o_output_c = output_c_q;
	assign o_input_groups = geometry_q[3:0];
	assign o_input_zero = input_zero_q;
	assign o_source_bank_b = source_bank_q;
	assign o_destination = destination_q;
	assign o_image_base = image_base_q;
	assign control_bad = (i_control_index != record_words_q[4:0]) ||
		((i_control_index == 5'd0) && (i_control_data != (32'hf14c0200 | {28'd0,stage_q}))) ||
		((i_control_index == 5'd1) && (i_control_data != ((stage_q == 4'd0) ? 32'd0 : ((stage_q == 4'd14) ? 32'd2 : 32'd1)))) ||
		((i_control_index == 5'd30) && (i_control_data != 32'd1));
	assign sub_error = i_record_error || i_asset_error || i_geometry_error || i_gather_error || i_tile_error || i_stream_error;
	assign timeout_error = (cycles_q >= C_TIMEOUT_CYCLES);

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin if(o_frame_accept)begin state_next = ST_RECORD_REQ; end end
			ST_RECORD_REQ:begin if(i_record_ready)begin state_next = ST_RECORD_WAIT; end end
			ST_RECORD_WAIT:begin
				if(i_record_done)begin
					state_next = (record_bad_q || (record_words_q != 6'd32)) ? ST_ERROR : ST_ASSET_CHECK;
				end
			end
			ST_ASSET_CHECK:begin state_next = (o_asset_count == 11'd0) ? ST_ASSET_NEXT : ST_ASSET_REQ; end
			ST_ASSET_REQ:begin if(i_asset_ready)begin state_next = ST_ASSET_WAIT; end end
			ST_ASSET_WAIT:begin if(i_asset_done)begin state_next = ST_ASSET_NEXT; end end
			ST_ASSET_NEXT:begin state_next = (section_q == 2'd3) ? ST_COMMIT : ST_ASSET_CHECK; end
			ST_COMMIT:begin state_next = ST_GEOMETRY_REQ; end
			ST_GEOMETRY_REQ:begin if(i_geometry_ready)begin state_next = ST_TILE_WAIT; end end
			ST_TILE_WAIT:begin
				if(i_geometry_done)begin state_next = ST_REPORT; end
				else if(i_geometry_tile_valid && i_gather_ready && i_tile_start_ready)begin state_next = ST_GATHER_REQ; end
			end
			ST_GATHER_REQ:begin if(i_gather_ready)begin state_next = ST_GATHER_WAIT; end end
			ST_GATHER_WAIT:begin if(i_gather_done)begin state_next = ST_COMPUTE_REQ; end end
			ST_COMPUTE_REQ:begin if(i_tile_start_ready)begin state_next = ST_COMPUTE_WAIT; end end
			ST_COMPUTE_WAIT:begin if(i_tile_done)begin state_next = ST_STREAM_REQ; end end
			ST_STREAM_REQ:begin if(i_stream_ready)begin state_next = ST_STREAM_WAIT; end end
			ST_STREAM_WAIT:begin if(i_stream_done)begin state_next = ST_TILE_ACK; end end
			ST_TILE_ACK:begin state_next = ST_TILE_WAIT; end
			ST_REPORT:begin
				if(i_stage_ready)begin state_next = (stage_q == 4'd14) ? ST_DONE : ST_RECORD_REQ; end
			end
			ST_DONE:begin state_next = ST_IDLE; end
			ST_ERROR:begin state_next = ST_IDLE; end
			default:begin state_next = ST_ERROR; end
		endcase
		if(o_busy && (state_current != ST_DONE) && (state_current != ST_ERROR) && (sub_error || timeout_error))begin
			state_next = ST_ERROR;
		end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin state_current <= ST_IDLE; active_q <= 1'b0; end
		else begin state_current <= state_next; active_q <= 1'b1; end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			stage_q <= 4'd0; frame_q <= 32'd0; cycles_q <= 32'd0; error_q <= 10'd0;
			record_words_q <= 6'd0; record_bad_q <= 1'b0; section_q <= 2'd0;
		end else begin
			if(o_frame_accept)begin
				stage_q <= 4'd0; frame_q <= i_frame_id; cycles_q <= 32'd0; error_q <= 10'd0;
				record_words_q <= 6'd0; record_bad_q <= 1'b0; section_q <= 2'd0;
			end else if(o_busy && !o_backbone_done)begin cycles_q <= cycles_q + 32'd1; end
			if(control_fire)begin
				record_words_q <= record_words_q + 6'd1;
				if(control_bad)begin record_bad_q <= 1'b1; end
			end
			if(state_current == ST_ASSET_NEXT)begin section_q <= section_q + 2'd1; end
			if(o_stage_valid && i_stage_ready && (stage_q != 4'd14))begin
				stage_q <= stage_q + 4'd1; section_q <= 2'd0;
				record_words_q <= 6'd0; record_bad_q <= 1'b0;
			end
			if((state_next == ST_ERROR) && (state_current != ST_ERROR))begin
				error_q <= {timeout_error,1'b0,record_bad_q || (record_words_q != 6'd32),
					i_record_error,i_asset_error,i_geometry_error,i_gather_error,i_tile_error,i_stream_error,1'b0};
			end
		end
	end
	// 只在完整的新记录到来时更新数据字段；软复位不异步扰动存储器地址。
	always@(posedge i_clk)begin
		if(control_fire)begin
			case(i_control_index)
				5'd1:begin kind_q <= i_control_data[1:0]; end
				5'd3:begin input_h_q <= i_control_data[7:0]; input_w_q <= i_control_data[23:16]; end
				5'd4:begin output_h_q <= i_control_data[6:0]; output_w_q <= i_control_data[22:16]; end
				5'd5:begin output_c_q <= i_control_data[25:16]; end
				5'd6:begin source_bank_q <= i_control_data[18]; end
				5'd7:begin destination_q <= i_control_data; end
				5'd8:begin sections_q[27:0] <= i_control_data[27:0]; end
				5'd9:begin sections_q[55:28] <= i_control_data[27:0]; end
				5'd10:begin sections_q[83:56] <= i_control_data[27:0]; end
				5'd11:begin sections_q[111:84] <= i_control_data[27:0]; end
				5'd21:begin geometry_q <= i_control_data; end
				5'd23:begin image_base_q <= i_control_data[16:0]; end
				5'd28:begin input_zero_q <= i_control_data[7:0]; end
				default:begin end
			endcase
		end
	end
endmodule
`default_nettype wire
