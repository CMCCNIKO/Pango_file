//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 斑马线图像支路总成：从叠线前 RGB565 生成掩码，经双 bank 跨域采集、串行几何扫描和时间门控输出场景有效/激活及诊断计数。
// Module Name: host_f14_crosswalk_path
// File Name: host/rtl/video/host_f14_crosswalk_path.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 斑马线图像支路总成：从叠线前 RGB565 生成掩码，经双 bank 跨域采集、串行几何扫描和时间门控输出场景有效/激活及诊断计数。
// 与 CNN 并行运行；原始帧号用于关联，门控通过累计归一帧号将正常步长二转换为 30 fps 单位，并处理 16 位源帧号回绕。
//
// Dependencies: host_f14_crosswalk_mask、host_f14_crosswalk_banks、host_f14_crosswalk_scan、host_f14_crosswalk_gate；像素域与系统域。
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
// 叠线前RGB565独立支路；与CNN并行。正常1080p60每2帧选1帧，frame_id步长2。
// 原始帧号用于关联；门控计时器使用累计归一帧号，跨16bit源帧号回绕仍连续。
`timescale 1ns/1ps
`default_nettype none
module host_f14_crosswalk_path (
	input wire i_pixel_clk,
	input wire i_pixel_rst_n,
	input wire i_valid,
	input wire [15:0] i_rgb565,
	input wire [8:0] i_row,
	input wire [9:0] i_col,
	input wire [15:0] i_frame_id,
	input wire i_abort,
	input wire i_sys_clk,
	input wire i_sys_rst_n,
	output wire o_valid,
	output wire [15:0] o_frame_id,
	output wire o_active,
	output wire [31:0] o_scan_cycles,
	output wire [15:0] o_capture_drops
);
	reg accept_q;
	wire first=i_valid && i_row==0 && i_col==0;
	wire mv,mb;
	wire [8:0] my;
	wire [9:0] mx;
	wire [15:0] mf;
	wire descriptor_valid,scan_busy,scan_done,scan_req,read_valid,candidate;
	wire [15:0] descriptor_frame;
	wire [11:0] read_address;
	wire [31:0] read_data;
	wire [9:0] left,right;
	wire [8:0] top,bottom;
	reg [15:0] source_frame_q,previous_source_q,logical_frame_q;
	reg source_seen_q;
	wire [15:0] source_delta=descriptor_frame-previous_source_q;
	wire start=descriptor_valid && !scan_busy;
	assign o_frame_id=source_frame_q;
	always @(posedge i_pixel_clk or negedge i_pixel_rst_n)begin
		if(!i_pixel_rst_n)accept_q<=0;
		else if(i_abort)accept_q<=0;
		else if(first)accept_q<=1;
	end
	always @(posedge i_sys_clk or negedge i_sys_rst_n)begin
		if(!i_sys_rst_n)begin source_frame_q<=0;previous_source_q<=0;logical_frame_q<=0;source_seen_q<=0;end
		else if(start)begin
			source_frame_q<=descriptor_frame;previous_source_q<=descriptor_frame;source_seen_q<=1;
			logical_frame_q<=source_seen_q ? logical_frame_q+{1'b0,source_delta[15:1]} : {1'b0,descriptor_frame[15:1]};
		end
	end
	host_f14_crosswalk_mask u_mask(.i_clk(i_pixel_clk),.i_rst_n(i_pixel_rst_n),.i_abort(i_abort),
		.i_valid(i_valid && (accept_q || first)),.i_rgb565(i_rgb565),.i_row(i_row),.i_col(i_col),.i_frame_id(i_frame_id),
		.o_valid(mv),.o_mask(mb),.o_row(my),.o_col(mx),.o_frame_id(mf));
	host_f14_crosswalk_banks u_banks(.i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(i_pixel_rst_n),.i_abort(i_abort),
		.i_valid(mv),.i_mask(mb),.i_row(my),.i_col(mx),.i_frame_id(mf),.i_sys_clk(i_sys_clk),.i_sys_rst_n(i_sys_rst_n),
		.o_frame_valid(descriptor_valid),.o_frame_id(descriptor_frame),.i_frame_ready(!scan_busy),
		.i_read_req(scan_req),.i_read_addr(read_address),.o_read_valid(read_valid),.o_read_data(read_data),.i_release(scan_done),.o_drop_count(o_capture_drops));
	host_f14_crosswalk_scan u_scan(.i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_start(start),.o_busy(scan_busy),.o_done(scan_done),
		.o_req(scan_req),.o_addr(read_address),.i_read_valid(read_valid),.i_read_data(read_data),
		.o_candidate(candidate),.o_left(left),.o_top(top),.o_right(right),.o_bottom(bottom),.o_cycles(o_scan_cycles));
	host_f14_crosswalk_gate u_gate(.i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_valid(scan_done),.i_frame_id(logical_frame_q),
		.i_candidate(candidate),.i_left(left),.i_right(right),.i_bottom(bottom),.o_valid(o_valid),.o_frame_id(),.o_active(o_active));
endmodule
`default_nettype wire
