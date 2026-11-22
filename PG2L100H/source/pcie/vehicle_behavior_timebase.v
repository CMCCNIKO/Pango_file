//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-21
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在 27MHz 参考时钟域分频产生 32 位毫秒计数，通过寄存 Gray 码同步到 PCIe 域供行为保持与超时判定。
// Module Name: vehicle_behavior_timebase
// File Name: source/pcie/vehicle_behavior_timebase.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在 27MHz 参考时钟域分频产生 32 位毫秒计数，通过寄存 Gray 码同步到 PCIe 域供行为保持与超时判定。
// 复位、链路和控制会话协同清空时间状态，分频参数支持 1..32768。
//
// Dependencies: 无子模块；上层提供参考/PCIe 时钟及协调会话复位。
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
// Create Date: 2026-10-21
// Design Name: Fixed reference milliseconds for vehicle behavior
// Module Name: vehicle_behavior_timebase
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 27 MHz reference milliseconds; registered Gray CDC to PCIe.
// Dependencies: None. C_REF_CYCLES_PER_MS must be in 1..32768.
// Revision: 0.01 - 2026-10-21 Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module vehicle_behavior_timebase
#(
	parameter integer C_REF_CYCLES_PER_MS = 27000
)
(
	input i_ref_clk,
	input i_ref_rst_n,
	input i_clk,
	input i_rst_n,
	input i_control_rst_n,
	input i_link_up,
	output o_dst_rst_n,
	output o_ready,
	output [31:0] o_timestamp_ms
);
	localparam [14:0] C_REF_LAST = C_REF_CYCLES_PER_MS - 1;
	wire common_rst_n;
	wire ref_rst_n;
	wire dst_rst_n;
	wire [31:0] timestamp_next;
	wire [31:0] timestamp_decode;
	reg [2:0] reg_ref_reset_sync /* synthesis syn_preserve = 1 */;
	reg [2:0] reg_dst_reset_sync /* synthesis syn_preserve = 1 */;
	reg [14:0] cnt_ref_cycle;
	reg [31:0] reg_timestamp_binary;
	reg [31:0] reg_timestamp_gray /* synthesis syn_preserve = 1 */;
	reg [31:0] reg_gray_sync0 /* synthesis syn_preserve = 1 */;
	reg [31:0] reg_gray_sync1 /* synthesis syn_preserve = 1 */;
	reg [31:0] reg_gray_sync2 /* synthesis syn_preserve = 1 */;
	reg [3:0] reg_ready_pipe;
	reg [31:0] timestamp_ms_o;
	genvar gray_bit;

	assign common_rst_n = i_ref_rst_n && i_rst_n && i_control_rst_n && i_link_up;
	assign ref_rst_n = reg_ref_reset_sync[2];
	assign dst_rst_n = reg_dst_reset_sync[2];
	assign timestamp_next = reg_timestamp_binary + 32'd1;
	assign o_dst_rst_n = dst_rst_n;
	assign o_ready = reg_ready_pipe[3];
	assign o_timestamp_ms = timestamp_ms_o;

	// Both endpoints assert together, including while the destination clock stops.
	always@(posedge i_ref_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			reg_ref_reset_sync <= 3'd0;
		end else begin
			reg_ref_reset_sync <= {reg_ref_reset_sync[1:0], 1'b1};
		end
	end
	always@(posedge i_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			reg_dst_reset_sync <= 3'd0;
		end else begin
			reg_dst_reset_sync <= {reg_dst_reset_sync[1:0], 1'b1};
		end
	end

	// Binary and Gray represent the same next millisecond on the same source edge.
	always@(posedge i_ref_clk or negedge ref_rst_n)begin
		if(!ref_rst_n)begin
			cnt_ref_cycle <= 15'd0;
			reg_timestamp_binary <= 32'd0;
			reg_timestamp_gray <= 32'd0;
		end else if(cnt_ref_cycle == C_REF_LAST)begin
			cnt_ref_cycle <= 15'd0;
			reg_timestamp_binary <= timestamp_next;
			reg_timestamp_gray <= timestamp_next ^ (timestamp_next >> 1);
		end else begin
			cnt_ref_cycle <= cnt_ref_cycle + 1'b1;
		end
	end

	// Only the registered Gray word crosses domains; decode the final stage.
	always@(posedge i_clk or negedge dst_rst_n)begin
		if(!dst_rst_n)begin
			reg_gray_sync0 <= 32'd0;
			reg_gray_sync1 <= 32'd0;
			reg_gray_sync2 <= 32'd0;
			reg_ready_pipe <= 4'd0;
			timestamp_ms_o <= 32'd0;
		end else begin
			reg_gray_sync0 <= reg_timestamp_gray;
			reg_gray_sync1 <= reg_gray_sync0;
			reg_gray_sync2 <= reg_gray_sync1;
			reg_ready_pipe <= {reg_ready_pipe[2:0], 1'b1};
			timestamp_ms_o <= timestamp_decode;
		end
	end
	generate
		for(gray_bit = 0; gray_bit < 32; gray_bit = gray_bit + 1)begin:gen_gray_decode
			assign timestamp_decode[gray_bit] = ^reg_gray_sync2[31:gray_bit];
		end
	endgenerate
endmodule
