//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 对控制子系统 ACK 执行整包轮询仲裁，支持默认四路及参数可选的第五路 ISP ACK。
// Module Name: udp_control_ack_arbiter
// File Name: source/video/control_v2/udp_control_ack_arbiter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对控制子系统 ACK 执行整包轮询仲裁，支持默认四路及参数可选的第五路 ISP ACK。
// 输出寄存 payload 和 length，在下游背压期间保持稳定，当前包完成前保持发送方所有权。
//
// Dependencies: 各控制 ACK 队列及下游 UDP 发送握手。
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
// Module Name: udp_control_ack_arbiter
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 整包轮询仲裁；输出FF在背压期间保持payload及length。
// Revision: 0.01
//           0.02 - C_ISP_ENABLE可选第五路ISP ACK，保持默认四路语义。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module udp_control_ack_arbiter
#(
	parameter integer C_LUT_ENABLE = 0,
	parameter integer C_ISP_ENABLE = 0
)
(
	input i_clk,
	input i_rstn,
	input i_algo_valid,
	input [143:0] i_algo_payload,
	input [15:0] i_algo_length,
	output o_algo_ready,
	input i_route_valid,
	input [143:0] i_route_payload,
	input [15:0] i_route_length,
	output o_route_ready,
	output o_valid,
	output [143:0] o_payload,
	output [15:0] o_length,
	input i_ready,
	input i_lut_valid,
	input [143:0] i_lut_payload,
	input [15:0] i_lut_length,
	output o_lut_ready,
	input i_behavior_valid,
	input [143:0] i_behavior_payload,
	input [15:0] i_behavior_length,
	output o_behavior_ready,
	input i_isp_valid,
	input [143:0] i_isp_payload,
	input [15:0] i_isp_length,
	output o_isp_ready
);
	reg valid_o;
	reg [143:0] payload_o;
	reg [15:0] length_o;
	reg [2:0] reg_next;
	reg [2:0] dec_select;
	wire flag_space;
	wire flag_lut;
	wire flag_behavior;
	wire flag_isp;
	wire flag_any;
	assign flag_space = !valid_o || i_ready;
	assign flag_lut = C_LUT_ENABLE != 0 && i_lut_valid;
	assign flag_behavior = i_behavior_valid;
	assign flag_isp = C_ISP_ENABLE != 0 && i_isp_valid;
	assign flag_any = i_algo_valid || i_route_valid || flag_lut || flag_behavior || flag_isp;
	assign o_algo_ready = flag_space && dec_select == 0;
	assign o_route_ready = flag_space && dec_select == 1;
	assign o_lut_ready = flag_space && dec_select == 2 && C_LUT_ENABLE != 0;
	assign o_behavior_ready = flag_space && dec_select == 3;
	assign o_isp_ready = flag_space && dec_select == 4 && C_ISP_ENABLE != 0;
	assign o_valid = valid_o;
	assign o_payload = payload_o;
	assign o_length = length_o;
	always@(*)begin
		dec_select = 0;
		case(reg_next)
			3'd0:begin
				if(i_algo_valid)begin dec_select=0;end
				else if(i_route_valid)begin dec_select=1;end
				else if(flag_lut)begin dec_select=2;end
				else if(flag_behavior)begin dec_select=3;end
				else if(flag_isp)begin dec_select=4;end
			end
			3'd1:begin
				if(i_route_valid)begin dec_select=1;end
				else if(flag_lut)begin dec_select=2;end
				else if(flag_behavior)begin dec_select=3;end
				else if(flag_isp)begin dec_select=4;end
				else begin dec_select=0;end
			end
			3'd2:begin
				if(flag_lut)begin dec_select=2;end
				else if(flag_behavior)begin dec_select=3;end
				else if(flag_isp)begin dec_select=4;end
				else if(i_algo_valid)begin dec_select=0;end
				else if(i_route_valid)begin dec_select=1;end
			end
			3'd3:begin
				if(flag_behavior)begin dec_select=3;end
				else if(flag_isp)begin dec_select=4;end
				else if(i_algo_valid)begin dec_select=0;end
				else if(i_route_valid)begin dec_select=1;end
				else if(flag_lut)begin dec_select=2;end
			end
			default:begin
				if(flag_isp)begin dec_select=4;end
				else if(i_algo_valid)begin dec_select=0;end
				else if(i_route_valid)begin dec_select=1;end
				else if(flag_lut)begin dec_select=2;end
				else if(flag_behavior)begin dec_select=3;end
			end
		endcase
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin valid_o <= 0; payload_o <= 0; length_o <= 0; reg_next <= 0; end
		else if(flag_space)begin
			valid_o <= flag_any;
			if(flag_any)begin
				case(dec_select)
					3'd1:begin payload_o<=i_route_payload;length_o<=i_route_length;end
					3'd2:begin payload_o<=i_lut_payload;length_o<=i_lut_length;end
					3'd3:begin payload_o<=i_behavior_payload;length_o<=i_behavior_length;end
					3'd4:begin payload_o<=i_isp_payload;length_o<=i_isp_length;end
					default:begin payload_o<=i_algo_payload;length_o<=i_algo_length;end
				endcase
				reg_next <= dec_select == 4 || (dec_select == 3 && C_ISP_ENABLE == 0) ? 0 : dec_select + 1'b1;
			end
		end
	end
endmodule
