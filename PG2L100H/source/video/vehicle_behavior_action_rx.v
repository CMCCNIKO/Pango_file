//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 解析 BV 用户动作包并向行为邮箱提交动作，生成 8 字节 BO 应答及 busy/offline 状态。
// Module Name: vehicle_behavior_action_rx
// File Name: source/video/vehicle_behavior_action_rx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 解析 BV 用户动作包并向行为邮箱提交动作，生成 8 字节 BO 应答及 busy/offline 状态。
// 使用有界 ACK 队列；BO 只确认邮箱接受，不代表动作执行完成，非空摘要同沿寄存供发送仲裁。
//
// Dependencies: vehicle_behavior_action_cdc 邮箱许可与状态；UDP 控制分流及 ACK 发送。
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
// Create Date: 2026-10-18
// Module Name: vehicle_behavior_action_rx
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: BV actions; BO acknowledges mailbox acceptance, not execution.
// Revision: 0.02 - 2026-10-20 Eight-byte BO, bounded ACK queue, busy/offline.
//           0.03 - 2026-10-29 Registered same-edge ACK nonempty summary.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module vehicle_behavior_action_rx(
	input i_clk,
	input i_rst_n,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	input i_ack_ready,
	input i_action_ready,
	input i_action_online,
	output o_action_valid,
	output [15:0] o_action_sequence,
	output [2:0] o_action_type,
	output o_action_request,
	output o_packet_error,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	output [31:0] o_ack_drop_count
);
	reg [7:0] reg_byte0;
	reg [7:0] reg_byte1;
	reg [7:0] reg_byte2;
	reg [7:0] reg_byte3;
	reg [7:0] reg_byte4;
	reg [7:0] reg_byte5;
	reg [4:0] reg_byte_count;
	reg reg_receiving;
	reg reg_error;
	reg reg_emit_pending;
	reg [63:0] reg_ack_queue [0:3];
	reg [1:0] reg_ack_read;
	reg [1:0] reg_ack_write;
	reg [2:0] cnt_ack;
	(* syn_preserve = 1 *) reg flag_ack_nonempty;
	reg [31:0] ack_drop_count_o;
	wire flag_behavior;
	wire flag_format;
	wire flag_ack_pop;
	wire flag_ack_space;
	wire flag_ack_push;
	wire flag_accept;
	wire [15:0] dec_error;
	integer reset_index;

	assign flag_behavior = reg_byte_count >= 5'd2 && reg_byte0 == 8'h42 && reg_byte1 == 8'h56;
	assign flag_format = !reg_error && reg_byte_count == 5'd6 &&
		{reg_byte3,reg_byte2} != 16'd0 && reg_byte4 >= 8'd1 &&
		reg_byte4 <= 8'd4 && reg_byte5 == 8'd1;
	assign flag_ack_pop = flag_ack_nonempty && i_ack_ready;
	assign flag_ack_space = cnt_ack < 3'd4 || flag_ack_pop;
	assign flag_ack_push = reg_emit_pending && flag_behavior && flag_ack_space;
	assign flag_accept = flag_ack_push && flag_format && i_action_online && i_action_ready;
	assign dec_error = !flag_format ? 16'd1 : !i_action_online ? 16'd3 :
		!i_action_ready ? 16'd2 : 16'd0;
	// A launch pulse and its success BO are sampled on the same source edge.
	assign o_action_valid = flag_accept;
	assign o_action_sequence = {reg_byte3,reg_byte2};
	assign o_action_type = reg_byte4[2:0];
	assign o_action_request = reg_byte5[0];
	assign o_packet_error = reg_emit_pending && flag_behavior && !flag_format;
	assign o_ack_valid = flag_ack_nonempty;
	assign o_ack_payload = {reg_ack_queue[reg_ack_read],80'd0};
	assign o_ack_length = 16'd8;
	assign o_ack_drop_count = ack_drop_count_o;

	// 预测同沿计数后的非空状态，切断计数归约到公共ACK仲裁的跨模块组合路径。
	// push优先：含pop+push时至少保留一项；仅pop时旧计数大于1才仍非空。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			flag_ack_nonempty <= 1'b0;
		end else if(flag_ack_push)begin
			flag_ack_nonempty <= 1'b1;
		end else if(flag_ack_pop)begin
			flag_ack_nonempty <= cnt_ack > 3'd1;
		end
	end

	// A full ACK queue rejects execution; the existing queue head stays intact.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_ack_read <= 2'd0;
			reg_ack_write <= 2'd0;
			cnt_ack <= 3'd0;
			ack_drop_count_o <= 32'd0;
			for(reset_index = 0; reset_index < 4; reset_index = reset_index + 1)begin
				reg_ack_queue[reset_index] <= 64'd0;
			end
		end else begin
			if(flag_ack_pop)begin
				reg_ack_read <= reg_ack_read + 1'b1;
			end
			if(flag_ack_push)begin
				reg_ack_queue[reg_ack_write] <= {8'h42,8'h4f,reg_byte3,reg_byte2,
					reg_byte4,(dec_error == 16'd0 ? 8'd0 : 8'd1),dec_error};
				reg_ack_write <= reg_ack_write + 1'b1;
			end
			case({flag_ack_push,flag_ack_pop})
				2'b10:begin cnt_ack <= cnt_ack + 1'b1; end
				2'b01:begin cnt_ack <= cnt_ack - 1'b1; end
				default:begin end
			endcase
			if(reg_emit_pending && flag_behavior && !flag_ack_space)begin
				ack_drop_count_o <= ack_drop_count_o + 1'b1;
			end
		end
	end

	// The end stage sees the final byte written on the prior edge.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_byte0 <= 8'd0;
			reg_byte1 <= 8'd0;
			reg_byte2 <= 8'd0;
			reg_byte3 <= 8'd0;
			reg_byte4 <= 8'd0;
			reg_byte5 <= 8'd0;
			reg_byte_count <= 5'd0;
			reg_receiving <= 1'b0;
			reg_error <= 1'b0;
			reg_emit_pending <= 1'b0;
		end else begin
			reg_emit_pending <= 1'b0;
			if(i_start)begin
				reg_receiving <= 1'b1;
				reg_byte_count <= 5'd0;
				reg_error <= i_error;
				if(i_valid)begin
					reg_byte0 <= i_data;
					reg_byte_count <= 5'd1;
				end
			end else if(reg_receiving && i_valid)begin
				case(reg_byte_count)
					5'd0:begin reg_byte0 <= i_data; end
					5'd1:begin reg_byte1 <= i_data; end
					5'd2:begin reg_byte2 <= i_data; end
					5'd3:begin reg_byte3 <= i_data; end
					5'd4:begin reg_byte4 <= i_data; end
					5'd5:begin reg_byte5 <= i_data; end
					default:begin reg_error <= 1'b1; end
				endcase
				if(reg_byte_count != 5'h1f)begin
					reg_byte_count <= reg_byte_count + 1'b1;
				end
				if(i_error)begin reg_error <= 1'b1; end
			end
			if(i_end && (reg_receiving || i_start))begin
				reg_receiving <= 1'b0;
				reg_emit_pending <= 1'b1;
				if(i_error)begin reg_error <= 1'b1; end
			end
		end
	end
endmodule
