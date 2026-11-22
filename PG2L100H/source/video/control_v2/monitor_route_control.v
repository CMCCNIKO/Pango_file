//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 解析 8 字节监视路由命令并生成 12 字节 ACK，通过单项稳定载荷握手提交上下监视源及最终输出模式。
// Module Name: monitor_route_control
// File Name: source/video/control_v2/monitor_route_control.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 解析 8 字节监视路由命令并生成 12 字节 ACK，通过单项稳定载荷握手提交上下监视源及最终输出模式。
// opcode 1 更新监视源，opcode 3 选择 AVM 或算法输出；请求 ready 仅表示接受，active_valid 以真实 HDMI 锁存确认为准。
//
// Dependencies: DDR 路由接收端及 HDMI 真实锁存回报；8 项寄存 ACK 队列。
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
// Module Name: monitor_route_control
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 8字节路由命令、12字节ACK、完整bundled-data握手；opcode 3 选择最终HDMI输出。
// DDR请求ready只确认接收；active_valid必须来自真实HDMI锁存确认。
// route[35:0]={sequence[15:0],top[1:0],bottom[1:0],display_mode,epoch[14:0]}。
// opcode 1 保持右上/右下监视源；opcode 3 的高字节为最终输出选择：0=AVM，1=算法。
// 双域复位必须同时异步断言并分别同步释放。单请求在途。
// Dependencies: 无；ACK缓存为8个复位FF槽，不依赖存储IP。
// Revision: 0.02 - 增加最终HDMI源运行时选择，不增加CRC或哈希。
//           0.03 - 2026-10-24，ACK非空标志与计数同步更新，缩短公共仲裁链。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module monitor_route_control
(
	input i_control_clk,
	input i_control_rstn,
	input i_ddr_clk,
	input i_ddr_rstn,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	output o_request_valid,
	output [35:0] o_request_route,
	input i_request_ready,
	input i_active_valid,
	input [35:0] i_active_route,
	output o_busy,
	output [35:0] o_active_route,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	input i_ack_ready,
	output [31:0] o_dropped_ack_count,
	output o_active_ready
);
	reg [63:0] reg_packet;
	reg [15:0] cnt_bytes;
	reg flag_receiving;
	reg flag_rx_error;
	reg flag_evaluate;
	reg [15:0] reg_length;
	reg flag_end_error;
	reg flag_busy;
	reg [35:0] reg_request;
	reg [7:0] reg_request_opcode;
	reg flag_request_toggle;
	reg [35:0] reg_active;
	reg flag_active_confirmed;
	reg [1:0] reg_request_sync;
	reg flag_request_seen;
	reg request_valid_o;
	reg [35:0] request_route_o;
	reg [35:0] reg_return;
	reg flag_return_toggle;
	reg [1:0] reg_return_sync;
	reg flag_return_seen;
	reg [1:0] reg_return_ack_sync;
	reg flag_wait_active;
	reg [95:0] reg_queue [0:7];
	reg [2:0] reg_write;
	reg [2:0] reg_read;
	reg [3:0] cnt_queue;
	reg flag_queue_valid;
	reg [31:0] dropped_ack_count_o;
	reg [7:0] dec_error;
	reg [95:0] dec_ack;
	wire flag_applied;
	wire flag_pop;
	wire flag_push;
	wire flag_same;
	wire flag_duplicate;
	wire flag_new_request;
	wire [15:0] dec_sequence;
	wire [7:0] dec_opcode;
	wire [1:0] dec_top;
	wire [1:0] dec_bottom;
	wire dec_display_mode;
	assign dec_opcode = reg_packet[39:32];
	assign dec_sequence = reg_packet[31:16];
	assign dec_top = reg_packet[9:8];
	assign dec_bottom = reg_packet[1:0];
	assign dec_display_mode = reg_packet[8];
	assign flag_applied = reg_return_sync[1] != flag_return_seen;
	// 复位默认值只是路由配置，不能冒充真实输出已锁存；首次同源SET也必须往返CDC。
	assign flag_same = flag_active_confirmed &&
		((dec_opcode == 1 && dec_top == reg_active[19:18] && dec_bottom == reg_active[17:16]) ||
		 (dec_opcode == 3 && dec_display_mode == reg_active[15]));
	assign flag_duplicate = flag_busy && dec_sequence == reg_request[35:20] &&
		((reg_request_opcode == 1 && dec_opcode == 1 && dec_top == reg_request[19:18] && dec_bottom == reg_request[17:16]) ||
		 (reg_request_opcode == 3 && dec_opcode == 3 && dec_display_mode == reg_request[15]));
	assign flag_pop = flag_queue_valid && i_ack_ready;
	assign flag_push = (flag_applied || flag_evaluate) && (cnt_queue < 8 || flag_pop);
	assign flag_new_request = flag_evaluate && !flag_applied && dec_error == 0 &&
		(dec_opcode == 1 || dec_opcode == 3) && !flag_same && !flag_duplicate;
	assign o_request_valid = request_valid_o;
	assign o_request_route = request_route_o;
	assign o_busy = flag_busy;
	assign o_active_route = reg_active;
	assign o_ack_valid = flag_queue_valid;
	assign o_ack_payload = {reg_queue[reg_read],48'd0};
	assign o_ack_length = 16'd12;
	assign o_dropped_ack_count = dropped_ack_count_o;
	assign o_active_ready = reg_return_ack_sync[1] == flag_return_toggle;
	// packet shift包括最后valid；评价延后一拍，避免旧最后字节。
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_packet <= 0; cnt_bytes <= 0; flag_receiving <= 0;
			flag_rx_error <= 0; flag_evaluate <= 0; reg_length <= 0; flag_end_error <= 0;
		end else begin
			if(flag_evaluate && !flag_applied)begin flag_evaluate <= 0; end
			if(i_start)begin
				reg_packet <= i_valid ? {56'd0,i_data} : 64'd0;
				cnt_bytes <= i_valid ? 1 : 0; flag_receiving <= 1; flag_rx_error <= i_error;
			end else if(flag_receiving)begin
				if(i_valid)begin reg_packet <= {reg_packet[55:0],i_data}; cnt_bytes <= cnt_bytes == 16'hffff ? cnt_bytes : cnt_bytes + 1'b1; end
				if(i_error)begin flag_rx_error <= 1; end
			end
			if(i_end)begin
				flag_receiving <= 0; flag_evaluate <= 1;
				reg_length <= i_length; flag_end_error <= i_error || flag_rx_error || !flag_receiving;
			end
		end
	end
	always@(*)begin
		dec_error = 0;
		if(flag_end_error)begin dec_error = 6; end
		else if(cnt_bytes != 8 || reg_length != 8)begin dec_error = 1; end
		else if(reg_packet[47:40] != 1)begin dec_error = 2; end
		else if(dec_opcode != 1 && dec_opcode != 2 && dec_opcode != 3)begin dec_error = 3; end
		else if(dec_opcode == 1 && (reg_packet[15:8] > 3 || reg_packet[7:0] > 3))begin dec_error = 4; end
		else if(dec_opcode == 3 && (reg_packet[15:8] > 1 || reg_packet[7:0] != 0))begin dec_error = 4; end
		else if((dec_opcode == 1 || dec_opcode == 3) && flag_busy && !flag_duplicate)begin dec_error = 5; end
		else if((dec_opcode == 1 || dec_opcode == 3) && !flag_busy && !flag_same && cnt_queue > 5)begin dec_error = 5; end
		dec_ack = {16'h4156,8'd1,(dec_opcode | 8'h80),dec_sequence,8'd3,dec_error,6'd0,reg_active[19:18],6'd0,reg_active[17:16],reg_active[15:0]};
		if(dec_error == 0)begin
			if(dec_opcode == 2)begin dec_ack[47:40] = 4; end
			else if(flag_duplicate)begin dec_ack[47:40] = 1; end
			else if(flag_same)begin dec_ack[47:40] = 2; end
			else begin dec_ack[47:40] = 1; end
		end
		if(flag_applied)begin
			dec_ack = {16'h4156,8'd1,(reg_request_opcode | 8'h80),reg_return[35:20],8'd2,8'd0,6'd0,
				reg_return[19:18],6'd0,reg_return[17:16],reg_return[15:0]};
		end
	end
	genvar n;
	generate for(n = 0; n < 8; n = n + 1)begin: gen_ack_ff
		always@(posedge i_control_clk or negedge i_control_rstn)begin
			if(!i_control_rstn)begin reg_queue[n] <= 0; end
			else if(flag_push && reg_write == n)begin reg_queue[n] <= dec_ack; end
		end
	end endgenerate
	// 镜像更新后的非空状态，不增加ACK延迟；同时push/pop时容量不变。
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			flag_queue_valid <= 1'b0;
		end else begin
			case({flag_push,flag_pop})
				2'b10:begin flag_queue_valid <= 1'b1; end
				2'b01:begin flag_queue_valid <= cnt_queue > 4'd1; end
				default:begin end
			endcase
		end
	end
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			flag_busy <= 0; reg_request <= {16'd0,2'd3,2'd2,16'd0}; reg_request_opcode <= 1; flag_request_toggle <= 0;
			reg_active <= {16'd0,2'd3,2'd2,16'd0}; reg_return_sync <= 0; flag_return_seen <= 0;
			flag_active_confirmed <= 1'b0;
			reg_write <= 0; reg_read <= 0; cnt_queue <= 0; dropped_ack_count_o <= 0;
		end else begin
			reg_return_sync <= {reg_return_sync[0],flag_return_toggle};
			if(flag_applied && flag_push)begin
				flag_return_seen <= reg_return_sync[1]; reg_active <= reg_return; flag_busy <= 0;
				flag_active_confirmed <= 1'b1;
			end
			if(flag_new_request && flag_push)begin
				reg_request_opcode <= dec_opcode;
				if(dec_opcode == 3)begin
					reg_request <= {dec_sequence,reg_active[19:16],dec_display_mode,reg_active[14:0] + 15'd1};
				end else begin
					reg_request <= {dec_sequence,dec_top,dec_bottom,reg_active[15],reg_active[14:0] + 15'd1};
				end
				flag_request_toggle <= !flag_request_toggle; flag_busy <= 1;
			end
			if(flag_push)begin reg_write <= reg_write + 1'b1; end
			if(flag_pop)begin reg_read <= reg_read + 1'b1; end
			case({flag_push,flag_pop})
				2'b10:begin cnt_queue <= cnt_queue + 1'b1; end
				2'b01:begin cnt_queue <= cnt_queue - 1'b1; end
				default:begin cnt_queue <= cnt_queue; end
			endcase
			if(flag_evaluate && !flag_applied && !flag_push)begin dropped_ack_count_o <= dropped_ack_count_o + 1'b1; end
		end
	end
	always@(posedge i_ddr_clk or negedge i_ddr_rstn)begin
		if(!i_ddr_rstn)begin
			reg_request_sync <= 0; flag_request_seen <= 0; request_valid_o <= 0;
			request_route_o <= {16'd0,2'd3,2'd2,16'd0}; reg_return <= {16'd0,2'd3,2'd2,16'd0}; flag_return_toggle <= 0; reg_return_ack_sync <= 0; flag_wait_active <= 0;
		end else begin
			reg_request_sync <= {reg_request_sync[0],flag_request_toggle};
			reg_return_ack_sync <= {reg_return_ack_sync[0],flag_return_seen};
			if(!request_valid_o && reg_request_sync[1] != flag_request_seen)begin
				request_route_o <= reg_request; request_valid_o <= 1; flag_request_seen <= reg_request_sync[1]; flag_wait_active <= 1;
			end
			if(request_valid_o && i_request_ready)begin request_valid_o <= 0; end
			// 只有本次在途完整route匹配的真实输出锁存，才允许返回源域。
			if(i_active_valid && o_active_ready && flag_wait_active && i_active_route == request_route_o)begin
				reg_return <= i_active_route; flag_return_toggle <= !flag_return_toggle; flag_wait_active <= 0;
			end
		end
	end
endmodule

