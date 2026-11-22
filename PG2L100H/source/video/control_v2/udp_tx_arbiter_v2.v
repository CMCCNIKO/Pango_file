//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在控制 ACK 与其他 UDP 发送源之间仲裁，ACK 优先并按整包锁定直到所选源专属发送完成。
// Module Name: udp_tx_arbiter_v2
// File Name: source/video/control_v2/udp_tx_arbiter_v2.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在控制 ACK 与其他 UDP 发送源之间仲裁，ACK 优先并按整包锁定直到所选源专属发送完成。
// 向无逐字节反压的 UDP 栈连续提供字节，只向当前所有者回传请求和 done。
//
// Dependencies: udp_ip_mac_top 的所有权限定 request/done 及各数据源。
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
// Create Date: 2026-10-10
// Design Name: HDMI algorithm UDP control
// Module Name: udp_tx_arbiter_v2
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: ACK 优先；整包锁定至专属 UDP 完成；连续字节流无背压。
// Dependencies: udp_ip_mac_top ownership-qualified request/done
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module udp_tx_arbiter_v2
(
	input i_clk,
	input i_rstn,
	input i_ack_valid,
	input [143:0] i_ack_payload,
	input [15:0] i_ack_length,
	output o_ack_ready,
	input i_test_request,
	input i_network_ready,
	output o_need_network,
	output o_tx_busy,
	output o_tx_done,
	output o_udp_request,
	output [15:0] o_udp_length,
	output o_udp_valid,
	output [7:0] o_udp_data,
	input i_udp_data_request,
	input i_udp_done
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_REQUEST = 2'd1;
	localparam [1:0] ST_BYTES = 2'd2;
	localparam [1:0] ST_FINISH = 2'd3;
	localparam [159:0] C_TEST_PAYLOAD = 160'h7777772e6d65796573656d692e636f6d2020200a;
	reg [1:0] state_current;
	reg [1:0] state_next;
	reg flag_test_pending;
	reg [159:0] reg_payload;
	reg [15:0] length_o;
	reg [4:0] cnt_byte;
	reg valid_o;
	reg [7:0] data_o;
	wire select_ack;
	wire select_test;
	assign o_ack_ready = i_rstn && state_current == ST_IDLE && i_network_ready;
	assign select_ack = o_ack_ready && i_ack_valid;
	assign select_test = o_ack_ready && !i_ack_valid && (flag_test_pending || i_test_request);
	assign o_need_network = i_ack_valid || flag_test_pending || i_test_request;
	assign o_tx_busy = state_current != ST_IDLE;
	assign o_tx_done = state_current == ST_FINISH && i_udp_done;
	assign o_udp_request = state_current == ST_REQUEST;
	assign o_udp_length = length_o;
	assign o_udp_valid = valid_o;
	assign o_udp_data = data_o;

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(select_ack || select_test)begin state_next = ST_REQUEST; end
			end
			ST_REQUEST:begin
				if(i_udp_data_request)begin state_next = ST_BYTES; end
			end
			ST_BYTES:begin
				if({11'd0, cnt_byte} == length_o - 16'd1)begin state_next = ST_FINISH; end
			end
			ST_FINISH:begin
				if(i_udp_done)begin state_next = ST_IDLE; end
			end
			default:begin state_next = ST_IDLE; end
		endcase
	end
	always@(posedge i_clk)begin
		if(!i_rstn)begin state_current <= ST_IDLE; end
		else begin state_current <= state_next; end
	end
	// 周期请求合并为一个 pending，ACK 同拍优先时不丢周期包。
	always@(posedge i_clk)begin
		if(!i_rstn)begin flag_test_pending <= 1'b0; end
		else if(select_test)begin flag_test_pending <= 1'b0; end
		else if(i_test_request)begin flag_test_pending <= 1'b1; end
	end
	always@(posedge i_clk)begin
		if(!i_rstn)begin
			reg_payload <= 160'd0;
			length_o <= 16'd0;
		end else if(select_ack)begin
			reg_payload <= {i_ack_payload, 16'd0};
			length_o <= i_ack_length;
		end else if(select_test)begin
			reg_payload <= C_TEST_PAYLOAD;
			length_o <= 16'd20;
		end
	end
	// 保留旧发送器在 data request 后的注册首字节延迟。
	always@(posedge i_clk)begin
		if(!i_rstn)begin
			cnt_byte <= 5'd0;
			valid_o <= 1'b0;
			data_o <= 8'd0;
		end else if(state_current == ST_BYTES)begin
			valid_o <= 1'b1;
			data_o <= reg_payload[159 - {cnt_byte, 3'b000} -: 8];
			cnt_byte <= cnt_byte + 5'd1;
		end else begin
			cnt_byte <= 5'd0;
			valid_o <= 1'b0;
			data_o <= 8'd0;
		end
	end
endmodule
