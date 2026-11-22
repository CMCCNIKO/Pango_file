//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-23
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 参数化宽度的稳定载荷跨时钟事务邮箱，通过请求/应答 toggle 传送配置和完成结果，源端一次只保留一项事务。
// Module Name: pcie_image_control_cdc
// File Name: source/pcie/pcie_image_control_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 参数化宽度的稳定载荷跨时钟事务邮箱，通过请求/应答 toggle 传送配置和完成结果，源端一次只保留一项事务。
// READY 表示接收许可，DONE 表示目标帧原子应用或查询完成；目的时钟停顿时保持事务，双域必须协调复位并各自同步释放。
//
// Dependencies: 无子模块；上层提供协调复位和 bundled-data CDC 时序约束。
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
// Create Date: 2026-10-23
// Design Name: PCIe image control transaction mailbox
// Module Name: pcie_image_control_cdc
// Project Name: PG2L100H_AVM_ISP_DUAL_CONTROL
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: One request in flight; source request/config stays stable until
//              destination DONE and response return. READY means acceptance,
//              DONE means the final frame-atomic apply/query has completed.
//              Both resets must assert together and release synchronously to
//              their local clocks. A stopped destination clock may delay a
//              transaction without dropping it. C_WIDTH must be positive.
// Dependencies: Two-stage toggle CDC and stable payload constraints by caller.
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_image_control_cdc
#(
	parameter C_WIDTH = 520
)
(
	input i_src_clk,
	input i_src_rst_n,
	input i_dst_clk,
	input i_dst_rst_n,
	input i_src_valid,
	input i_src_query,
	input [C_WIDTH - 1:0] i_src_config,
	output o_src_ready,
	output o_src_done,
	output [7:0] o_src_status,
	output [C_WIDTH - 1:0] o_src_snapshot,
	output [15:0] o_src_epoch,
	output [1:0] o_src_source,
	output o_dst_valid,
	output o_dst_query,
	output [C_WIDTH - 1:0] o_dst_config,
	input i_dst_ready,
	input i_dst_done,
	input [7:0] i_dst_status,
	input [C_WIDTH - 1:0] i_dst_snapshot,
	input [15:0] i_dst_epoch,
	input [1:0] i_dst_source
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_SEND = 2'd1;
	localparam [1:0] ST_WAIT = 2'd2;
	reg [1:0] state_current;
	reg [1:0] state_next;
	reg flag_src_busy;
	reg flag_request;
	reg flag_ack;
	reg reg_request_seen;
	reg reg_ack_seen;
	(* syn_preserve = 1 *) reg [1:0] reg_request_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_ack_sync;
	reg reg_request_query;
	reg [C_WIDTH - 1:0] reg_request_config;
	reg [7:0] reg_response_status;
	reg [C_WIDTH - 1:0] reg_response_snapshot;
	reg [15:0] reg_response_epoch;
	reg [1:0] reg_response_source;
	reg src_done_o;
	reg [7:0] src_status_o;
	reg [C_WIDTH - 1:0] src_snapshot_o;
	reg [15:0] src_epoch_o;
	reg [1:0] src_source_o;
	reg dst_valid_o;
	reg dst_query_o;
	reg [C_WIDTH - 1:0] dst_config_o;
	wire flag_src_accept;
	wire flag_dst_complete;

	assign flag_src_accept = i_src_valid && !flag_src_busy;
	assign flag_dst_complete = i_dst_done &&
		(state_current == ST_WAIT || (state_current == ST_SEND && i_dst_ready));
	assign o_src_ready = !flag_src_busy && i_src_rst_n;
	assign o_src_done = src_done_o;
	assign o_src_status = src_status_o;
	assign o_src_snapshot = src_snapshot_o;
	assign o_src_epoch = src_epoch_o;
	assign o_src_source = src_source_o;
	assign o_dst_valid = dst_valid_o;
	assign o_dst_query = dst_query_o;
	assign o_dst_config = dst_config_o;

	// 请求和响应分别同步toggle，payload由各自的握手窗口保证稳定。
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_ack_sync <= 2'b00;
		end else begin
			reg_ack_sync <= {reg_ack_sync[0], flag_ack};
		end
	end

	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_request_sync <= 2'b00;
		end else begin
			reg_request_sync <= {reg_request_sync[0], flag_request};
		end
	end

	// 源端冻结整份请求，目标完成后才允许下一次覆盖。
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			flag_src_busy <= 1'b0;
			flag_request <= 1'b0;
			reg_request_query <= 1'b0;
			reg_request_config <= {C_WIDTH{1'b0}};
			reg_ack_seen <= 1'b0;
			src_done_o <= 1'b0;
			src_status_o <= 8'd0;
			src_snapshot_o <= {C_WIDTH{1'b0}};
			src_epoch_o <= 16'd0;
			src_source_o <= 2'd0;
		end else begin
			src_done_o <= 1'b0;
			if(flag_src_accept)begin
				flag_src_busy <= 1'b1;
				flag_request <= !flag_request;
				reg_request_query <= i_src_query;
				reg_request_config <= i_src_config;
			end
			if(flag_src_busy && reg_ack_sync[1] != reg_ack_seen)begin
				reg_ack_seen <= reg_ack_sync[1];
				flag_src_busy <= 1'b0;
				src_done_o <= 1'b1;
				src_status_o <= reg_response_status;
				src_snapshot_o <= reg_response_snapshot;
				src_epoch_o <= reg_response_epoch;
				src_source_o <= reg_response_source;
			end
		end
	end

	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			state_current <= ST_IDLE;
		end else begin
			state_current <= state_next;
		end
	end

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(reg_request_sync[1] != reg_request_seen)begin
					state_next = ST_SEND;
				end
			end
			ST_SEND:begin
				if(i_dst_ready)begin
					if(i_dst_done)begin
						state_next = ST_IDLE;
					end else begin
						state_next = ST_WAIT;
					end
				end
			end
			ST_WAIT:begin
				if(i_dst_done)begin
					state_next = ST_IDLE;
				end
			end
			default:begin
				state_next = ST_IDLE;
			end
		endcase
	end

	// 目标请求允许背压；接收后继续等待最终DONE，不能提前ACK。
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_request_seen <= 1'b0;
			dst_valid_o <= 1'b0;
			dst_query_o <= 1'b0;
			dst_config_o <= {C_WIDTH{1'b0}};
		end else begin
			case(state_current)
				ST_IDLE:begin
					dst_valid_o <= 1'b0;
					if(reg_request_sync[1] != reg_request_seen)begin
						reg_request_seen <= reg_request_sync[1];
						dst_valid_o <= 1'b1;
						dst_query_o <= reg_request_query;
						dst_config_o <= reg_request_config;
					end
				end
				ST_SEND:begin
					if(i_dst_ready)begin
						dst_valid_o <= 1'b0;
					end
				end
				ST_WAIT:begin
					dst_valid_o <= 1'b0;
				end
				default:begin
					dst_valid_o <= 1'b0;
				end
			endcase
		end
	end

	// 响应payload与ACK同拍锁存，并保持至下一次目标完成。
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			flag_ack <= 1'b0;
			reg_response_status <= 8'd0;
			reg_response_snapshot <= {C_WIDTH{1'b0}};
			reg_response_epoch <= 16'd0;
			reg_response_source <= 2'd0;
		end else if(flag_dst_complete)begin
			flag_ack <= !flag_ack;
			reg_response_status <= i_dst_status;
			reg_response_snapshot <= i_dst_snapshot;
			reg_response_epoch <= i_dst_epoch;
			reg_response_source <= i_dst_source;
		end
	end
endmodule
