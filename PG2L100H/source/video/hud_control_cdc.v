//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-12
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 PCIe HUD 稳定请求送到像素域，并通过单项反向邮箱返回真实 overlay 锁存状态快照。
// Module Name: hud_control_cdc
// File Name: source/video/hud_control_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 PCIe HUD 稳定请求送到像素域，并通过单项反向邮箱返回真实 overlay 锁存状态快照。
// 请求保持到真实提交确认，状态载荷保持到读侧确认；周期状态允许合并，CSR 一次只保留一项在途事务。
//
// Dependencies: 调用方协调复位与本域同步释放；PCIe CSR 和像素域 HUD 锁存。
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
// Create Date: 2026-10-12
// Design Name: PCIe HUD bundled-data control CDC
// Module Name: hud_control_cdc
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 稳定请求跨域；真实HUD快照以单项握手返回，可合并周期状态。
// Dependencies: 调用方协调异步断言/各域同步释放；CSR保持一个在途事务。
// Revision: 0.01 - File Created
// Additional Comments: 请求保持至真实提交返回，状态mailbox保持至读侧确认。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hud_control_cdc
#(
	parameter integer C_SNAPSHOT_WIDTH = 43
)
(
	input i_pcie_clk,
	input i_pcie_rstn,
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_send,
	input [31:0] i_request,
	output o_ready,
	output o_command_valid,
	output [31:0] o_command,
	input i_snapshot_valid,
	input [C_SNAPSHOT_WIDTH - 1:0] i_snapshot,
	output o_snapshot_valid,
	output [C_SNAPSHOT_WIDTH - 1:0] o_snapshot
);
	reg [31:0] reg_request_payload;
	reg reg_request_toggle;
	reg [1:0] reg_request_sync;
	reg reg_request_seen;
	reg command_valid_o;
	reg [31:0] command_o;
	reg [1:0] reg_pixel_up_sync;
	reg [C_SNAPSHOT_WIDTH - 1:0] reg_snapshot_latest;
	reg flag_snapshot_dirty;
	reg [C_SNAPSHOT_WIDTH - 1:0] reg_snapshot_payload;
	reg reg_snapshot_toggle;
	reg [1:0] reg_snapshot_ack_sync;
	reg [1:0] reg_snapshot_sync;
	reg reg_snapshot_seen;
	reg snapshot_valid_o;
	reg [C_SNAPSHOT_WIDTH - 1:0] snapshot_o;
	wire flag_snapshot_send;

	assign o_ready = reg_pixel_up_sync[1];
	assign o_command_valid = command_valid_o;
	assign o_command = command_o;
	assign o_snapshot_valid = snapshot_valid_o;
	assign o_snapshot = snapshot_o;
	assign flag_snapshot_send = flag_snapshot_dirty && (reg_snapshot_toggle == reg_snapshot_ack_sync[1]);

	// 源域只在CSR接受时推进toggle；payload保持至applied，不能被busy写改动。
	always@(posedge i_pcie_clk or negedge i_pcie_rstn)begin
		if(!i_pcie_rstn)begin
			reg_request_payload <= 32'd0;
			reg_request_toggle <= 1'b0;
		end else if(i_send)begin
			reg_request_payload <= i_request;
			reg_request_toggle <= !reg_request_toggle;
		end
	end
	always@(posedge i_pcie_clk or negedge i_pcie_rstn)begin
		if(!i_pcie_rstn)begin
			reg_pixel_up_sync <= 2'd0;
		end else begin
			reg_pixel_up_sync <= {reg_pixel_up_sync[0], i_pixel_rstn};
		end
	end
	// 只同步请求事件；多位载荷受保持契约及精确max-delay约束保护。
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_request_sync <= 2'd0;
		end else begin
			reg_request_sync <= {reg_request_sync[0], reg_request_toggle};
		end
	end
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_request_seen <= 1'b0;
			command_valid_o <= 1'b0;
			command_o <= 32'd0;
		end else begin
			command_valid_o <= 1'b0;
			if(reg_request_sync[1] != reg_request_seen)begin
				reg_request_seen <= reg_request_sync[1];
				command_o <= reg_request_payload;
				command_valid_o <= 1'b1;
			end
		end
	end

	// 周期状态可合并；applied请求记录持久保存在像素域，每次快照都携带。
	// 因源域仅允许一个在途请求，合并不会越过尚未确认的提交事务。
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_snapshot_latest <= {C_SNAPSHOT_WIDTH{1'b0}};
			flag_snapshot_dirty <= 1'b0;
		end else begin
			if(flag_snapshot_send)begin
				flag_snapshot_dirty <= 1'b0;
			end
			if(i_snapshot_valid)begin
				reg_snapshot_latest <= i_snapshot;
				flag_snapshot_dirty <= 1'b1;
			end
		end
	end
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_snapshot_payload <= {C_SNAPSHOT_WIDTH{1'b0}};
			reg_snapshot_toggle <= 1'b0;
		end else if(flag_snapshot_send)begin
			reg_snapshot_payload <= reg_snapshot_latest;
			reg_snapshot_toggle <= !reg_snapshot_toggle;
		end
	end
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_snapshot_ack_sync <= 2'd0;
		end else begin
			reg_snapshot_ack_sync <= {reg_snapshot_ack_sync[0], reg_snapshot_seen};
		end
	end
	always@(posedge i_pcie_clk or negedge i_pcie_rstn)begin
		if(!i_pcie_rstn)begin
			reg_snapshot_sync <= 2'd0;
		end else begin
			reg_snapshot_sync <= {reg_snapshot_sync[0], reg_snapshot_toggle};
		end
	end
	always@(posedge i_pcie_clk or negedge i_pcie_rstn)begin
		if(!i_pcie_rstn)begin
			reg_snapshot_seen <= 1'b0;
			snapshot_valid_o <= 1'b0;
			snapshot_o <= {C_SNAPSHOT_WIDTH{1'b0}};
		end else begin
			snapshot_valid_o <= 1'b0;
			if(reg_snapshot_sync[1] != reg_snapshot_seen)begin
				reg_snapshot_seen <= reg_snapshot_sync[1];
				snapshot_o <= reg_snapshot_payload;
				snapshot_valid_o <= 1'b1;
			end
		end
	end
endmodule
