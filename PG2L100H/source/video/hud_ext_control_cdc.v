//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-17
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 HUD 扩展场景及行为快照通过单项稳定载荷邮箱跨域，载荷宽度可参数化。
// Module Name: hud_ext_control_cdc
// File Name: source/video/hud_ext_control_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 HUD 扩展场景及行为快照通过单项稳定载荷邮箱跨域，载荷宽度可参数化。
// 一个提交事件对应一次握手，busy 时禁止覆盖在途快照，目的域采样后回传确认。
//
// Dependencies: 源/目的时钟、协调复位和 stable-payload 时序约束。
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
// Create Date: 2026-10-17
// Module Name: hud_ext_control_cdc
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: HUD扩展场景/行为快照的单项稳定载荷CDC，可配置载荷宽度。
//              只传递一次提交事件，不增加哈希或复杂门禁。
// Revision: 0.02 - 2026-10-21 参数化快照位宽，busy时禁止覆盖在途载荷。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hud_ext_control_cdc #(
	parameter C_PAYLOAD_WIDTH = 112
)(
	input i_pcie_clk,
	input i_pcie_rstn,
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_send,
	input [C_PAYLOAD_WIDTH-1:0] i_payload,
	output o_ready,
	output o_command_valid,
	output [C_PAYLOAD_WIDTH-1:0] o_command
);
	reg [C_PAYLOAD_WIDTH-1:0] reg_payload;
	reg reg_toggle;
	reg [1:0] reg_ack_sync;
	reg [1:0] reg_toggle_sync;
	reg reg_seen;
	reg reg_ack_toggle;
	reg command_valid_o;
	reg [C_PAYLOAD_WIDTH-1:0] command_o;
	assign o_ready = reg_toggle == reg_ack_sync[1];
	assign o_command_valid = command_valid_o;
	assign o_command = command_o;
	always@(posedge i_pcie_clk or negedge i_pcie_rstn)begin
		if(!i_pcie_rstn)begin
			reg_payload <= {C_PAYLOAD_WIDTH{1'b0}};
			reg_toggle <= 1'b0;
			reg_ack_sync <= 2'd0;
		end else begin
			reg_ack_sync <= {reg_ack_sync[0], reg_ack_toggle};
			if(i_send && o_ready)begin
				reg_payload <= i_payload;
				reg_toggle <= !reg_toggle;
			end
		end
	end
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_toggle_sync <= 2'd0;
			reg_seen <= 1'b0;
			reg_ack_toggle <= 1'b0;
			command_valid_o <= 1'b0;
			command_o <= {C_PAYLOAD_WIDTH{1'b0}};
		end else begin
			command_valid_o <= 1'b0;
			reg_toggle_sync <= {reg_toggle_sync[0], reg_toggle};
			if(reg_toggle_sync[1] != reg_seen)begin
				reg_seen <= reg_toggle_sync[1];
				reg_ack_toggle <= reg_toggle_sync[1];
				command_o <= reg_payload;
				command_valid_o <= 1'b1;
			end
		end
	end
endmodule
