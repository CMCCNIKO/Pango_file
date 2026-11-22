//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-23
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 实现 BAR1 0x670 导航寄存器窗口，接收导航 shadow/提交命令并分 16 拍完成距离 BCD 转换。
// Module Name: pcie_hud_navigation_regs
// File Name: source/pcie/pcie_hud_navigation_regs.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 实现 BAR1 0x670 导航寄存器窗口，接收导航 shadow/提交命令并分 16 拍完成距离 BCD 转换。
// 一次只允许一项导航事务在途，等待真实 overlay 锁存确认才更新 active/applied，内部 token 隔离旧快照且允许 sequence=0。
//
// Dependencies: pcie_hud_regs.vh；外部稳定载荷 CDC 与协调复位。
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
// Design Name: RK3566 HUD navigation
// Module Name: pcie_hud_navigation_regs
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 0x670导航窗口，单事务、16拍BCD转换、真实overlay锁存确认。
// Dependencies: pcie_hud_regs.vh；外部稳定payload CDC与协调复位。
// Revision: 0.01 - File Created
// Additional Comments: sequence允许0；内部token隔离旧快照，不改变用户ABI。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`include "pcie_hud_regs.vh"
module pcie_hud_navigation_regs
(
	input i_clk,
	input i_rstn,
	input i_ready,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	output [127:0] o_rd_window,
	output o_send,
	output [83:0] o_payload,
	input i_snapshot_valid,
	input [64:0] i_snapshot
);
	reg [31:0] reg_shadow;
	reg [31:0] reg_accepted;
	reg [31:0] reg_active;
	reg [15:0] reg_accepted_seq;
	reg [15:0] reg_applied_seq;
	reg [15:0] reg_token;
	reg flag_accepted;
	reg flag_applied;
	reg flag_busy;
	reg flag_convert;
	reg flag_pending;
	reg [3:0] cnt_convert;
	reg [35:0] reg_conversion;
	reg [35:0] dec_conversion;
	reg [83:0] payload_o;
	reg [7:0] reg_error;
	reg [15:0] cnt_reject;
	reg [7:0] dec_error;
	wire flag_window;
	wire flag_duplicate;
	wire flag_accept;
	wire flag_clear;
	integer digit;
	assign flag_window = i_wr_en && i_wr_addr[11:4] == 8'h67;
	assign flag_duplicate = flag_accepted && i_wr_data[31:16] == reg_accepted_seq && reg_shadow == reg_accepted;
	assign flag_accept = flag_window && i_wr_addr == `HUD_REG_NAV_CONTROL &&
		i_wr_data[15:0] == 16'd1 && dec_error == 8'd0 && !flag_duplicate;
	assign flag_clear = flag_window && i_wr_addr == `HUD_REG_NAV_CONTROL &&
		i_wr_data[15:0] == 16'd2 && dec_error == 8'd0;
	assign o_send = i_rstn && flag_pending && i_ready;
	assign o_payload = payload_o;
	assign o_rd_window = {reg_active,
		cnt_reject, reg_error, 2'd0, (reg_error != 8'd0), flag_accepted, i_ready,
		reg_active[0], flag_applied, flag_busy, reg_accepted_seq, reg_applied_seq, reg_shadow};

	// 保留位、DWORD访问和单事务保护；重复同一请求可安全轮询重试。
	always@(*)begin
		dec_error = 8'd0;
		if(i_wr_strb != 4'hf || i_wr_addr[1:0] != 2'd0)begin
			dec_error = 8'd2;
		end else if(i_wr_addr == `HUD_REG_NAV_SHADOW)begin
			dec_error = 8'd0;
		end else if(i_wr_addr != `HUD_REG_NAV_CONTROL)begin
			dec_error = 8'd7;
		end else if(i_wr_data[15:0] == 16'd2)begin
			dec_error = 8'd0;
		end else if(i_wr_data[15:0] != 16'd1)begin
			dec_error = 8'd3;
		end else if(reg_shadow[15:3] != 13'd0 || reg_shadow[2:1] == 2'd3)begin
			dec_error = 8'd4;
		end else if(flag_duplicate)begin
			dec_error = 8'd0;
		end else if(flag_accepted && i_wr_data[31:16] == reg_accepted_seq)begin
			dec_error = 8'd6;
		end else if(flag_busy)begin
			dec_error = 8'd1;
		end else if(!i_ready)begin
			dec_error = 8'd8;
		end
	end

	// 五位BCD逐拍double-dabble；没有像素域除法、乘法或新增IP。
	always@(*)begin
		dec_conversion = reg_conversion;
		for(digit = 0; digit < 5; digit = digit + 1)begin
			if(reg_conversion[16 + digit * 4 +: 4] >= 4'd5)begin
				dec_conversion[16 + digit * 4 +: 4] = reg_conversion[16 + digit * 4 +: 4] + 4'd3;
			end
		end
		dec_conversion = {dec_conversion[34:0], 1'b0};
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_shadow <= 32'd0;
			reg_accepted <= 32'd0;
			reg_active <= 32'd0;
			reg_accepted_seq <= 16'd0;
			reg_applied_seq <= 16'd0;
			reg_token <= 16'd0;
			flag_accepted <= 1'b0;
			flag_applied <= 1'b0;
			flag_busy <= 1'b0;
			flag_convert <= 1'b0;
			flag_pending <= 1'b0;
			cnt_convert <= 4'd0;
			reg_conversion <= 36'd0;
			payload_o <= 84'd0;
			reg_error <= 8'd0;
			cnt_reject <= 16'd0;
		end else begin
			if(o_send)begin
				flag_pending <= 1'b0;
			end
			if(flag_convert)begin
				reg_conversion <= dec_conversion;
				cnt_convert <= cnt_convert + 4'd1;
				if(cnt_convert == 4'd15)begin
					payload_o <= {reg_token, reg_accepted_seq, dec_conversion[35:16], reg_accepted};
					flag_convert <= 1'b0;
					flag_pending <= 1'b1;
				end
			end
			if(i_snapshot_valid)begin
				reg_active <= i_snapshot[31:0];
				reg_applied_seq <= i_snapshot[47:32];
				flag_applied <= i_snapshot[64];
				if(i_snapshot[64] && flag_busy && i_snapshot[63:48] == reg_token &&
					i_snapshot[47:32] == reg_accepted_seq && i_snapshot[31:0] == reg_accepted)begin
					flag_busy <= 1'b0;
				end
			end
			if(flag_accept)begin
				reg_accepted <= reg_shadow;
				reg_accepted_seq <= i_wr_data[31:16];
				reg_token <= reg_token + 16'd1;
				flag_accepted <= 1'b1;
				flag_busy <= 1'b1;
				flag_convert <= 1'b1;
				cnt_convert <= 4'd0;
				reg_conversion <= {20'd0, reg_shadow[31:16]};
			end
			if(flag_clear)begin
				reg_error <= 8'd0;
				cnt_reject <= 16'd0;
			end
			if(flag_window && dec_error != 8'd0)begin
				reg_error <= dec_error;
				cnt_reject <= cnt_reject + 16'd1;
			end
			if(flag_window && dec_error == 8'd0 && i_wr_addr == `HUD_REG_NAV_SHADOW)begin
				reg_shadow <= i_wr_data;
			end
		end
	end
endmodule
