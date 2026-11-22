//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-12
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: PCIe 域 HUD 控制寄存器，接收基础请求、扩展场景/行为快照及导航事务，并维护接受、应用、拒绝和错误状态。
// Module Name: pcie_hud_regs
// File Name: source/pcie/pcie_hud_regs.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// PCIe 域 HUD 控制寄存器，接收基础请求、扩展场景/行为快照及导航事务，并维护接受、应用、拒绝和错误状态。
// COMMIT 冻结整项事件，只有真实 HUD 显示锁存回报才确认生效；非法 TLP 由零 strobe 通知并拒绝。
//
// Dependencies: pcie_hud_regs.vh、pcie_hud_navigation_regs；外部稳定载荷握手 CDC。
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
// Design Name: PCIe HUD v2 control (v1 CSR-compatible front interface)
// Module Name: pcie_hud_regs
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: PCIe域CSR，接收一次完整请求；真实HUD锁存快照确认生效。
//              i_rstn由wrapper联合复位并在本域同步释放；非法TLP以strb=0通知。
// Dependencies: source/pcie/pcie_hud_regs.vh，外部稳定载荷握手CDC
// Revision: 0.02 - HUD v2 extension with v1 CSR-compatible REQUEST
// Revision: 0.03 - 2026-10-21 冻结每次COMMIT事件，扩展状态回读真实显示发送器。
// Revision: 0.04 - 2026-10-23 REQUEST独立译码，快照ID按8位并行进位，保持同沿可见。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`include "pcie_hud_regs.vh"

module pcie_hud_regs
(
	input i_clk,
	input i_rstn,
	input i_ready,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window,
	output o_send,
	output [31:0] o_request,
	input i_ext_ready,
	input i_ext_pending,
	output o_ext_commit_valid,
	output [111:0] o_ext_commit_payload,
	input i_snapshot_valid,
	input [123:0] i_snapshot,
	input i_nav_ready,
	output o_nav_send,
	output [83:0] o_nav_payload
);
	//--------------寄存器信号--------------//
	reg flag_busy;
	reg flag_accepted;
	reg flag_applied;
	reg flag_reject;
	reg flag_effective;
	reg flag_override;
	reg reg_ready;
	reg [31:0] reg_accepted_request;
	reg [31:0] reg_applied_request;
	reg [8:0] reg_effective;
	reg [31:0] cnt_reject;
	reg [31:0] reg_error;
	reg [15:0] reg_rejected_seq;
	reg [31:0] cnt_snapshot;
	reg [31:0] reg_ext_request;
	reg [31:0] reg_time0;
	reg [31:0] reg_time1;
	reg [15:0] reg_ext_seq;
	reg [15:0] reg_ext_applied_seq;
	reg [111:0] ext_commit_payload_o;
	reg ext_commit_valid_o;
	//---------------译码信号---------------//
	reg [31:0] dec_error;
	reg flag_reject_seq;
	wire flag_window;
	wire flag_duplicate;
	wire flag_request;
	wire flag_request_fields_valid;
	wire flag_accept;
	wire flag_visible_change;
	wire [3:0] flag_snapshot_increment;
	wire flag_ext_window;
	wire flag_ext_accept;
	wire [127:0] nav_rd_window;
	reg [127:0] rd_window_o;
	reg [31:0] rd_word;
	reg [11:0] rd_address;
	integer rd_lane;

	assign flag_window = i_wr_en && i_wr_addr[11:7] == 5'h0c && i_wr_addr[11:4] != 8'h67;
	assign flag_ext_window = i_wr_en && i_wr_addr[11:7] == 5'h0c;
	assign flag_duplicate = flag_accepted && i_wr_data == reg_accepted_request;
	// REQUEST地址天然不属于NAV；接受路径不再经过窗口排除和错误码优先编码。
	// 同序号无论同/异载荷均不产生新接受事件；幂等/拒绝仍由原错误优先级管理。
	assign flag_request = i_wr_en && i_wr_addr == `HUD_REG_REQUEST;
	assign flag_request_fields_valid = i_wr_data[15:13] == 3'd0 &&
		(i_wr_data[12] || i_wr_data[11:0] == 12'd0) &&
		(!i_wr_data[9] || i_wr_data[1]) &&
		(!i_wr_data[10] || i_wr_data[2]) &&
		(!i_wr_data[11] || i_wr_data[3]) && i_wr_data[31:16] != 16'd0;
	assign flag_accept = flag_request && i_wr_strb == 4'hf && flag_request_fields_valid &&
		(!flag_accepted || i_wr_data[31:16] != reg_accepted_request[31:16]) && i_ready && !flag_busy;
	assign flag_ext_accept = i_rstn && flag_ext_window && i_wr_strb == 4'hf &&
		i_wr_addr == `HUD_REG_EXT_COMMIT && i_wr_data[15:0] != 16'd0;
	assign o_ext_commit_valid = ext_commit_valid_o;
	assign o_ext_commit_payload = ext_commit_payload_o;
	// 任意可读状态变化时，同沿增加ID；重复请求/重复快照不增加。
	assign flag_visible_change = (reg_ready != i_ready) || flag_accept ||
		(flag_window && dec_error != 32'd0) ||
		(i_snapshot_valid && (reg_ext_applied_seq != i_snapshot[58:43] ||
		!flag_effective || reg_effective != i_snapshot[8:0] ||
		flag_override != i_snapshot[9] ||
		(i_snapshot[42] && (!flag_applied || reg_applied_request != i_snapshot[41:10] ||
		(flag_busy && i_snapshot[41:10] == reg_accepted_request)))));

	// 四段均由当前值并行判断进位；事件仍同沿加1并按32位自然回绕。
	// 每段只做8位加法，避免变化检测后再串过整条32位CECO链。
	assign flag_snapshot_increment[0] = flag_visible_change;
	assign flag_snapshot_increment[1] = flag_visible_change && (&cnt_snapshot[7:0]);
	assign flag_snapshot_increment[2] = flag_visible_change && (&cnt_snapshot[15:0]);
	assign flag_snapshot_increment[3] = flag_visible_change && (&cnt_snapshot[23:0]);
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_snapshot <= 32'd0;
		end else begin
			cnt_snapshot[7:0] <= cnt_snapshot[7:0] + {7'd0, flag_snapshot_increment[0]};
			cnt_snapshot[15:8] <= cnt_snapshot[15:8] + {7'd0, flag_snapshot_increment[1]};
			cnt_snapshot[23:16] <= cnt_snapshot[23:16] + {7'd0, flag_snapshot_increment[2]};
			cnt_snapshot[31:24] <= cnt_snapshot[31:24] + {7'd0, flag_snapshot_increment[3]};
		end
	end

	// 访问与字段优先于幂等；只有完整REQUEST写能关联拒绝序号。
	always@(*)begin
		dec_error = 32'd0;
		flag_reject_seq = 1'b0;
		if(i_wr_strb != 4'hf || i_wr_addr[1:0] != 2'b00)begin
			dec_error = 32'd2;
		end else if(i_wr_addr == `HUD_REG_EXT_REQUEST || i_wr_addr == `HUD_REG_TIME0 ||
			i_wr_addr == `HUD_REG_TIME1 || i_wr_addr == `HUD_REG_EXT_COMMIT)begin
			dec_error = 32'd0;
		end else if(i_wr_addr != `HUD_REG_REQUEST)begin
			dec_error = 32'd7;
		end else begin
			flag_reject_seq = 1'b1;
			if(i_wr_data[15:13] != 3'd0)begin
				dec_error = 32'd3;
			end else if((!i_wr_data[12] && i_wr_data[11:0] != 12'd0) ||
				(i_wr_data[9] && !i_wr_data[1]) ||
				(i_wr_data[10] && !i_wr_data[2]) ||
				(i_wr_data[11] && !i_wr_data[3]))begin
				dec_error = 32'd4;
			end else if(i_wr_data[31:16] == 16'd0)begin
				dec_error = 32'd5;
			end else if(flag_duplicate)begin
				dec_error = 32'd0;
			end else if(flag_accepted && i_wr_data[31:16] == reg_accepted_request[31:16])begin
				dec_error = 32'd6;
			end else if(!i_ready)begin
				dec_error = 32'd8;
			end else if(flag_busy)begin
				dec_error = 32'd1;
			end
		end
	end

	// 原子管理操作状态：快照由真实overlay锁存产生；新接收优先置busy。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			flag_busy <= 1'b0;
			flag_accepted <= 1'b0;
			flag_applied <= 1'b0;
			flag_reject <= 1'b0;
			flag_effective <= 1'b0;
			flag_override <= 1'b0;
			reg_ready <= 1'b0;
			reg_accepted_request <= 32'd0;
			reg_applied_request <= 32'd0;
			reg_effective <= 9'd0;
			cnt_reject <= 32'd0;
			reg_error <= 32'd0;
			reg_rejected_seq <= 16'd0;
			reg_ext_request <= 32'd0;
			reg_time0 <= 32'd0;
			reg_time1 <= 32'd0;
			reg_ext_seq <= 16'd0;
			reg_ext_applied_seq <= 16'd0;
			ext_commit_payload_o <= 112'd0;
			ext_commit_valid_o <= 1'b0;
		end else begin
			// 每次接收均交付冻结事件，显示背压不抑制识别计时刷新。
			ext_commit_valid_o <= flag_ext_accept;
			if(flag_ext_accept)begin
				ext_commit_payload_o <= {i_wr_data[15:0], reg_time1, reg_time0, reg_ext_request};
			end
			reg_ready <= i_ready;
			if(i_snapshot_valid)begin
				reg_ext_applied_seq <= i_snapshot[58:43];
				flag_effective <= 1'b1;
				flag_override <= i_snapshot[9];
				reg_effective <= i_snapshot[8:0];
				if(i_snapshot[42])begin
					flag_applied <= 1'b1;
					reg_applied_request <= i_snapshot[41:10];
					if(flag_busy && i_snapshot[41:10] == reg_accepted_request)begin
						flag_busy <= 1'b0;
					end
				end
			end
			if(flag_accept)begin
				flag_busy <= 1'b1;
				flag_accepted <= 1'b1;
				reg_accepted_request <= i_wr_data;
			end
			if(flag_window && dec_error != 32'd0)begin
				cnt_reject <= cnt_reject + 32'd1;
				reg_error <= dec_error;
				flag_reject <= flag_reject_seq;
				reg_rejected_seq <= flag_reject_seq ? i_wr_data[31:16] : 16'd0;
			end
			if(i_wr_en && i_wr_strb == 4'hf)begin
				case(i_wr_addr)
					`HUD_REG_EXT_REQUEST: reg_ext_request <= i_wr_data;
					`HUD_REG_TIME0: reg_time0 <= i_wr_data;
					`HUD_REG_TIME1: reg_time1 <= i_wr_data;
					`HUD_REG_EXT_COMMIT: begin
						if(i_wr_data[15:0] != 16'd0)begin
							reg_ext_seq <= i_wr_data[15:0];
						end
					end
					default: begin end
				endcase
			end
		end
	end

	// 对齐16字节窗口中的四个DWORD保持原lane，配合PCIe完成包lane选择。
	always@(*)begin
		rd_window_o = 128'd0;
		rd_word = 32'd0;
		rd_address = 12'd0;
		for(rd_lane = 0; rd_lane < 4; rd_lane = rd_lane + 1)begin
			rd_address = {i_rd_addr[11:4],rd_lane[1:0],2'b00};
			rd_word = 32'd0;
			case(rd_address)
				`HUD_REG_MAGIC:begin rd_word = `HUD_MAGIC; end
				`HUD_REG_VERSION:begin rd_word = `HUD_VERSION; end
				`HUD_REG_STATUS:begin rd_word = {24'd0,reg_ready,flag_effective,flag_reject,flag_applied,flag_accepted,flag_override,reg_applied_request[12],flag_busy}; end
				`HUD_REG_ACCEPTED_SEQ:begin rd_word = {16'd0,reg_accepted_request[31:16]}; end
				`HUD_REG_APPLIED_SEQ:begin rd_word = {16'd0,reg_applied_request[31:16]}; end
				`HUD_REG_RK_STATE:begin rd_word = {19'd0,reg_applied_request[12:0]}; end
				`HUD_REG_EFFECTIVE_STATE:begin rd_word = {23'd0,reg_effective}; end
				`HUD_REG_REJECT_COUNT:begin rd_word = cnt_reject; end
				`HUD_REG_LAST_ERROR:begin rd_word = reg_error; end
				`HUD_REG_REJECTED_SEQ:begin rd_word = {16'd0,reg_rejected_seq}; end
				`HUD_REG_SNAPSHOT_ID:begin rd_word = cnt_snapshot; end
				`HUD_REG_CAPS:begin rd_word = `HUD_CAPS; end
				`HUD_REG_LAST_ACCEPTED_REQUEST:begin rd_word = reg_accepted_request; end
				`HUD_REG_EXT_REQUEST:begin rd_word = reg_ext_request; end
				`HUD_REG_TIME0:begin rd_word = reg_time0; end
				`HUD_REG_TIME1:begin rd_word = reg_time1; end
				`HUD_REG_EXT_COMMIT:begin rd_word = {16'd0,reg_ext_seq}; end
				`HUD_REG_EXT_STATUS:begin rd_word = {30'd0,i_ext_ready,i_ext_pending}; end
				`HUD_REG_EXT_APPLIED_SEQ:begin rd_word = {16'd0,reg_ext_applied_seq}; end
				default:begin rd_word = 32'd0; end
			endcase
			rd_window_o[rd_lane * 32 +: 32] = rd_word;
		end
	end
	assign o_rd_window = i_rd_addr[11:4] == 8'h67 ? nav_rd_window : rd_window_o;
	pcie_hud_navigation_regs pcie_hud_navigation_regs_Inst_nav(
		.i_clk(i_clk),
		.i_rstn(i_rstn),
		.i_ready(i_nav_ready && i_ready),
		.i_wr_en(i_wr_en),
		.i_wr_addr(i_wr_addr),
		.i_wr_data(i_wr_data),
		.i_wr_strb(i_wr_strb),
		.o_rd_window(nav_rd_window),
		.o_send(o_nav_send),
		.o_payload(o_nav_payload),
		.i_snapshot_valid(i_snapshot_valid),
		.i_snapshot(i_snapshot[123:59])
	);
	// send与payload同沿由外部CDC采样，避免NBA后一拍错位。
	assign o_send = i_rstn && flag_accept;
	assign o_request = i_wr_data;
endmodule
