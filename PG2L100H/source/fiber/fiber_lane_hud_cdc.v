//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026/10/23
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 系统域先寄存五位车道状态，再经原子 mailbox 传入像素域 HUD，busy 时合并中间变化并最终送达最新稳定值。
// Module Name: fiber_lane_hud_cdc
// File Name: source/fiber/fiber_lane_hud_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 系统域先寄存五位车道状态，再经原子 mailbox 传入像素域 HUD，busy 时合并中间变化并最终送达最新稳定值。
// HSST freshness 有效时，即使车道未知/无效/未检出也保持 HSST 来源，不用旧 RK 状态补空；更新仅依赖系统/像素复位，不依赖 PCIe 会话。
//
// Dependencies: vehicle_behavior_action_cdc、已规范化的系统域 LANE-META-V1 快照。
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
//
// Create Date: 2026/10/23
// Design Name: Independent HSST Lane HUD Control
// Module Name: fiber_lane_hud_cdc
// Project Name: PG2L100H_AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 系统域先寄存五位车道状态，再原子送往像素域。
//              fresh 时未知/无效/未检出仍归属 HSST，不允许旧 RK 车道补空。
//              仅依赖系统/像素复位；PCIe 掉链或停钟不阻塞 HUD 更新。
// Dependencies: vehicle_behavior_action_cdc，已规范化的 LANE-META-V1 系统快照。
// Revision:
// Revision 0.01 - 2026/10/23 新增独立窄车道显示 mailbox。
// Additional Comments: i_sys_clk=27 MHz，i_pixel_clk=74.25 MHz。
//                      忙时合并中间状态，持续稳定的最新状态最终送达。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module fiber_lane_hud_cdc
(
	input i_sys_clk,                              // 稳定系统时钟，不依赖 PCIe
	input i_sys_rst_n,                            // 系统域低有效复位
	input [31:0] i_payload,                       // LANE-META-V1 系统域规范化 payload
	input i_valid,                                // 完整包快照有效
	input i_stale,                                // 超时或尚未接收完整包
	input i_pixel_clk,                            // HDMI 输出像素时钟
	input i_pixel_rst_n,                          // 像素域低有效复位
	output [4:0] o_lane_state                      // {fresh,right_type[1:0],left_type[1:0]}
);
	//--------------寄存器信号--------------//
	reg [2:0] reg_sys_reset_sync;                  // 分类寄存器异步断言、系统域同步释放
	reg [4:0] reg_lane_state;                      // 只向 mailbox 提交已寄存的五位状态

	//---------------译码信号---------------//
	wire flag_fresh;                              // 新鲜整包拥有两侧，包括未知值
	wire flag_left_usable;                        // 类型有效且没有明确零条边界
	wire flag_right_usable;                       // 同上，右侧独立归一化
	wire [1:0] dec_left_type;                      // 0 未知、1 实线、2 虚线
	wire [1:0] dec_right_type;                     // 保留类型 3 不传给绘图/行为判定

	//---------------输出信号---------------//
	wire [4:0] lane_state_o;                       // mailbox 像素域原子输出

	//-------------其他信号连线-------------//
	assign flag_fresh = i_valid && !i_stale;
	assign flag_left_usable = flag_fresh && i_payload[22] &&
		(i_payload[17:16] == 2'd1 || i_payload[17:16] == 2'd2) &&
		!(i_payload[20] && i_payload[7:0] == 8'd0);
	assign flag_right_usable = flag_fresh && i_payload[23] &&
		(i_payload[19:18] == 2'd1 || i_payload[19:18] == 2'd2) &&
		!(i_payload[21] && i_payload[15:8] == 8'd0);
	assign dec_left_type = flag_left_usable ? i_payload[17:16] : 2'd0;
	assign dec_right_type = flag_right_usable ? i_payload[19:18] : 2'd0;

	//-------------输出信号连线-------------//
	assign o_lane_state = lane_state_o;             // 已在像素域寄存，不另增加一拍

	//-----------主要任务处理区域-----------//
	// source 分类的复位释放不使用任意 PCIe 时钟、复位或 link 状态。
	always@(posedge i_sys_clk or negedge i_sys_rst_n)begin
		if(!i_sys_rst_n)begin
			reg_sys_reset_sync <= 3'd0;
		end else begin
			reg_sys_reset_sync <= {reg_sys_reset_sync[1:0], 1'b1};
		end
	end

	// 所有组合归一化只驱动本域五位寄存器，CDC payload 由 mailbox 独立保持。
	always@(posedge i_sys_clk or negedge reg_sys_reset_sync[2])begin
		if(!reg_sys_reset_sync[2])begin
			reg_lane_state <= 5'd0;
		end else begin
			reg_lane_state <= {flag_fresh, dec_right_type, dec_left_type};
		end
	end

	//------------模块实例化区域------------//
	// 常开 level/latest-state 通道；任一域复位都会清空在途事务并重发当前值。
	vehicle_behavior_action_cdc
	#(
		.C_DATA_WIDTH(5),
		.C_CHANGE_ONLY(1)
	)
	vehicle_behavior_action_cdc_Inst_lane_display
	(
		.i_src_clk(i_sys_clk),
		.i_src_rst_n(i_sys_rst_n),
		.i_src_valid(1'b1),
		.i_src_payload(reg_lane_state),
		.o_src_ready(),
		.o_src_online(),
		.i_dst_clk(i_pixel_clk),
		.i_dst_rst_n(i_pixel_rst_n),
		.i_dst_online(1'b1),
		.o_dst_valid(),
		.o_dst_payload(lane_state_o)
	);
endmodule
