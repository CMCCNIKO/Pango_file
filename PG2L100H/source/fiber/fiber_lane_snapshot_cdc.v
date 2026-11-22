//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-21
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将光纤完整车道元数据经 64-bit 原子 mailbox 跨到独立系统域，busy 时保留最新快照，不反压视频。
// Module Name: fiber_lane_snapshot_cdc
// File Name: source/fiber/fiber_lane_snapshot_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将光纤完整车道元数据经 64-bit 原子 mailbox 跨到独立系统域，busy 时保留最新快照，不反压视频。
// 任一复位或链路丢失清除两端事务，恢复后必须等新快照；系统时钟计 freshness，RX 停钟仍可超时，重复值也刷新。
//
// Dependencies: vehicle_behavior_action_cdc；C_SYS_CLK_HZ>=1000、C_STALE_MS>=1。
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
// Create Date: 2026-10-21
// Design Name: Fiber lane metadata snapshot
// Module Name: fiber_lane_snapshot_cdc
// Project Name: PG2L100H_AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 光纤完整元数据原子跨域，忙时保留最新快照，不反压视频。
//              任一复位或链路丢失清除两端事务；恢复后必须收到新快照。
//              稳定系统域计时，RX 时钟停止仍会超时；重复值也刷新。
//              参数要求 C_SYS_CLK_HZ >= 1000，C_STALE_MS >= 1。
//              非整数毫秒时钟周期向上取整，超时不会提前发生。
// Dependencies: vehicle_behavior_action_cdc (64-bit event mailbox)
// Revision:
// Revision 0.01 - 2026-10-21 Independent fiber snapshot and stale timeout.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module fiber_lane_snapshot_cdc
#(
	parameter integer C_SYS_CLK_HZ = 27000000,
	parameter integer C_STALE_MS = 500
)
(
	input i_rx_clk,
	input i_rx_rst_n,
	input i_rx_link_ready,
	input i_snapshot_valid,
	input [63:0] i_snapshot,
	input i_sys_clk,
	input i_sys_rst_n,
	output o_update,
	output [63:0] o_snapshot,
	output o_valid,
	output o_stale
);
	//-------------配置参数区域-------------//
	localparam integer TICK_CYCLES = C_SYS_CLK_HZ / 1000 +
		((C_SYS_CLK_HZ % 1000) != 0);
	localparam integer TICK_WIDTH =
		TICK_CYCLES <= 2 ? 1 : TICK_CYCLES <= 4 ? 2 :
		TICK_CYCLES <= 8 ? 3 : TICK_CYCLES <= 16 ? 4 :
		TICK_CYCLES <= 32 ? 5 : TICK_CYCLES <= 64 ? 6 :
		TICK_CYCLES <= 128 ? 7 : TICK_CYCLES <= 256 ? 8 :
		TICK_CYCLES <= 512 ? 9 : TICK_CYCLES <= 1024 ? 10 :
		TICK_CYCLES <= 2048 ? 11 : TICK_CYCLES <= 4096 ? 12 :
		TICK_CYCLES <= 8192 ? 13 : TICK_CYCLES <= 16384 ? 14 :
		TICK_CYCLES <= 32768 ? 15 : TICK_CYCLES <= 65536 ? 16 :
		TICK_CYCLES <= 131072 ? 17 : TICK_CYCLES <= 262144 ? 18 :
		TICK_CYCLES <= 524288 ? 19 : TICK_CYCLES <= 1048576 ? 20 :
		TICK_CYCLES <= 2097152 ? 21 : 22;
	localparam integer AGE_WIDTH =
		C_STALE_MS <= 2 ? 1 : C_STALE_MS <= 4 ? 2 :
		C_STALE_MS <= 8 ? 3 : C_STALE_MS <= 16 ? 4 :
		C_STALE_MS <= 32 ? 5 : C_STALE_MS <= 64 ? 6 :
		C_STALE_MS <= 128 ? 7 : C_STALE_MS <= 256 ? 8 :
		C_STALE_MS <= 512 ? 9 : C_STALE_MS <= 1024 ? 10 :
		C_STALE_MS <= 2048 ? 11 : C_STALE_MS <= 4096 ? 12 :
		C_STALE_MS <= 8192 ? 13 : C_STALE_MS <= 16384 ? 14 :
		C_STALE_MS <= 32768 ? 15 : C_STALE_MS <= 65536 ? 16 :
		C_STALE_MS <= 131072 ? 17 : C_STALE_MS <= 262144 ? 18 :
		C_STALE_MS <= 524288 ? 19 : C_STALE_MS <= 1048576 ? 20 :
		C_STALE_MS <= 2097152 ? 21 : C_STALE_MS <= 4194304 ? 22 :
		C_STALE_MS <= 8388608 ? 23 : C_STALE_MS <= 16777216 ? 24 :
		C_STALE_MS <= 33554432 ? 25 : C_STALE_MS <= 67108864 ? 26 :
		C_STALE_MS <= 134217728 ? 27 : C_STALE_MS <= 268435456 ? 28 :
		C_STALE_MS <= 536870912 ? 29 : C_STALE_MS <= 1073741824 ? 30 : 31;
	localparam [TICK_WIDTH - 1:0] TICK_LAST = TICK_CYCLES - 1;
	localparam [AGE_WIDTH - 1:0] AGE_LAST = C_STALE_MS - 1;

	//------------模块实例化信号------------//
	wire mailbox_ready;
	wire mailbox_update;
	wire [63:0] mailbox_snapshot;

	//---------------计数信号---------------//
	reg [TICK_WIDTH - 1:0] cnt_tick;
	reg [AGE_WIDTH - 1:0] cnt_age_ms;

	//--------------寄存器信号--------------//
	reg [2:0] reg_rx_reset_sync;
	reg [2:0] reg_sys_reset_sync;
	reg [63:0] reg_pending_snapshot;
	reg flag_pending;

	//---------------其他信号---------------//
	wire common_rst_n;
	wire rx_rst_n;
	wire sys_rst_n;
	wire flag_send;
	wire flag_ms_tick;

	//---------------输出信号---------------//
	reg update_o;
	reg [63:0] snapshot_o;
	reg valid_o;
	reg stale_o;

	//-------------其他信号连线-------------//
	assign common_rst_n = i_rx_rst_n && i_sys_rst_n && i_rx_link_ready;
	assign rx_rst_n = reg_rx_reset_sync[2];
	assign sys_rst_n = reg_sys_reset_sync[2];
	assign flag_send = flag_pending && mailbox_ready;
	assign flag_ms_tick = cnt_tick == TICK_LAST;

	//-------------输出信号连线-------------//
	assign o_update = update_o;
	assign o_snapshot = snapshot_o;
	assign o_valid = valid_o;
	assign o_stale = stale_o;

	//-------------主要任务处理区域-------------//
	// 链路丢失/任一端复位异步撤销；两端分别同步释放，不依赖 PCIe。
	always@(posedge i_rx_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			reg_rx_reset_sync <= 3'd0;
		end else begin
			reg_rx_reset_sync <= {reg_rx_reset_sync[1:0], 1'b1};
		end
	end
	always@(posedge i_sys_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			reg_sys_reset_sync <= 3'd0;
		end else begin
			reg_sys_reset_sync <= {reg_sys_reset_sync[1:0], 1'b1};
		end
	end

	// 在途数据由 mailbox 保持，新事件只替换 pending；同拍新包优先保留。
	always@(posedge i_rx_clk or negedge rx_rst_n)begin
		if(!rx_rst_n)begin
			reg_pending_snapshot <= 64'd0;
			flag_pending <= 1'b0;
		end else begin
			if(flag_send)begin
				flag_pending <= 1'b0;
			end
			if(i_snapshot_valid)begin
				reg_pending_snapshot <= i_snapshot;
				flag_pending <= 1'b1;
			end
		end
	end

	// 默认只用 15-bit 分频加法；新快照重新锚定毫秒相位，超时不提前。
	always@(posedge i_sys_clk or negedge sys_rst_n)begin
		if(!sys_rst_n)begin
			cnt_tick <= {TICK_WIDTH{1'b0}};
		end else if(mailbox_update || !valid_o || flag_ms_tick)begin
			cnt_tick <= {TICK_WIDTH{1'b0}};
		end else begin
			cnt_tick <= cnt_tick + {{(TICK_WIDTH - 1){1'b0}}, 1'b1};
		end
	end

	// 默认 9-bit 毫秒计数，完整包事件优先于同拍超时。
	always@(posedge i_sys_clk or negedge sys_rst_n)begin
		if(!sys_rst_n)begin
			cnt_age_ms <= {AGE_WIDTH{1'b0}};
		end else if(mailbox_update)begin
			cnt_age_ms <= {AGE_WIDTH{1'b0}};
		end else if(valid_o && flag_ms_tick && cnt_age_ms != AGE_LAST)begin
			cnt_age_ms <= cnt_age_ms + {{(AGE_WIDTH - 1){1'b0}}, 1'b1};
		end
	end

	// 超时保留数据供诊断，但撤销有效性；复位/LOS 清数据和在途事件。
	always@(posedge i_sys_clk or negedge sys_rst_n)begin
		if(!sys_rst_n)begin
			update_o <= 1'b0;
			snapshot_o <= 64'd0;
			valid_o <= 1'b0;
			stale_o <= 1'b1;
		end else begin
			update_o <= 1'b0;
			if(mailbox_update)begin
				update_o <= 1'b1;
				snapshot_o <= mailbox_snapshot;
				valid_o <= 1'b1;
				stale_o <= 1'b0;
			end else if(valid_o && flag_ms_tick && cnt_age_ms == AGE_LAST)begin
				valid_o <= 1'b0;
				stale_o <= 1'b1;
			end
		end
	end

	//------------模块实例化区域------------//
	vehicle_behavior_action_cdc
	#(
		.C_DATA_WIDTH(64),
		.C_CHANGE_ONLY(0)
	)
	vehicle_behavior_action_cdc_Inst_snapshot
	(
		.i_src_clk(i_rx_clk),
		.i_src_rst_n(common_rst_n),
		.i_src_valid(flag_send),
		.i_src_payload(reg_pending_snapshot),
		.o_src_ready(mailbox_ready),
		.o_src_online(),
		.i_dst_clk(i_sys_clk),
		.i_dst_rst_n(common_rst_n),
		.i_dst_online(1'b1),
		.o_dst_valid(mailbox_update),
		.o_dst_payload(mailbox_snapshot)
	);
endmodule
