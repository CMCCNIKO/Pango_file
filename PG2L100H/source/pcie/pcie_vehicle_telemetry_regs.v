//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-28
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 对同 PCIe 时钟域的车道与用户行为 192 位状态采集一致性只读快照，公开字段变化时同步更新 generation。
// Module Name: pcie_vehicle_telemetry_regs
// File Name: source/pcie/pcie_vehicle_telemetry_regs.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对同 PCIe 时钟域的车道与用户行为 192 位状态采集一致性只读快照，公开字段变化时同步更新 generation。
// 软件按 generation/数据/generation 两次读数一致后采用；仅保留最新状态，无反压、写副作用或无损事件队列。
//
// Dependencies: vehicle_behavior_decision 同域快照及 BAR1 读分流。
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
// Create Date: 2026-10-28
// Design Name: PCIe lane and user behavior telemetry
// Module Name: pcie_vehicle_telemetry_regs
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 同PCIe时钟域的192bit一致性快照；仅只读，无反压及写副作用。
//              任意公开快照字段变化时generation与快照同沿更新。
//              软件读generation/数据/generation，相同才采用；复位后重新采样。
// Dependencies: vehicle_behavior_decision的同拍组合快照；既有BAR1读桥。
// Revision: 0.01 - 2026-10-28 File Created
// Additional Comments: 只提供最新状态，不是无损事件队列；32bit计数自然回绕。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_vehicle_telemetry_regs
#(
	parameter [31:0] C_ACTION_HOLD_MS = 32'd1001,
	parameter [31:0] C_HSST_STALE_MS = 32'd500
)
(
	input i_clk,
	input i_rst_n,
	input [191:0] i_snapshot,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window_data
);
	//--------------寄存器信号--------------//
	reg [191:0] reg_snapshot;
	reg [31:0] reg_generation;

	//---------------标志信号---------------//
	wire flag_snapshot_change;

	//---------------输出信号---------------//
	reg [127:0] rd_window_data_o;

	//-------------其他信号连线-------------//
	assign flag_snapshot_change = i_snapshot != reg_snapshot;

	//-------------输出信号连线-------------//
	assign o_rd_window_data = rd_window_data_o;

	//-----------输出信号处理区域-----------//
	// 无额外读流水，延迟由既有PIO桥保持；每个窗口低字在低32bit。
	always@(*)begin
		rd_window_data_o = 128'd0;
		case(i_rd_addr[11:4])
			8'h69:begin
				rd_window_data_o = {32'h0000_0007, reg_generation, 32'h0001_0000, 32'h3154_564c};
			end
			8'h6a:begin
				rd_window_data_o = reg_snapshot[127:0];
			end
			8'h6b:begin
				rd_window_data_o = {C_HSST_STALE_MS, C_ACTION_HOLD_MS, reg_snapshot[191:128]};
			end
			default:begin
				rd_window_data_o = 128'd0;
			end
		endcase
	end

	//-----------主要任务处理区域-----------//
	// 每拍观察，不向动作或HSST链路返回ready；generation仅随真实公开状态变化。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_snapshot <= 192'd0;
		end else begin
			reg_snapshot <= i_snapshot;
		end
	end

	// 四个8bit加法器并行计算进位，避免新增32bit串行递增路径。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_generation <= 32'd0;
		end else if(flag_snapshot_change)begin
			reg_generation[7:0] <= reg_generation[7:0] + 8'd1;
			reg_generation[15:8] <= reg_generation[15:8] + {7'd0, (&reg_generation[7:0])};
			reg_generation[23:16] <= reg_generation[23:16] + {7'd0, (&reg_generation[15:0])};
			reg_generation[31:24] <= reg_generation[31:24] + {7'd0, (&reg_generation[23:0])};
		end
	end
endmodule
