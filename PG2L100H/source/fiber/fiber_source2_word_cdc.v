//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026/09/17
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 消费 DDR 域授予 source2 的稳定 grant token，在恢复时钟域接收光纤 256-bit 帧字并经官方异步 FIFO 跨到 DDR。
// Module Name: fiber_source2_word_cdc
// File Name: source/fiber/fiber_source2_word_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 消费 DDR 域授予 source2 的稳定 grant token，在恢复时钟域接收光纤 256-bit 帧字并经官方异步 FIFO 跨到 DDR。
// 一次授权仅接收一帧，完整提交/错误事件跨域；任一协议错误或 FIFO 不可写时使整帧作废并清空 FIFO。
//
// Dependencies: c6_net_to_ddr_fifo_wrapper、c6_event_sync、c6_net_to_ddr_fifo 官方 IP。
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
// Create Date: 2026/09/17
// Design Name: Fiber Source2 Word CDC
// Module Name: fiber_source2_word_cdc
// Project Name: Ethernet_HDMI_OV5640_SD_HDMI_OUT
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 消费 source2 grant token，将光纤 256-bit 帧字流经官方异步 FIFO 跨入 DDR 域。
//
// Dependencies: c6_net_to_ddr_fifo_wrapper、c6_event_sync、c6_net_to_ddr_fifo IP
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments: 任一协议错误或 FIFO 不可写均立即作废整帧并清空 FIFO。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module fiber_source2_word_cdc
(
	//--------------HSST RX 时钟域-----------//
	input          i_rx_clk,                      // HSST RX fabric 时钟
	input          i_rx_rst_n,                    // RX 域低有效异步复位
	input          i_frame_accept,                // RX 域允许接收新 source2 帧
	input          i_grant_toggle,                // DDR 域每次授权翻转一次的 token
	input          i_frame_start,                 // assembler 新帧开始单拍事件
	input          i_frame_end,                   // assembler 完整帧结束单拍事件
	input          i_frame_error,                 // assembler 错误帧单拍事件
	input          i_word_valid,                  // assembler 256-bit word 有效
	input  [255:0] i_word_data,                   // 当前 16 个 RGB565 像素

	//---------------DDR 时钟域--------------//
	input          i_ddr_clk,                     // DDR 用户时钟
	input          i_ddr_rst_n,                   // DDR 域低有效异步复位
	input          i_word_pop,                    // DDR writer 消费当前 show-ahead word
	output [255:0] o_word_data,                   // DDR 域当前 show-ahead word
	output         o_word_empty,                  // DDR 域无可消费 word
	output [15:0]  o_wr_water_level,              // RX 写域 FIFO 水位
	output [15:0]  o_rd_water_level,              // DDR 读域 FIFO 水位

	//--------------DDR 帧事件---------------//
	output         o_frame_prepare_ddr,           // 已授权新帧开始，DDR 域单拍事件
	output         o_frame_complete_ddr,          // 完整帧结束，DDR 域单拍事件
	output         o_frame_abort_ddr              // 当前帧作废，DDR 域单拍事件
);

	//--------------寄存器信号--------------//
	reg capture_active;                           // 当前 grant 已消费且允许写入本帧
	reg prepare_rx;                               // RX 域 prepare 单拍事件
	reg complete_rx;                              // RX 域 complete 单拍事件
	reg abort_rx;                                 // RX 域 abort 单拍事件
	reg error_latched;                            // 当前 capture 已观察到不可提交错误
	reg grant_sync1 /* synthesis syn_preserve = 1 */; // grant toggle 第一级同步
	reg grant_sync2 /* synthesis syn_preserve = 1 */; // grant toggle 第二级同步
	reg grant_seen;                               // 最近一次已消费的 grant toggle

	//---------------其他信号---------------//
	wire grant_pending;                           // RX 域存在一个尚未消费的 grant token
	wire fifo_word_ready;                         // 官方 FIFO 当前允许写入
	wire fifo_word_valid;                         // 当前 word 属于活动 capture

	//-------------其他信号连线-------------//
	assign grant_pending = grant_sync2 != grant_seen;
	assign fifo_word_valid = i_word_valid && capture_active;

	// DDR 域保持 toggle 至下一次授权；RX 域同步后只在接受 frame_start 时消费一次。
	always@(posedge i_rx_clk or negedge i_rx_rst_n)begin
		if(i_rx_rst_n == 1'b0)begin
			grant_sync1 <= 1'b0;
			grant_sync2 <= 1'b0;
			grant_seen <= 1'b0;
		end else begin
			grant_sync1 <= i_grant_toggle;
			grant_sync2 <= grant_sync1;
			if((i_frame_start == 1'b1) && (capture_active == 1'b0) &&
				(i_frame_accept == 1'b1) && (grant_pending == 1'b1))begin
				grant_seen <= grant_sync2;
			end
		end
	end

	// 管理一次已授权帧；错误和 FIFO backpressure 均优先于正常 frame_end。
	always@(posedge i_rx_clk or negedge i_rx_rst_n)begin
		if(i_rx_rst_n == 1'b0)begin
			capture_active <= 1'b0;
			prepare_rx <= 1'b0;
			complete_rx <= 1'b0;
			abort_rx <= 1'b0;
			error_latched <= 1'b0;
		end else begin
			prepare_rx <= 1'b0;
			complete_rx <= 1'b0;
			abort_rx <= 1'b0;
			if(i_frame_start == 1'b1)begin
				if(capture_active == 1'b1)begin
					capture_active <= 1'b0;
					error_latched <= 1'b1;
					abort_rx <= 1'b1;
				end else if((i_frame_accept == 1'b1) && (grant_pending == 1'b1))begin
					capture_active <= 1'b1;
					error_latched <= 1'b0;
					prepare_rx <= 1'b1;
				end
			end else if((capture_active == 1'b1) && (i_frame_error == 1'b1))begin
				capture_active <= 1'b0;
				error_latched <= 1'b1;
				abort_rx <= 1'b1;
			end else if((capture_active == 1'b1) && (i_word_valid == 1'b1) &&
				(fifo_word_ready == 1'b0))begin
				capture_active <= 1'b0;
				error_latched <= 1'b1;
				abort_rx <= 1'b1;
			end else if((capture_active == 1'b1) && (i_frame_end == 1'b1))begin
				capture_active <= 1'b0;
				if((error_latched == 1'b1) || (i_frame_error == 1'b1))begin
					error_latched <= 1'b1;
					abort_rx <= 1'b1;
				end else begin
					error_latched <= 1'b0;
					complete_rx <= 1'b1;
				end
			end
		end
	end

	// 复用当前 PG2L100H 工程已生成并登记的 512x256 官方异步 FIFO wrapper。
	c6_net_to_ddr_fifo_wrapper u_source2_fifo(
		.i_net_clk(i_rx_clk),
		.i_net_rst_n(i_rx_rst_n),
		.i_frame_prepare(prepare_rx),
		.i_frame_abort(abort_rx),
		.i_word_valid(fifo_word_valid),
		.i_word_data(i_word_data),
		.o_word_ready(fifo_word_ready),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rst_n),
		.i_word_pop(i_word_pop),
		.o_word_data(o_word_data),
		.o_word_empty(o_word_empty),
		.o_wr_water_level(o_wr_water_level),
		.o_rd_water_level(o_rd_water_level)
	);

	// 帧事件间隔远大于两级同步窗口，复用现有 toggle CDC 产生 DDR 单拍事件。
	c6_event_sync u_prepare_sync(
		.i_src_clk(i_rx_clk),
		.i_src_rst_n(i_rx_rst_n),
		.i_src_event(prepare_rx),
		.i_dst_clk(i_ddr_clk),
		.i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_prepare_ddr)
	);

	c6_event_sync u_complete_sync(
		.i_src_clk(i_rx_clk),
		.i_src_rst_n(i_rx_rst_n),
		.i_src_event(complete_rx),
		.i_dst_clk(i_ddr_clk),
		.i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_complete_ddr)
	);

	c6_event_sync u_abort_sync(
		.i_src_clk(i_rx_clk),
		.i_src_rst_n(i_rx_rst_n),
		.i_src_event(abort_rx),
		.i_dst_clk(i_ddr_clk),
		.i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_abort_ddr)
	);

endmodule
