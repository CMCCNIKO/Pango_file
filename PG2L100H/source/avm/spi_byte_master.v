//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: SPI Mode-0、MSB 优先的单字节收发引擎，start 时锁存分频，生成完整 8 位 SCK/MOSI 序列并在上升沿采样 MISO。
// Module Name: spi_byte_master
// File Name: source/avm/spi_byte_master.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// SPI Mode-0、MSB 优先的单字节收发引擎，start 时锁存分频，生成完整 8 位 SCK/MOSI 序列并在上升沿采样 MISO。
// 最后下降沿完成字节锁存并报告 done，采用同步低有效复位；调用方保持片选及字节间协议状态。
//
// Dependencies: 无 IP 依赖；纯分频与移位寄存器。
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
// Create Date: 2026-10-14
// Design Name: PG2L100H SD SPI
// Module Name: spi_byte_master
// Project Name: pg2l100h_sd_lut
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Mode-0 MSB-first byte engine, synchronous active-low reset.
// Dependencies: none
// Revision: 0.02 - explicit byte handshake and latched divider
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module spi_byte_master
#(
	parameter C_DIV = 54                        // 默认 SPI 半周期分频值
)
(
	input i_clk,                                // 单一系统时钟
	input i_rstn,                               // 低有效同步复位
	input i_start,                              // 仅空闲时接受的一拍启动请求
	input [15:0] i_div,                         // SPI 半周期系统时钟数，0 选择默认参数
	input [7:0] i_tx,                           // 待发送字节，MSB 优先
	output [7:0] o_rx,                          // 字节完成后锁存的接收结果
	output o_busy,                              // 字节传输正在进行
	output o_done,                              // 字节完成单拍脉冲
	output o_sclk,                              // Mode-0 SPI 时钟输出
	output o_mosi,                              // SPI 主机串行输出
	input i_miso                                // SPI 从机串行输入，在上升沿采样
);
	//--------------寄存器信号--------------//
	// 发送移位寄存器与上升沿采样的接收移位寄存器。
	reg [7:0] reg_tx;
	reg [7:0] reg_rx;

	//---------------计数信号---------------//
	reg [2:0] cnt_bit;
	reg [15:0] cnt_div;
	reg [15:0] reg_div;

	//---------------输出信号---------------//
	reg [7:0] rx_o;
	reg busy_o;
	reg done_o;
	reg sclk_o;
	reg mosi_o;

	//-------------输出信号连线-------------//
	assign o_rx = rx_o;
	assign o_busy = busy_o;
	assign o_done = done_o;
	assign o_sclk = sclk_o;
	assign o_mosi = mosi_o;

	//-----------主要任务处理区域-----------//
	// 分频值在字节开始时锁存；0 选择 C_DIV，合法有效值为 1..65535。
	always@(posedge i_clk)begin
		if(!i_rstn)begin
			reg_tx <= 8'hff;
			reg_rx <= 8'd0;
			cnt_bit <= 3'd0;
			cnt_div <= 16'd0;
			reg_div <= C_DIV;
			rx_o <= 8'd0;
			busy_o <= 1'b0;
			done_o <= 1'b0;
			sclk_o <= 1'b0;
			mosi_o <= 1'b1;
		end else begin
			done_o <= 1'b0;
			if(!busy_o)begin
				sclk_o <= 1'b0;
				mosi_o <= 1'b1;
				if(i_start)begin
					reg_div <= (i_div == 16'd0) ? C_DIV : i_div;
					reg_tx <= i_tx;
					reg_rx <= 8'd0;
					cnt_div <= 16'd0;
					cnt_bit <= 3'd0;
					mosi_o <= i_tx[7];
					busy_o <= 1'b1;
				end
			end else if(cnt_div == reg_div - 16'd1)begin
				cnt_div <= 16'd0;
				if(!sclk_o)begin
					sclk_o <= 1'b1;
					reg_rx <= {reg_rx[6:0], i_miso};
				end else begin
					sclk_o <= 1'b0;
					if(cnt_bit == 3'd7)begin
						// 最后一个上升沿已完成采样，下降沿直接锁存完整字节。
						rx_o <= reg_rx;
						busy_o <= 1'b0;
						done_o <= 1'b1;
						mosi_o <= 1'b1;
					end else begin
						cnt_bit <= cnt_bit + 3'd1;
						reg_tx <= {reg_tx[6:0], 1'b1};
						mosi_o <= reg_tx[6];
					end
				end
			end else begin
				cnt_div <= cnt_div + 16'd1;
			end
		end
	end
endmodule
