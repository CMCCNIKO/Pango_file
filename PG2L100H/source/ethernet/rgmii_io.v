//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 面向本板 PDS 2025.2 的 RGMII 双沿 I/O 适配层，RXC 经专用全局时钟供 GMII/IDDR，数据与控制双沿采样组成 8-bit 字节及 RX_ER。
// Module Name: rgmii_io
// File Name: source/ethernet/rgmii_io.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 面向本板 PDS 2025.2 的 RGMII 双沿 I/O 适配层，RXC 经专用全局时钟供 GMII/IDDR，数据与控制双沿采样组成 8-bit 字节及 RX_ER。
// RX 数据/控制保持原双段 230 tap 延时；TX 字节由 ODDR 输出，TXC 直接 ODDR 转发，所需约 2ns TX 延迟由 RTL8211F 的 MDIO 配置提供。
//
// Dependencies: GTP_INBUF、GTP_CLKBUFG、GTP_IODELAY_E2、GTP_IDDR_E1、GTP_ODDR_E1、GTP_OUTBUF；mdio_link_status 配置 PHY TX 内部延迟。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 参考同板官方 2_11_ethernet_test；原旧输出单段 200 tap 的说明不可沿用，Logos2 输出延时链仅支持 0..127 tap。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// PG2L100H RGMII 双沿 I/O 适配层
//
// 依据同板官方 2_11_ethernet_test 例程适配 PDS 2025.2。
// 2026-11-01：TXC 直接由 ODDR 转发，RTL8211F 通过 MDIO 配置内部
// 2 ns TX 延时。Logos2 输出延时链仅支持 0..127 tap，旧单段 200 tap
// 接法无效，不能用输入双链的名义公式替代。接收数据和控制保留
// 原双段 230 tap，由 RXC 域 IDDR 采样；RX PHY 配置不在本层改动。
//////////////////////////////////////////////////////////////////////////////////
module rgmii_io #(
	parameter [7:0] C_RX_DELAY_TAPS  = 8'd230
)(
	input i_rst_n,
	input i_rgmii_rxc,
	input i_rgmii_rx_ctl,
	input [3:0] i_rgmii_rxd,
	output o_rgmii_clk,
	output reg o_gmii_rx_dv,
	output reg [7:0] o_gmii_rxd,
	output reg o_gmii_rx_error,
	input i_gmii_tx_en,
	input [7:0] i_gmii_txd,
	output o_rgmii_txc,
	output o_rgmii_tx_ctl,
	output [3:0] o_rgmii_txd
);
	//------------RGMII 接收时钟与延时控制-------------//
	wire rgmii_rx_clk;
	wire [7:0] rx_delay_gray;
	assign rx_delay_gray = (C_RX_DELAY_TAPS >> 1) ^ C_RX_DELAY_TAPS;
	assign o_rgmii_clk = rgmii_rx_clk;

	// 将 PHY RXC 送入专用全局时钟路径，供 GMII 字节域与 IDDR 共用。
	GTP_CLKBUFG u_rgmii_rxc_bufg(
		.CLKIN(i_rgmii_rxc),
		.CLKOUT(rgmii_rx_clk)
	);

	//----------------RGMII 接收通道----------------//
	wire rx_ctl_ibuf;
	wire rx_ctl_delayed;
	wire [3:0] rx_data_ibuf;
	wire [3:0] rx_data_delayed;
	wire rx_ctl_rise;
	wire rx_ctl_fall;
	wire [3:0] rx_data_rise;
	wire [3:0] rx_data_fall;

	GTP_INBUF #(
		.IOSTANDARD("LVCMOS33"),
		.TERM_DDR()
	) u_rgmii_rx_ctl_ibuf(
		.O(rx_ctl_ibuf),
		.I(i_rgmii_rx_ctl)
	);

	GTP_IODELAY_E2 #(
		.DELAY_STEP_VALUE(C_RX_DELAY_TAPS),
		.DELAY_STEP_SEL("PORT"),
		.TDELAY_EN("FALSE")
	) u_rgmii_rx_ctl_delay(
		.DELAY_STEP(rx_delay_gray),
		.DO(rx_ctl_delayed),
		.DELAY_SEL(1'b1),
		.DI(rx_ctl_ibuf),
		.EN_N(1'b0)
	);

	GTP_IDDR_E1 #(
		.GRS_EN("TRUE"),
		.IDDR_MODE("SAME_PIPELINED"),
		.RS_TYPE("ASYNC_RESET")
	) u_rgmii_rx_ctl_iddr(
		.Q0(rx_ctl_rise),
		.Q1(rx_ctl_fall),
		.CE(1'b1),
		.CLK(rgmii_rx_clk),
		.D(rx_ctl_delayed),
		.RS(!i_rst_n)
	);

	genvar rx_bit;
	generate
		for (rx_bit = 0; rx_bit < 4; rx_bit = rx_bit + 1) begin: gen_rgmii_rx_data
			GTP_INBUF #(
				.IOSTANDARD("LVCMOS33"),
				.TERM_DDR()
			) u_rgmii_rxd_ibuf(
				.O(rx_data_ibuf[rx_bit]),
				.I(i_rgmii_rxd[rx_bit])
			);

			GTP_IODELAY_E2 #(
				.DELAY_STEP_VALUE(C_RX_DELAY_TAPS),
				.DELAY_STEP_SEL("PORT"),
				.TDELAY_EN("FALSE")
			) u_rgmii_rxd_delay(
				.DELAY_STEP(rx_delay_gray),
				.DO(rx_data_delayed[rx_bit]),
				.DELAY_SEL(1'b1),
				.DI(rx_data_ibuf[rx_bit]),
				.EN_N(1'b0)
			);

			GTP_IDDR_E1 #(
				.GRS_EN("TRUE"),
				.IDDR_MODE("SAME_PIPELINED"),
				.RS_TYPE("ASYNC_RESET")
			) u_rgmii_rxd_iddr(
				.Q0(rx_data_rise[rx_bit]),
				.Q1(rx_data_fall[rx_bit]),
				.CE(1'b1),
				.CLK(rgmii_rx_clk),
				.D(rx_data_delayed[rx_bit]),
				.RS(!i_rst_n)
			);
		end
	endgenerate

	always @(posedge rgmii_rx_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			o_gmii_rx_dv <= 1'b0;
			o_gmii_rxd <= 8'd0;
			o_gmii_rx_error <= 1'b0;
		end else begin
			o_gmii_rx_dv <= rx_ctl_rise;
			o_gmii_rxd <= {rx_data_fall, rx_data_rise};
			o_gmii_rx_error <= rx_ctl_rise ^ rx_ctl_fall;
		end
	end

	//----------------RGMII 发送通道----------------//
	wire [3:0] tx_data_oddr;
	wire tx_ctl_oddr;
	wire tx_clk_oddr;

	genvar tx_bit;
	generate
		for (tx_bit = 0; tx_bit < 4; tx_bit = tx_bit + 1) begin: gen_rgmii_tx_data
			GTP_ODDR_E1 #(
				.GRS_EN("TRUE"),
				.ODDR_MODE("SAME_EDGE"),
				.RS_TYPE("ASYNC_RESET")
			) u_rgmii_txd_oddr(
				.Q(tx_data_oddr[tx_bit]),
				.CE(1'b1),
				.CLK(rgmii_rx_clk),
				.D0(i_gmii_txd[tx_bit]),
				.D1(i_gmii_txd[tx_bit + 4]),
				.RS(!i_rst_n)
			);

			GTP_OUTBUF #(
				.IOSTANDARD("LVCMOS33"),
				.SLEW_RATE("FAST"),
				.DRIVE_STRENGTH("4")
			) u_rgmii_txd_obuf(
				.I(tx_data_oddr[tx_bit]),
				.O(o_rgmii_txd[tx_bit])
			);
		end
	endgenerate

	GTP_ODDR_E1 #(
		.GRS_EN("TRUE"),
		.ODDR_MODE("SAME_EDGE"),
		.RS_TYPE("ASYNC_RESET")
	) u_rgmii_tx_ctl_oddr(
		.Q(tx_ctl_oddr),
		.CE(1'b1),
		.CLK(rgmii_rx_clk),
		.D0(i_gmii_tx_en),
		.D1(i_gmii_tx_en),
		.RS(!i_rst_n)
	);

	GTP_OUTBUF #(
		.IOSTANDARD("LVCMOS33"),
		.SLEW_RATE("FAST"),
		.DRIVE_STRENGTH("4")
	) u_rgmii_tx_ctl_obuf(
		.I(tx_ctl_oddr),
		.O(o_rgmii_tx_ctl)
	);

	GTP_ODDR_E1 #(
		.GRS_EN("TRUE"),
		.ODDR_MODE("SAME_EDGE"),
		.RS_TYPE("ASYNC_RESET")
	) u_rgmii_txc_oddr(
		.Q(tx_clk_oddr),
		.CE(1'b1),
		.CLK(rgmii_rx_clk),
		.D0(1'b1),
		.D1(1'b0),
		.RS(1'b0)
	);

	GTP_OUTBUF #(
		.IOSTANDARD("LVCMOS33"),
		.SLEW_RATE("FAST"),
		.DRIVE_STRENGTH("4")
	) u_rgmii_txc_obuf(
		.I(tx_clk_oddr),
		.O(o_rgmii_txc)
	);
endmodule
