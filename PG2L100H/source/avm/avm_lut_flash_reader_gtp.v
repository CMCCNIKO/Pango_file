//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 LUT Flash 读取器接到本板 U15/SPI0，使用 GTP_CFGCLK 专用配置时钟资源将 SCK 送至 H13。
// Module Name: avm_lut_flash_reader_gtp
// File Name: source/avm/avm_lut_flash_reader_gtp.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 LUT Flash 读取器接到本板 U15/SPI0，使用 GTP_CFGCLK 专用配置时钟资源将 SCK 送至 H13。
// DQ0 为 MOSI，DQ1 为 MISO，其余 DQ 按只读 1-1-1 使用；CS/DQ 对应工程约束，C_FLASH_SELECT 在本包装中须为 0。
//
// Dependencies: avm_lut_flash_reader、GTP_CFGCLK、U15/SPI0 板级约束。
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
// Module Name: avm_lut_flash_reader_gtp
// Project Name: avm_fpga_handoff_20261011
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: AVM LUT reader to board U15/SPI0 user Flash.
//
// H13 is a dedicated configuration SPI clock and cannot be placed as GPIO.
// GTP_CFGCLK receives generated SCK and routes it to H13. U15 CS/DQ are
// ordinary top-level ports constrained to P18/N16/N17/R16/R17.
// C_FLASH_SELECT remains for source compatibility and must stay zero here.
// This wrapper uses 1-1-1 SPI: DQ0 is SI/MOSI and DQ1 is SO/MISO.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_lut_flash_reader_gtp #(
    parameter [27:0] C_LUT_BASE = 28'h0300000,
    parameter integer C_FLASH_SELECT = 0,
    parameter [23:0] C_FLASH_BASE_ADDR = 24'h800000,
    parameter [31:0] C_LUT_BYTES = 32'd4915200,
    // c5_ddr_clk is implementation dependent (100/200 MHz in existing
    // builds); 16 keeps SCK comfortably below 10 MHz in either case.
    parameter integer C_SPI_DIV = 16,
    parameter integer C_START_DELAY = 32,
    parameter integer C_TIMEOUT_CYCLES = 32'd2000000000
)(
    input  wire         i_clk,
    input  wire         i_rst_n,
    input  wire         i_enable,
    output wire         o_lut_ready,
    output wire         o_busy,
    output wire         o_error,
    output wire         o_jedec_ok,
    output wire [31:0]  o_bytes_read,
    output wire [27:0]  o_axi_awaddr,
    output wire [3:0]   o_axi_awlen,
    output wire         o_axi_awvalid,
    input  wire         i_axi_awready,
    output wire [255:0] o_axi_wdata,
    output wire [31:0]  o_axi_wstrb,
    output wire         o_axi_wvalid,
    input  wire         i_axi_wready,
    output wire         o_flash_cs_n,
    inout  wire [3:0]   io_flash_dq
);
    wire spi_clk_i;
    wire spi_cs0_n_i;
    wire spi_cs1_n_i;
    wire spi_d0_o;
    wire spi_d1_i;
    wire spi_cs_n_i;

    // Physical build is U15/SPI0. A non-zero selection is intentionally
    // disabled rather than accidentally routing U9 pins to the U15 port.
    assign spi_cs_n_i   = (C_FLASH_SELECT == 0) ? spi_cs0_n_i : 1'b1;
    assign o_flash_cs_n = spi_cs_n_i;

    // DQ0 (SI) is driven only while CS is low; DQ1 (SO) is sampled from the
    // Flash. DQ2/DQ3 are released because the reader is single-bit mode.
    assign io_flash_dq = !spi_cs_n_i ? {3'bzzz, spi_d0_o} : 4'bzzzz;
    assign spi_d1_i    = io_flash_dq[1];

    // GTP_CFGCLK has no RTL output; its dedicated route drives H13.
    // CE_N is low exactly when the user Flash transaction is selected.
    GTP_CFGCLK u_gtp_cfgclk(
        .CLKIN (spi_clk_i),
        .CE_N  (spi_cs_n_i)
    );

    avm_lut_flash_reader #(
        .C_LUT_BASE(C_LUT_BASE),
        .C_FLASH_SELECT(0),
        .C_FLASH_BASE_ADDR(C_FLASH_BASE_ADDR),
        .C_LUT_BYTES(C_LUT_BYTES),
        .C_SPI_DIV(C_SPI_DIV),
        .C_START_DELAY(C_START_DELAY),
        .C_TIMEOUT_CYCLES(C_TIMEOUT_CYCLES)
    ) u_reader(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_enable(i_enable),
        .io_spi0_d0(spi_d0_o),
        .io_spi0_d1(spi_d1_i),
        .io_spi0_d2(),
        .io_spi0_d3(),
        .io_spi1_d0(),
        .io_spi1_d1(),
        .io_spi1_d2(),
        .io_spi1_d3(),
        .o_spi_clk(spi_clk_i),
        .o_spi0_cs_n(spi_cs0_n_i),
        .o_spi1_cs_n(spi_cs1_n_i),
        .o_lut_ready(o_lut_ready),
        .o_busy(o_busy),
        .o_error(o_error),
        .o_jedec_ok(o_jedec_ok),
        .o_bytes_read(o_bytes_read),
        .o_axi_awaddr(o_axi_awaddr),
        .o_axi_awlen(o_axi_awlen),
        .o_axi_awvalid(o_axi_awvalid),
        .i_axi_awready(i_axi_awready),
        .o_axi_wdata(o_axi_wdata),
        .o_axi_wstrb(o_axi_wstrb),
        .o_axi_wvalid(o_axi_wvalid),
        .i_axi_wready(i_axi_wready)
    );

    // spi_cs1_n_i is deliberately left internal; U9/SPI1 is not selected.
endmodule
