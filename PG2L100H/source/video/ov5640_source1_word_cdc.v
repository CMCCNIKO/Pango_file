//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将摄像头原始 DVP 输入转换为 RGB565 像素，经过 2 倍抽取及裁剪形成 640×320 tile，再打包跨入 DDR 域。
// Module Name: ov5640_source1_word_cdc
// File Name: source/video/ov5640_source1_word_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将摄像头原始 DVP 输入转换为 RGB565 像素，经过 2 倍抽取及裁剪形成 640×320 tile，再打包跨入 DDR 域。
// 复用通用 HDMI tile 打包/FIFO 桥，传递帧授权、完成和错误；同一模块可供多路摄像头独立实例化。
//
// Dependencies: ov5640_dvp_unpack、ov5640_tile_normalizer、hdmi_source3_word_cdc。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 原始 Engineer: Codex；保留原始历史署名记录。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: Codex
// Create Date: 2026-09-14
// Design Name: Quad Video Mosaic
// Module Name: ov5640_source1_word_cdc
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: CMOS1 原始 DVP PCLK 域到 DDR 域的 source1 适配器。
//              先做 RGB565 byte-pair、2x 抽取/crop，再复用官方 FIFO。
//              CMOS2 不复制逻辑，后续直接实例化本模块即可。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module ov5640_source1_word_cdc #(
  parameter integer C_FRAME_PIXELS = 204800,
  parameter C_FRAME_ACCEPT = 1'b1,
  parameter integer C_SOURCE_WIDTH = 1280,
  parameter integer C_SOURCE_HEIGHT = 720,
  parameter integer C_CROP_TOP = 20,
  parameter integer C_TILE_WIDTH = 640,
  parameter integer C_TILE_HEIGHT = 320,
  parameter C_USE_GRANT_TOKEN = 1'b0
)(
  input i_cmos_pclk,
  input i_cmos_rst_n,
  input i_cmos_vsync,
  input i_cmos_href,
  input [7:0] i_cmos_data,
  input i_ddr_clk,
  input i_ddr_rst_n,
  input i_grant_toggle,
  input i_word_pop,
  output [255:0] o_word_data,
  output o_word_empty,
  output [15:0] o_wr_water_level,
  output [15:0] o_rd_water_level,
  output o_frame_prepare_ddr,
  output o_frame_complete_ddr,
  output o_frame_abort_ddr
);
  wire unpack_pixel_valid;
  wire [15:0] unpack_pixel_rgb565;
  wire unpack_frame_start;
  wire unpack_frame_end;
  wire unpack_line_start;
  wire unpack_line_end;
  wire tile_pixel_valid;
  wire [15:0] tile_pixel_rgb565;
  wire tile_frame_start;
  wire tile_frame_end;
  wire tile_frame_error;

  ov5640_dvp_unpack u_unpack(
    .i_pclk(i_cmos_pclk), .i_rst_n(i_cmos_rst_n),
    .i_vsync(i_cmos_vsync), .i_href(i_cmos_href), .i_data(i_cmos_data),
    .o_pixel_valid(unpack_pixel_valid), .o_pixel_rgb565(unpack_pixel_rgb565),
    .o_frame_start(unpack_frame_start), .o_frame_end(unpack_frame_end),
    .o_line_start(unpack_line_start), .o_line_end(unpack_line_end)
  );

  ov5640_tile_normalizer #(
    .C_SOURCE_WIDTH(C_SOURCE_WIDTH), .C_SOURCE_HEIGHT(C_SOURCE_HEIGHT),
    .C_CROP_TOP(C_CROP_TOP), .C_TILE_WIDTH(C_TILE_WIDTH),
    .C_TILE_HEIGHT(C_TILE_HEIGHT)
  ) u_normalizer(
    .i_pclk(i_cmos_pclk), .i_rst_n(i_cmos_rst_n),
    .i_frame_start(unpack_frame_start), .i_frame_end(unpack_frame_end),
    .i_line_start(unpack_line_start), .i_line_end(unpack_line_end),
    .i_pixel_valid(unpack_pixel_valid), .i_pixel_rgb565(unpack_pixel_rgb565),
    .o_pixel_valid(tile_pixel_valid), .o_pixel_rgb565(tile_pixel_rgb565),
    .o_frame_start(tile_frame_start), .o_frame_end(tile_frame_end),
    .o_frame_error(tile_frame_error)
  );

  // The source3 adapter is a generic normalized RGB565 stream adapter;
  // reusing it keeps the official FIFO and frame event semantics identical.
  hdmi_source3_word_cdc #(.C_FRAME_PIXELS(C_FRAME_PIXELS), .C_USE_GRANT_TOKEN(C_USE_GRANT_TOKEN)) u_word_cdc(
    .i_rx_clk(i_cmos_pclk), .i_rx_rst_n(i_cmos_rst_n),
    .i_frame_accept(C_FRAME_ACCEPT), .i_frame_start(tile_frame_start),
    .i_grant_toggle(i_grant_toggle),
    .i_frame_end(tile_frame_end), .i_frame_error(tile_frame_error),
    .i_pixel_valid(tile_pixel_valid), .i_pixel_rgb565(tile_pixel_rgb565),
    .i_ddr_clk(i_ddr_clk), .i_ddr_rst_n(i_ddr_rst_n),
    .i_word_pop(i_word_pop), .o_word_data(o_word_data),
    .o_word_empty(o_word_empty), .o_wr_water_level(o_wr_water_level),
    .o_rd_water_level(o_rd_water_level),
    .o_frame_prepare_ddr(o_frame_prepare_ddr),
    .o_frame_complete_ddr(o_frame_complete_ddr),
    .o_frame_abort_ddr(o_frame_abort_ddr)
  );
endmodule
