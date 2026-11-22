//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 定义四路 640x320 RGB565 视频的 DDR 帧槽统一地址：每帧 204800 像素、12800 个 256-bit word、每 word 地址步长 8、每源三槽且槽步长 0x40000。
// Module Name: (include) quad_frame_address_map.vh
// File Name: source/memory/quad_frame_address_map.vh
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 定义四路 640x320 RGB565 视频的 DDR 帧槽统一地址：每帧 204800 像素、12800 个 256-bit word、每 word 地址步长 8、每源三槽且槽步长 0x40000。
// 当前源映射为 source0=Ethernet/PGV1、source1=OV5640 CMOS1、source2=HSST 光纤、source3=HDMI-IN；旧正文 source2 的 CMOS2/reserved 注释为历史用途。
//
// Dependencies: 被四源写入、读取、帧槽模块 include 的地址宏；不定义 module。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`ifndef QUAD_FRAME_ADDRESS_MAP_VH
`define QUAD_FRAME_ADDRESS_MAP_VH
// 四路 640x320 RGB565 帧槽统一地址定义（HMIC 地址单位）
`define QUAD_FRAME_WIDTH       640
`define QUAD_FRAME_HEIGHT      320
`define QUAD_FRAME_PIXELS      204800
`define QUAD_FRAME_WORDS       12800
`define QUAD_WORD_STRIDE       32'h00000008
`define QUAD_FRAME_SPAN        32'h00019000
`define QUAD_SLOT_STRIDE       32'h00040000
`define QUAD_SOURCE_COUNT      4
`define QUAD_SLOT_COUNT        3

// source 0: Ethernet / PGV1
`define QUAD_SRC0_BASE          32'h00000000
`define QUAD_SRC0_SLOT0_BASE    32'h00000000
`define QUAD_SRC0_SLOT1_BASE    32'h00040000
`define QUAD_SRC0_SLOT2_BASE    32'h00080000
// source 1: OV5640 #1
`define QUAD_SRC1_BASE          32'h000C0000
`define QUAD_SRC1_SLOT0_BASE    32'h000C0000
`define QUAD_SRC1_SLOT1_BASE    32'h00100000
`define QUAD_SRC1_SLOT2_BASE    32'h00140000
// source 2: OV5640 #2 / reserved
`define QUAD_SRC2_BASE          32'h00180000
`define QUAD_SRC2_SLOT0_BASE    32'h00180000
`define QUAD_SRC2_SLOT1_BASE    32'h001C0000
`define QUAD_SRC2_SLOT2_BASE    32'h00200000
// source 3: HDMI-IN / MS7200
`define QUAD_SRC3_BASE          32'h00240000
`define QUAD_SRC3_SLOT0_BASE    32'h00240000
`define QUAD_SRC3_SLOT1_BASE    32'h00280000
`define QUAD_SRC3_SLOT2_BASE    32'h002C0000
`endif