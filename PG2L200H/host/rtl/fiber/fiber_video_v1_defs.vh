//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 集中定义 HSST-VIDEO-V1 的 32 位控制字、四位 K 标志与 640×320 帧尺寸，统一每行 320 字、每帧 102400 字的数据契约。
// Module Name: (header) fiber_video_v1_defs.vh
// File Name: host/rtl/fiber/fiber_video_v1_defs.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 集中定义 HSST-VIDEO-V1 的 32 位控制字、四位 K 标志与 640×320 帧尺寸，统一每行 320 字、每帧 102400 字的数据契约。
//
// Dependencies: 由视频打包器及 HSST 适配层 include；常量头文件，无独立时钟或复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`ifndef FIBER_VIDEO_V1_DEFS_VH
`define FIBER_VIDEO_V1_DEFS_VH
// HSST-VIDEO-V1: 32-bit fabric data plus four K bits.
`define FVV1_IDLE   32'h0000_00BC
`define FVV1_VSYNC  32'hFFAA_55BC
`define FVV1_HSTART 32'hAA00_00BC
`define FVV1_HEND   32'hCC00_00BC
`define FVV1_K_CTRL 4'b0001
`define FVV1_K_DATA 4'b0000
`define FVV1_WIDTH  640
`define FVV1_HEIGHT 320
`define FVV1_DATA_PER_ROW 320
`define FVV1_DATA_PER_FRAME 102400
`endif
