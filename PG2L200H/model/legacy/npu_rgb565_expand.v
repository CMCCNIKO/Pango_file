//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 通过位复制把 RGB565 的 R5/G6/B5 扩展为 RGB888 的三个 8 位通道。
// Module Name: npu_rgb565_expand
// File Name: model/legacy/npu_rgb565_expand.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 通过位复制把 RGB565 的 R5/G6/B5 扩展为 RGB888 的三个 8 位通道。
// 红蓝低位复制各自高 3 位，绿色低位复制高 2 位，保持确定性无乘法的颜色展开。
//
// Dependencies: RGB565 位序：R[15:11]、G[10:5]、B[4:0]
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 纯组合模块，无时钟、复位或额外拍延迟。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`default_nettype none

// Bit-replication expansion. This is identical to
// R8=(R5<<3)|(R5>>2), G8=(G6<<2)|(G6>>4), B8=(B5<<3)|(B5>>2).
module npu_rgb565_expand
(
    input  wire [15:0] rgb565,
    output wire [23:0] rgb888
);

wire [4:0] red5   = rgb565[15:11];
wire [5:0] green6 = rgb565[10:5];
wire [4:0] blue5  = rgb565[4:0];

assign rgb888[23:16] = {red5, red5[4:2]};
assign rgb888[15:8]  = {green6, green6[5:4]};
assign rgb888[7:0]   = {blue5, blue5[4:2]};

endmodule

`default_nettype wire
