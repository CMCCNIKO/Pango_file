//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 定义 PGV 视频 DMA 的 magic、协议版本、1280×720 RGB565 帧字节数、缓冲长度及 BAR1 寄存器地址。
// Module Name: (include) pcie_video_regs.vh
// File Name: source/pcie/pcie_video_regs.vh
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 定义 PGV 视频 DMA 的 magic、协议版本、1280×720 RGB565 帧字节数、缓冲长度及 BAR1 寄存器地址。
// 集中列出描述符基址/步长及 FREE、READY、HW、DONE、ERROR 状态编码，供 CSR 逻辑复用。
//
// Dependencies: pcie_video_regs.v 等视频 DMA 控制逻辑。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`ifndef PCIE_VIDEO_REGS_VH
`define PCIE_VIDEO_REGS_VH
`define PGV_MAGIC 32'h31564750
`define PGV_VERSION 32'h00000001
`define PGV_FRAME_BYTES 32'd1843200
`define PGV_BUFFER_BYTES 32'd1847296
`define PGV_REG_MAGIC 12'h400
`define PGV_REG_VERSION 12'h404
`define PGV_REG_COMMAND 12'h408
`define PGV_REG_STATUS 12'h40c
`define PGV_REG_GENERATION 12'h410
`define PGV_REG_REJECT 12'h414
`define PGV_REG_DONE 12'h418
`define PGV_REG_ERROR 12'h41c
`define PGV_REG_DROP 12'h420
`define PGV_REG_OVERFLOW 12'h424
`define PGV_REG_RATE30 12'h428
`define PGV_DESC_BASE 12'h500
`define PGV_DESC_STRIDE 12'h020
`define PGV_STATE_FREE 3'd0
`define PGV_STATE_READY 3'd1
`define PGV_STATE_HW 3'd2
`define PGV_STATE_DONE 3'd3
`define PGV_STATE_ERROR 3'd4
`endif
