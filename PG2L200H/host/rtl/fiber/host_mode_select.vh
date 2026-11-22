//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 单点选择当前 HOST 发送源；本文件定义 HOST_TX_MODE_HDMI，使顶层使用真实 HDMI 视频及车道元数据发送路径。
// Module Name: (header) host_mode_select.vh
// File Name: host/rtl/fiber/host_mode_select.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 单点选择当前 HOST 发送源；本文件定义 HOST_TX_MODE_HDMI，使顶层使用真实 HDMI 视频及车道元数据发送路径。
//
// Dependencies: 由 host_build_config.vh include；模式宏在编译时生效。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原注释提及的 delivery/ 与 success_version/ 不属于本源码包的运行依赖。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`ifndef HOST_MODE_SELECT_VH
`define HOST_MODE_SELECT_VH
// R06 T4 TX delivery mode: real HDMI -> V1 path.  P1/P3 VIDEO_PATTERN
// builds are preserved in delivery/ and success_version/.
`define HOST_TX_MODE_HDMI
`endif
