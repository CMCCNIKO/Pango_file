//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 定义 NONE、SOLID、DASHED、UNKNOWN 四种两位车道类型编码，区分无几何、实线、虚线与有几何但未分类。
// Module Name: (header) host_lane_type_defs.vh
// File Name: host/rtl/video/host_lane_type_defs.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 定义 NONE、SOLID、DASHED、UNKNOWN 四种两位车道类型编码，区分无几何、实线、虚线与有几何但未分类。
//
// Dependencies: 供车道显示与类型处理 include；纯常量定义，无时钟复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`ifndef HOST_LANE_TYPE_DEFS_VH
`define HOST_LANE_TYPE_DEFS_VH
`define HOST_LANE_TYPE_NONE   2'b00
`define HOST_LANE_TYPE_SOLID  2'b01
`define HOST_LANE_TYPE_DASHED 2'b10
// Geometry exists, but no line-type classifier is connected. Display only.
`define HOST_LANE_TYPE_UNKNOWN 2'b11
`endif
