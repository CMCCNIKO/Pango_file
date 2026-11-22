//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 集中定义 HOST 的 IDLE/PATTERN/HDMI 模式编码、实际生效模式号、LED 状态选择及八位构建标识。
// Module Name: (header) host_build_config.vh
// File Name: host/rtl/fiber/host_build_config.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 集中定义 HOST 的 IDLE/PATTERN/HDMI 模式编码、实际生效模式号、LED 状态选择及八位构建标识。
// 通过 host_mode_select.vh 获取当前模式宏；嵌套条件编译保持 PATTERN、HDMI、IDLE 的分支优先级一致。
//
// Dependencies: host_mode_select.vh；供 host_hdmi_to_fiber_top 使用，无运行时逻辑。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原注释提及 scripts/build_host_modes.py，但本源码包未提供该脚本；不要把该历史流程写成本包可运行入口。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`ifndef HOST_BUILD_CONFIG_VH
`define HOST_BUILD_CONFIG_VH
// Compile-time TX source selection for the PG2L200H HOST.
//
// The Pango Verilog compiler supports only `ifdef / `ifndef / `else / `endif
// (no `if integer comparison, no `elsif), so the mode is selected with named
// defines. Exactly one of HOST_TX_MODE_IDLE / HOST_TX_MODE_PATTERN /
// HOST_TX_MODE_HDMI must be defined; the single source of truth for that
// define is rtl/fiber/host_mode_select.vh (included below), which
// scripts/build_host_modes.py rewrites and validates before synthesis. If none
// is defined, IDLE_ONLY is the safe default. Numeric equivalents are kept for
// manifests and status export.

// Single source of truth for the active mode define.
`include "host_mode_select.vh"

`define HOST_TX_IDLE_ONLY 0
`define HOST_TX_PATTERN   1
`define HOST_TX_HDMI      2

// Numeric mode value actually selected (for status/manifests). Precedence
// mirrors rtl/host_hdmi_to_fiber_top.v's data-select branch order
// (PATTERN > HDMI > IDLE), so the value reported here can never disagree with
// the branch that actually drives the TX word. A build that defines more than
// one mode is rejected by scripts/build_host_modes.py before synthesis, so
// this precedence is only a defensive fallback.
`ifdef HOST_TX_MODE_PATTERN
`define HOST_TX_MODE_VAL 1
`else
`ifdef HOST_TX_MODE_HDMI
`define HOST_TX_MODE_VAL 2
`else
`define HOST_TX_MODE_VAL 0
`endif
`endif

// The single constrained LED (V9, LVCMOS15, active-high per the board 1_led
// example) can reflect one status bit without adding a new pin.
//   0 = mode_valid, 1 = tx_ready (default), 2 = fabric_clock_alive.
`ifndef HOST_LED_SELECT
`define HOST_LED_SELECT 1
`endif

// 8-bit build identity retained by the existing status monitor. 0x12 marks
// the R06 HDMI source selection.
`ifndef HOST_BUILD_ID
`define HOST_BUILD_ID 8'h12
`endif
`endif
