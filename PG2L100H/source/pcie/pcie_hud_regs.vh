//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 集中定义 HUD 控制 ABI 的 magic、版本、能力位和 BAR1 0x600 起始各 DWORD 地址。
// Module Name: (include) pcie_hud_regs.vh
// File Name: source/pcie/pcie_hud_regs.vh
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 集中定义 HUD 控制 ABI 的 magic、版本、能力位和 BAR1 0x600 起始各 DWORD 地址。
// 包括基础请求、状态、扩展提交及 0x670 导航寄存器常量，仅提供宏，不生成硬件模块。
//
// Dependencies: 供 pcie_hud_regs、pcie_hud_navigation_regs 等 HUD CSR 逻辑 include。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`ifndef PCIE_HUD_REGS_VH
`define PCIE_HUD_REGS_VH
`define HUD_MAGIC 32'h31445548
`define HUD_VERSION 32'h00010000
`define HUD_CAPS 32'h000001ff
`define HUD_REG_MAGIC 12'h600
`define HUD_REG_VERSION 12'h604
`define HUD_REG_REQUEST 12'h608
`define HUD_REG_STATUS 12'h60c
`define HUD_REG_ACCEPTED_SEQ 12'h610
`define HUD_REG_APPLIED_SEQ 12'h614
`define HUD_REG_RK_STATE 12'h618
`define HUD_REG_EFFECTIVE_STATE 12'h61c
`define HUD_REG_REJECT_COUNT 12'h620
`define HUD_REG_LAST_ERROR 12'h624
`define HUD_REG_REJECTED_SEQ 12'h628
`define HUD_REG_SNAPSHOT_ID 12'h62c
`define HUD_REG_CAPS 12'h630
`define HUD_REG_LAST_ACCEPTED_REQUEST 12'h634
`define HUD_REG_EXT_REQUEST 12'h638
`define HUD_REG_TIME0 12'h63c
`define HUD_REG_TIME1 12'h640
`define HUD_REG_EXT_COMMIT 12'h644
`define HUD_REG_EXT_STATUS 12'h648
`define HUD_REG_EXT_APPLIED_SEQ 12'h64c
`define HUD_REG_NAV_SHADOW 12'h670
`define HUD_REG_NAV_CONTROL 12'h674
`define HUD_REG_NAV_STATUS 12'h678
`define HUD_REG_NAV_ACTIVE 12'h67c
`endif
