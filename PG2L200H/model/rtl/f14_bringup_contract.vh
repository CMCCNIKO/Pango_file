//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 统一定义 F14 冻结接口常量：224×168 输入、S8/S16 特征尺寸、四车道/21行/28列输出和描述符数量。
// Module Name: (header) f14_bringup_contract.vh
// File Name: model/rtl/f14_bringup_contract.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 统一定义 F14 冻结接口常量：224×168 输入、S8/S16 特征尺寸、四车道/21行/28列输出和描述符数量。
// 同时规定 BRAM 单拍读、时钟频率及目标帧率周期预算，供各模块通过宏保持尺寸/位宽一致。
//
// Dependencies: 由引用本头文件的 F14 RTL 使用
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 编译期宏定义文件，无可实例化模块；include guard 和各常量保持原值。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`ifndef F14_BRINGUP_CONTRACT_VH
`define F14_BRINGUP_CONTRACT_VH
// Frozen F14 single-frame contract.
`define F14_INPUT_WIDTH       224
`define F14_INPUT_HEIGHT      168
`define F14_INPUT_PIXELS      37632
`define F14_STRIDE8_CHANNELS  24
`define F14_STRIDE8_HEIGHT    21
`define F14_STRIDE8_WIDTH     28
`define F14_STRIDE16_CHANNELS 72
`define F14_STRIDE16_HEIGHT   11
`define F14_STRIDE16_WIDTH    14
`define F14_HEAD_CHANNELS     64
`define F14_LANES              4
`define F14_OUTPUT_ROWS        21
`define F14_OUTPUT_COLS        28
`define F14_OUTPUT_WORD_BITS  32
`define F14_CLS_BITS          (`F14_LANES * `F14_OUTPUT_ROWS * `F14_OUTPUT_COLS * `F14_OUTPUT_WORD_BITS)
`define F14_VERTICAL_BITS     (`F14_LANES * `F14_OUTPUT_ROWS * `F14_OUTPUT_WORD_BITS)
`define F14_OFFSET_BITS       (`F14_LANES * `F14_OUTPUT_ROWS * `F14_OUTPUT_COLS * `F14_OUTPUT_WORD_BITS)
`define F14_BACKBONE_DESCRIPTORS 40
`define F14_HEAD_DESCRIPTORS     12
`define F14_DESCRIPTOR_BYTES     64
`define F14_SEGMENT_COUNT        15
`define F14_BRAM_READ_LATENCY    1
`define F14_10FPS_MAX_CYCLES_50M 5000000
`define F14_20FPS_MAX_CYCLES_75M 3750000
`define F14_CLOCK_BRINGUP_HZ  50000000
`define F14_CLOCK_TARGET_HZ   75000000
`define F14_CLOCK_UNIT_HZ     100000000
`endif
