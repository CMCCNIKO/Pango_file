//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 把两个相邻输出像素的C8源数据和F8×C8权重组合映射为16 lane×8 term的head微操作包。
// Module Name: f14_head_operand_packer
// File Name: model/rtl/f14_head_operand_packer.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 把两个相邻输出像素的C8源数据和F8×C8权重组合映射为16 lane×8 term的head微操作包。
// 输出lane_valid、term_valid及1024位qd/qw，布局与exp/proj打包器和共享MAC一致。
//
// Dependencies: 共享16×8后端接口；lane=filter×2+pixel，term=输入通道
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 纯组合模块，不含算术、存储、时钟或额外拍延迟。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// F14 generic head operand layout adapter.
//
// One head uop covers two adjacent output pixels and eight input channels.
// Eight output filters are interleaved as two pixels per lane, matching the
// existing npu_exp_proj_operand_packer_v1 and shared 16x8 backend:
// lane = filter*2 + pixel, term = input channel.
// This module contains no arithmetic or storage.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module f14_head_operand_packer (
    input  wire [127:0]  i_source_c8x2,
    input  wire [511:0]  i_weight_f8c8,
    input  wire          i_pixel0_valid,
    input  wire          i_pixel1_valid,
    output wire [15:0]   o_lane_valid,
    output wire [127:0]  o_term_valid,
    output wire [1023:0] o_qd,
    output wire [1023:0] o_qw
);
    genvar filter_index;
    genvar pixel_index;
    genvar term_index;
    generate
        for (filter_index = 0; filter_index < 8;
             filter_index = filter_index + 1) begin : g_filter
            for (pixel_index = 0; pixel_index < 2;
                 pixel_index = pixel_index + 1) begin : g_pixel
                localparam integer LANE_INDEX = filter_index*2 + pixel_index;
                wire pixel_valid = (pixel_index == 0)
                    ? i_pixel0_valid : i_pixel1_valid;
                assign o_lane_valid[LANE_INDEX] = pixel_valid;
                for (term_index = 0; term_index < 8;
                     term_index = term_index + 1) begin : g_term
                    localparam integer FLAT_TERM_INDEX =
                        LANE_INDEX*8 + term_index;
                    assign o_term_valid[FLAT_TERM_INDEX] = pixel_valid;
                    assign o_qd[FLAT_TERM_INDEX*8 +: 8] = i_source_c8x2[
                        (pixel_index*8 + term_index)*8 +: 8];
                    assign o_qw[FLAT_TERM_INDEX*8 +: 8] = i_weight_f8c8[
                        (filter_index*8 + term_index)*8 +: 8];
                end
            end
        end
    endgenerate
endmodule

`default_nettype wire
