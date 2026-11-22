//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 定义 21 个原模型锚点在 640×320 显示坐标中的纵坐标 old_anchor_y，将原始 y=289..979 裁剪映射到显示 y=60..1019 再三倍缩小的坐标域。
// Module Name: (header) host_f14_old_coordinates.vh
// File Name: host/rtl/video/host_f14_old_coordinates.vh
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 定义 21 个原模型锚点在 640×320 显示坐标中的纵坐标 old_anchor_y，将原始 y=289..979 裁剪映射到显示 y=60..1019 再三倍缩小的坐标域。
//
// Dependencies: 被结果后处理和线型取样模块 include；纯函数表，无独立时钟复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Original crop289:980 projected into display crop60:1020 /3.
function [8:0] old_anchor_y;
 input [4:0] index;
 begin case(index)
 5'd0:old_anchor_y=9'd82;
 5'd1:old_anchor_y=9'd93;
 5'd2:old_anchor_y=9'd104;
 5'd3:old_anchor_y=9'd115;
 5'd4:old_anchor_y=9'd126;
 5'd5:old_anchor_y=9'd137;
 5'd6:old_anchor_y=9'd148;
 5'd7:old_anchor_y=9'd159;
 5'd8:old_anchor_y=9'd170;
 5'd9:old_anchor_y=9'd181;
 5'd10:old_anchor_y=9'd192;
 5'd11:old_anchor_y=9'd202;
 5'd12:old_anchor_y=9'd213;
 5'd13:old_anchor_y=9'd224;
 5'd14:old_anchor_y=9'd235;
 5'd15:old_anchor_y=9'd246;
 5'd16:old_anchor_y=9'd257;
 5'd17:old_anchor_y=9'd268;
 5'd18:old_anchor_y=9'd279;
 5'd19:old_anchor_y=9'd290;
 5'd20:old_anchor_y=9'd301;
 default:old_anchor_y=9'd0;
 endcase end
endfunction
