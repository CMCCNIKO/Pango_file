//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 64/32 位逐拍有符号除法，先取绝对值执行 64 次恢复迭代，再按符号还原商，结果向零截断。
// Module Name: host_lane_signed_divider
// File Name: host/rtl/video/host_lane_signed_divider.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 64/32 位逐拍有符号除法，先取绝对值执行 64 次恢复迭代，再按符号还原商，结果向零截断。
// 空闲接受 start，忙时不接受新请求；分母为零时返回 0 并置 done。
//
// Dependencies: i_clk/i_rst_n；signed64 分子、signed32 分母与 done/result 握手。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 64/32 逐拍有符号除法，向零截断；零分母返回0。
`timescale 1ns/1ps
module host_lane_signed_divider (
 input wire i_clk,input wire i_rst_n,input wire i_start,
 input wire signed [63:0] i_num,input wire signed [31:0] i_den,
 output reg o_done,output reg signed [63:0] o_result
);
 reg busy,negative;
 reg [63:0] work_q,quotient;
 reg [31:0] denominator,remainder;
 reg [6:0] left;
 wire [32:0] shifted={remainder,work_q[63]};
 wire take=shifted>={1'b0,denominator};
 wire [63:0] next_quotient={quotient[62:0],take};
 always @(posedge i_clk or negedge i_rst_n) begin
  if(!i_rst_n) begin busy<=0;negative<=0;work_q<=0;quotient<=0;denominator<=0;remainder<=0;left<=0;o_done<=0;o_result<=0;end
  else begin
   o_done<=0;
   if(!busy && i_start) begin
    negative<=(i_num<0)!=(i_den<0);work_q<=i_num<0 ? -i_num : i_num;
    denominator<=i_den<0 ? -i_den : i_den;remainder<=0;quotient<=0;left<=64;
    if(i_den==0) begin o_done<=1;o_result<=0;end else busy<=1;
   end else if(busy) begin
    work_q<={work_q[62:0],1'b0};quotient<=next_quotient;
    remainder<=take ? shifted-{1'b0,denominator} : shifted;left<=left-1'b1;
    if(left==1) begin busy<=0;o_done<=1;o_result<=negative ? -next_quotient : next_quotient;end
   end
  end
 end
endmodule
