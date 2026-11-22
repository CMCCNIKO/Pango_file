//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 两 bank、每行 52 位的曲线表双时钟存储：系统域写入四车道坐标/有效/类型，像素域按行地址一拍读取并对齐 bank 选择。
// Module Name: host_f14_curve_ram
// File Name: host/rtl/video/host_f14_curve_ram.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 两 bank、每行 52 位的曲线表双时钟存储：系统域写入四车道坐标/有效/类型，像素域按行地址一拍读取并对齐 bank 选择。
//
// Dependencies: f14_ram_128x512；F14_SIMULATION 时使用行为 RAM；bank 所有权握手由 host_f14_curve_bridge 承担，存储体不复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 两份曲线行表；异步读写域通过发布/确认握手交接整个bank。
`timescale 1ns/1ps
module host_f14_curve_ram (
    input wire i_wr_clk, input wire i_wr_en, input wire i_wr_bank,
    input wire [8:0] i_wr_addr, input wire [51:0] i_wr_data,
    input wire i_rd_clk, input wire i_rd_bank, input wire [8:0] i_rd_addr,
    output wire [51:0] o_rd_data
);
    wire [127:0] rd_data [0:1];
    reg bank_q;
    always @(posedge i_rd_clk) bank_q <= i_rd_bank;
    genvar b;
    generate for (b=0;b<2;b=b+1) begin : g_bank
`ifdef F14_SIMULATION
        reg [127:0] mem [0:511];
        reg [127:0] data_q;
        always @(posedge i_wr_clk)
            if (i_wr_en && i_wr_bank == b) mem[i_wr_addr] <= {76'd0,i_wr_data};
        always @(posedge i_rd_clk)
            if (i_rd_bank == b) data_q <= mem[i_rd_addr];
        assign rd_data[b] = data_q;
`else
        f14_ram_128x512 u_ram (
            .wr_clk(i_wr_clk), .wr_rst(1'b0), .wr_en(i_wr_en && i_wr_bank == b),
            .wr_addr(i_wr_addr), .wr_data({76'd0,i_wr_data}),
            .rd_clk(i_rd_clk), .rd_rst(1'b0), .rd_clk_en(i_rd_bank == b),
            .rd_addr(i_rd_addr), .rd_data(rd_data[b])
        );
`endif
    end endgenerate
    assign o_rd_data = bank_q ? rd_data[1][51:0] : rd_data[0][51:0];
endmodule
