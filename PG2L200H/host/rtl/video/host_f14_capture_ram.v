//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 224×168 RGB565 双时钟采集存储，默认 37632 像素；PDS 分支以十个官方 128×512 RAM 打包保存，读取数据与有效标志按一拍契约返回。
// Module Name: host_f14_capture_ram
// File Name: host/rtl/video/host_f14_capture_ram.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 224×168 RGB565 双时钟采集存储，默认 37632 像素；PDS 分支以十个官方 128×512 RAM 打包保存，读取数据与有效标志按一拍契约返回。
// F14_SIMULATION 分支提供行为双时钟存储模型，便于功能仿真，不代替实际综合 RAM 实例。
//
// Dependencies: f14_ram_128x512；独立写入/读取时钟及使能；存储体无复位端口，由采集与所有权控制保证有效。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Dual-clock capture store for the normalized 224x168 RGB565 frame.
//
// The implementation branch used by PDS is explicitly packed into ten
// 128x512 PG2L RAM IPs (the same generated RAM used by the F14 image bank).
// Simulation deliberately uses a small behavioral dual-clock model so that
// ModelSim does not need to elaborate the vendor primitive.  Both branches
// have the same one-cycle read contract.
module host_f14_capture_ram #(
    parameter integer C_DEPTH = 37632
) (
    input  wire        i_wr_clk,
    input  wire        i_wr_en,
    input  wire [15:0] i_wr_addr,
    input  wire [15:0] i_wr_data,
    input  wire        i_rd_clk,
    input  wire        i_rd_en,
    input  wire [15:0] i_rd_addr,
    output wire [15:0] o_rd_data,
    output reg         o_rd_valid
);
    localparam integer BANK_COUNT = 10;

`ifdef F14_SIMULATION
    reg [15:0] sim_mem [0:C_DEPTH-1];
    reg [15:0] sim_rd_data_q;
    integer k;
    initial begin
        for (k = 0; k < C_DEPTH; k = k + 1)
            sim_mem[k] = 16'd0;
    end
    always @(posedge i_wr_clk) begin
        if (i_wr_en && (i_wr_addr < C_DEPTH))
            sim_mem[i_wr_addr] <= i_wr_data;
    end
    always @(posedge i_rd_clk) begin
        o_rd_valid <= i_rd_en;
        if (i_rd_en && (i_rd_addr < C_DEPTH))
            sim_rd_data_q <= sim_mem[i_rd_addr];
        else if (i_rd_en)
            sim_rd_data_q <= 16'd0;
    end
    assign o_rd_data = sim_rd_data_q;
`else
    reg [127:0] wr_pack_q;
    wire [12:0] wr_word_addr = i_wr_addr >> 3;
    wire [12:0] rd_word_addr = i_rd_addr >> 3;
    wire [3:0] wr_bank = wr_word_addr[12:9];
    wire [3:0] rd_bank = rd_word_addr[12:9];
    wire [8:0] wr_local = wr_word_addr[8:0];
    wire [8:0] rd_local = rd_word_addr[8:0];
    wire wr_valid = i_wr_en && (i_wr_addr < C_DEPTH);
    wire rd_valid = i_rd_en && (i_rd_addr < C_DEPTH);
    wire wr_word_en = wr_valid && (i_wr_addr[2:0] == 3'd7);
    wire [127:0] wr_word_data = {i_wr_data, wr_pack_q[111:0]};
    wire [127:0] bank_data [0:BANK_COUNT-1];
    reg [3:0] rd_bank_q;
    reg [2:0] rd_lane_q;
    reg [127:0] selected_word_data;
    genvar b;
    generate
        for (b = 0; b < BANK_COUNT; b = b + 1) begin : g_capture_bank
            f14_ram_128x512 u_ram (
                .wr_data(wr_word_data), .wr_addr(wr_local),
                .wr_en(wr_word_en && (wr_bank == b)), .wr_clk(i_wr_clk),
                .wr_rst(1'b0), .rd_data(bank_data[b]), .rd_addr(rd_local),
                .rd_clk(i_rd_clk), .rd_clk_en(rd_valid && (rd_bank == b)),
                .rd_rst(1'b0)
            );
        end
    endgenerate
    always @(posedge i_wr_clk) begin
        if (wr_valid) begin
            case (i_wr_addr[2:0])
                3'd0: wr_pack_q[15:0]   <= i_wr_data;
                3'd1: wr_pack_q[31:16]  <= i_wr_data;
                3'd2: wr_pack_q[47:32]  <= i_wr_data;
                3'd3: wr_pack_q[63:48]  <= i_wr_data;
                3'd4: wr_pack_q[79:64]  <= i_wr_data;
                3'd5: wr_pack_q[95:80]  <= i_wr_data;
                3'd6: wr_pack_q[111:96] <= i_wr_data;
                3'd7: wr_pack_q[127:112] <= i_wr_data;
                default: begin end
            endcase
        end
    end
    always @(posedge i_rd_clk) begin
        o_rd_valid <= rd_valid;
        if (rd_valid) begin
            rd_bank_q <= rd_bank;
            rd_lane_q <= i_rd_addr[2:0];
        end
    end
    always @* begin
        case (rd_bank_q)
            4'd0: selected_word_data = bank_data[0];
            4'd1: selected_word_data = bank_data[1];
            4'd2: selected_word_data = bank_data[2];
            4'd3: selected_word_data = bank_data[3];
            4'd4: selected_word_data = bank_data[4];
            4'd5: selected_word_data = bank_data[5];
            4'd6: selected_word_data = bank_data[6];
            4'd7: selected_word_data = bank_data[7];
            4'd8: selected_word_data = bank_data[8];
            4'd9: selected_word_data = bank_data[9];
            default: selected_word_data = 128'd0;
        endcase
    end
    assign o_rd_data = selected_word_data[rd_lane_q*16 +: 16];
`endif
endmodule
`default_nettype wire
