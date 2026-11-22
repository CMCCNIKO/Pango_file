//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 系统域将四车道锚点插值成逐行曲线并写入空闲 bank，完成后用发布/确认握手跨到像素域，在第零行行首原子切换整帧结果。
// Module Name: host_f14_curve_bridge
// File Name: host/rtl/video/host_f14_curve_bridge.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 系统域将四车道锚点插值成逐行曲线并写入空闲 bank，完成后用发布/确认握手跨到像素域，在第零行行首原子切换整帧结果。
// 像素侧同步读出每行车道坐标、有效位与类型；超过 C_MAX_AGE_FRAMES 无新结果时隐藏旧曲线，避免长时间沿用过期结果。
//
// Dependencies: host_lane_spatial、host_f14_curve_ram；i_result_clk/i_result_rst_n 与 i_pixel_clk/i_pixel_rst_n；输入表在插值期间由调用方保持。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 系统域插值，双bank曲线RAM，帧边界原子切换；像素路径仅一拍RAM读取。
`timescale 1ns/1ps
module host_f14_curve_bridge #(
    parameter integer C_MAX_AGE_FRAMES = 6
) (
    input wire i_result_clk, input wire i_result_rst_n,
    input wire i_result_valid, input wire [15:0] i_result_frame_id,
    input wire [839:0] i_result_x_table,
    input wire [83:0] i_result_valid_table,
    input wire [83:0] i_result_edge_table,
    input wire [7:0] i_result_type_table,
    output wire o_result_ready,
    input wire i_pixel_clk, input wire i_pixel_rst_n,
    input wire i_pixel_row_start, input wire [8:0] i_pixel_row,
    output wire [15:0] o_frame_id,
    output wire [3:0] o_lane_valid,
    output wire [39:0] o_lane_x,
    output wire [7:0] o_lane_type,
    output wire o_result_available
);
    localparam [1:0] ST_IDLE=0, ST_BUILD=1, ST_PUBLISH=2;
    reg [1:0] state_q;
    reg [15:0] frame_q;
    reg build_bank_q,publish_bank_q,publish_toggle_q;
    reg ack_pixel_q;
    reg [1:0] ack_sync_q;
    wire raster_done,raster_wr_en;
    wire [8:0] raster_row;
    wire [51:0] raster_word;
    assign o_result_ready=state_q==ST_IDLE && ack_sync_q[1]==publish_toggle_q;
    host_lane_spatial u_spatial (
        .i_clk(i_result_clk),.i_rst_n(i_result_rst_n),
        .i_start(i_result_valid && o_result_ready),
        .i_x_table(i_result_x_table),.i_valid_table(i_result_valid_table),
        .i_edge_table(i_result_edge_table),.i_type_table(i_result_type_table),
        .o_done(raster_done),.o_wr_en(raster_wr_en),.o_wr_row(raster_row),.o_wr_data(raster_word)
    );
    always @(posedge i_result_clk or negedge i_result_rst_n) begin
        if(!i_result_rst_n) begin
            state_q<=ST_IDLE;frame_q<=0;build_bank_q<=1;publish_bank_q<=0;publish_toggle_q<=0;ack_sync_q<=0;
        end else begin
            ack_sync_q<={ack_sync_q[0],ack_pixel_q};
            case(state_q)
                ST_IDLE: if(i_result_valid && o_result_ready) begin
                    frame_q<=i_result_frame_id;build_bank_q<=~publish_bank_q;state_q<=ST_BUILD;
                end
                ST_BUILD: if(raster_done) state_q<=ST_PUBLISH;
                ST_PUBLISH: begin publish_bank_q<=build_bank_q;publish_toggle_q<=~publish_toggle_q;state_q<=ST_IDLE;end
                default: state_q<=ST_IDLE;
            endcase
        end
    end
    reg [1:0] publish_sync_q;
    reg active_bank_q, have_result_q;
    reg [15:0] active_frame_q, frame_read_q;
    reg [7:0] age_q;
    reg show_q;
    wire frame_start = i_pixel_row_start && i_pixel_row==0;
    wire switch_now = frame_start && publish_sync_q[1] != ack_pixel_q;
    wire read_bank = switch_now ? publish_bank_q : active_bank_q;
    wire [51:0] read_data;
    always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
        if (!i_pixel_rst_n) begin
            publish_sync_q<=0; ack_pixel_q<=0; active_bank_q<=0; have_result_q<=0;
            active_frame_q<=0; frame_read_q<=0; age_q<=0; show_q<=0;
        end else begin
            publish_sync_q<={publish_sync_q[0],publish_toggle_q};
            if (switch_now) begin
                ack_pixel_q<=publish_sync_q[1]; active_bank_q<=publish_bank_q;
                active_frame_q<=frame_q; have_result_q<=1; age_q<=0;
            end else if(frame_start && age_q<255) age_q<=age_q+1'b1;
            frame_read_q<=switch_now ? frame_q : active_frame_q;
            show_q<=switch_now || (have_result_q && age_q<C_MAX_AGE_FRAMES &&
                       !(frame_start && age_q==C_MAX_AGE_FRAMES-1));
        end
    end
    host_f14_curve_ram u_curve_ram (
        .i_wr_clk(i_result_clk), .i_wr_en(raster_wr_en), .i_wr_bank(build_bank_q),
        .i_wr_addr(raster_row), .i_wr_data(raster_word), .i_rd_clk(i_pixel_clk),
        .i_rd_bank(read_bank), .i_rd_addr(i_pixel_row), .o_rd_data(read_data)
    );
    assign o_frame_id=frame_read_q;
    assign o_result_available=show_q;
    assign o_lane_type=show_q ? read_data[51:44] : 8'd0;
    genvar l;
    generate for(l=0;l<4;l=l+1) begin : g_lane
        assign o_lane_valid[l]=show_q && read_data[l*11+10];
        assign o_lane_x[l*10+:10]=read_data[l*11+:10];
    end endgenerate
endmodule
