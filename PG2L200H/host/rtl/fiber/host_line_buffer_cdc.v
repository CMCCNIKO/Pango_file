//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 四 bank 双时钟行缓存：像素域将两个 RGB565 像素拼为 32 位字，整行提交后冻结帧号/行号/bank 描述符，经异步 FIFO 交给发送域。
// Module Name: host_line_buffer_cdc
// File Name: host/rtl/fiber/host_line_buffer_cdc.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 四 bank 双时钟行缓存：像素域将两个 RGB565 像素拼为 32 位字，整行提交后冻结帧号/行号/bank 描述符，经异步 FIFO 交给发送域。
// 发送侧按 RAM/FIFO 实际读取延迟执行 ready/valid 传输，发送或丢弃后归还 bank；flush 排空描述符与归还队列、释放 bank，并报告两域完成。
//
// Dependencies: host_line_ram、host_descriptor_fifo、host_bank_return_fifo；像素域与发送域独立复位，默认有效 RAM/FIFO 读取等待为 3/2 拍。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// Four-bank 2048x32 line store for the PG2L200H HOST.
//
// Fixes over the prior revision:
//   1. Write address/data/enable registered together as one RAM transaction
//      (no combinational address that advances past its data).
//   2. Descriptor frozen (frame_id/row/bank) at row commit, published only
//      after the final write, and records the bank actually written.
//   3. Vendor read latency honored. Measured against the Pango GTP simulation
//      primitives: RAM (OUTPUT_REG=1) = 2-cycle read; async FIFO
//      (OUTPUT_REG=0) = 1-cycle read. With a registered control signal the
//      effective latencies are C_RAM_RD_LAT=3 and C_FIFO_RD_LAT=2.
//   4. Real flush: i_flush_pix/i_flush_tx abandon the current row, drain the
//      descriptor and bank-return FIFOs, release every bank, and only then
//      assert the per-domain o_flush_done_* outputs.
module host_line_buffer_cdc #(
    parameter integer C_DATA_PER_ROW = 320,
    parameter integer C_RAM_RD_LAT  = 3,
    parameter integer C_FIFO_RD_LAT = 2
) (
    input wire i_pix_clk, input wire i_pix_rst_n,
    input wire i_pixel_valid, input wire [15:0] i_pixel_rgb565,
    input wire i_row_start, input wire i_row_commit,
    input wire [15:0] i_frame_id, input wire [8:0] i_row_index,
    input wire i_tx_clk, input wire i_tx_rst_n,
    output reg o_desc_valid, input wire i_desc_ready,
    input wire i_desc_drop,
    output reg [15:0] o_desc_frame_id, output reg [8:0] o_desc_row,
    output reg [1:0] o_desc_bank,
    output reg o_data_valid, input wire i_data_ready,
    output reg [31:0] o_data32,
    input wire i_bank_return_valid, input wire [1:0] i_bank_return,
    // Flush: level inputs (held while draining), level outputs (drain complete).
    input wire i_flush_pix,
    output wire o_flush_done_pix,
    input wire i_flush_tx,
    output wire o_flush_done_tx,
    output wire o_buffer_abort
);
    // ------------------------------------------------------------------
    // RAM + FIFO instances (unchanged vendor wrappers).
    // ------------------------------------------------------------------
    reg  [10:0] ram_wr_addr_r;
    reg  [31:0] ram_wr_data_r;
    reg         ram_wr_en_r;
    reg  [10:0] ram_rd_addr_r;
    wire [31:0] ram_rd_data;

    wire desc_wr_full, desc_rd_empty;
    wire [31:0] desc_rd_data;
    reg  desc_wr_en;
    reg  desc_rd_en;
    reg  [31:0] desc_wr_data_r;

    wire bank_wr_full, bank_rd_empty;
    wire [1:0] bank_rd_data;
    reg  bank_wr_en;
    reg  [1:0] bank_wr_data_r;
    reg  bank_rd_en;

    host_line_ram u_line_ram (
        .wr_data(ram_wr_data_r), .wr_addr(ram_wr_addr_r), .wr_en(ram_wr_en_r),
        .wr_clk(i_pix_clk), .wr_rst(~i_pix_rst_n),
        .rd_addr(ram_rd_addr_r), .rd_data(ram_rd_data),
        .rd_clk(i_tx_clk), .rd_rst(~i_tx_rst_n)
    );
    host_descriptor_fifo u_desc_fifo (
        .wr_clk(i_pix_clk), .wr_rst(~i_pix_rst_n), .wr_en(desc_wr_en),
        .wr_data(desc_wr_data_r), .wr_full(desc_wr_full), .wr_water_level(),
        .almost_full(), .rd_clk(i_tx_clk), .rd_rst(~i_tx_rst_n),
        .rd_en(desc_rd_en), .rd_data(desc_rd_data), .rd_empty(desc_rd_empty),
        .rd_water_level(), .almost_empty()
    );
    host_bank_return_fifo u_bank_fifo (
        .wr_clk(i_tx_clk), .wr_rst(~i_tx_rst_n), .wr_en(bank_wr_en),
        .wr_data(bank_wr_data_r), .wr_full(bank_wr_full), .wr_water_level(),
        .almost_full(), .rd_clk(i_pix_clk), .rd_rst(~i_pix_rst_n),
        .rd_en(bank_rd_en), .rd_data(bank_rd_data), .rd_empty(bank_rd_empty),
        .rd_water_level(), .almost_empty()
    );

    // ------------------------------------------------------------------
    // Pixel domain.
    // ------------------------------------------------------------------
    reg [1:0]  active_bank;
    reg [8:0]  word_index;
    reg        wr_half;
    reg [15:0] wr_low_pixel;
    reg [3:0]  bank_free;
    reg [15:0] row_frame_id;
    reg [8:0]  row_row_index;
    reg        row_active;
    reg        pending_desc_valid;
    reg [31:0] pending_desc;
    reg        buffer_abort_q;

    reg        bank_rd_req;
    reg [1:0]  bank_rd_wait;

    wire       row_has_free_bank = |bank_free;
    wire       row_desc_slot_ok = !pending_desc_valid || !desc_wr_full;
    wire [1:0] row_alloc_bank = bank_free[0] ? 2'd0 :
                                 bank_free[1] ? 2'd1 :
                                 bank_free[2] ? 2'd2 : 2'd3;
    wire       row_alloc_ok = row_has_free_bank && row_desc_slot_ok;

    always @(posedge i_pix_clk or negedge i_pix_rst_n) begin
        if (!i_pix_rst_n) begin
            active_bank <= 2'd0; word_index <= 9'd0; wr_half <= 1'b0;
            wr_low_pixel <= 16'd0; bank_free <= 4'b1111;
            row_frame_id <= 16'd0; row_row_index <= 9'd0;
            row_active <= 1'b0;
            pending_desc_valid <= 1'b0; pending_desc <= 32'd0;
            buffer_abort_q <= 1'b0;
            ram_wr_addr_r <= 11'd0; ram_wr_data_r <= 32'd0; ram_wr_en_r <= 1'b0;
            desc_wr_en <= 1'b0; desc_wr_data_r <= 32'd0;
            bank_rd_en <= 1'b0; bank_rd_req <= 1'b0; bank_rd_wait <= 2'd0;
        end else begin
            ram_wr_en_r <= 1'b0;
            desc_wr_en <= 1'b0;
            bank_rd_en <= 1'b0;
            buffer_abort_q <= 1'b0;

            if (i_flush_pix) begin
                // Abandon the current row, release every bank, drop any pending
                // descriptor, and drain the bank-return FIFO read side.
                active_bank <= 2'd0;
                word_index <= 9'd0;
                wr_half <= 1'b0;
                wr_low_pixel <= 16'd0;
                bank_free <= 4'b1111;
                row_active <= 1'b0;
                pending_desc_valid <= 1'b0;
                pending_desc <= 32'd0;
                bank_rd_req <= 1'b0;
                bank_rd_wait <= 2'd0;
                if (!bank_rd_empty)
                    bank_rd_en <= 1'b1;
            end else begin
                // Bank-return FIFO read with measured latency.
                if (bank_rd_req) begin
                    if (bank_rd_wait == C_FIFO_RD_LAT - 1) begin
                        bank_free[bank_rd_data] <= 1'b1;
                        bank_rd_req <= 1'b0;
                    end else
                        bank_rd_wait <= bank_rd_wait + 1'b1;
                end else if (!bank_rd_empty) begin
                    bank_rd_en <= 1'b1;
                    bank_rd_req <= 1'b1;
                    bank_rd_wait <= 2'd0;
                end

                // Publish a frozen descriptor when the FIFO has room.
                if (pending_desc_valid && !desc_wr_full) begin
                    desc_wr_en <= 1'b1;
                    desc_wr_data_r <= pending_desc;
                    pending_desc_valid <= 1'b0;
                end

                // row_start: allocate the lowest FREE bank; accept first pixel.
                if (i_row_start) begin
                    if (row_alloc_ok) begin
                        active_bank <= row_alloc_bank;
                        bank_free[row_alloc_bank] <= 1'b0;
                        row_active <= 1'b1;
                    end else begin
                        // Never alias a busy bank.  The frame is aborted and
                        // frame control will flush all outstanding state.
                        row_active <= 1'b0;
                        buffer_abort_q <= 1'b1;
                    end
                    word_index <= 9'd0;
                    wr_half <= 1'b0;
                    row_frame_id <= i_frame_id;
                    row_row_index <= i_row_index;
                end

                // Two pixels -> one registered RAM write transaction.
                if (i_pixel_valid && (row_active || (i_row_start && row_alloc_ok))) begin
                    if (!wr_half) begin
                        wr_low_pixel <= i_pixel_rgb565;
                        wr_half <= 1'b1;
                    end else begin
                        ram_wr_addr_r <= {active_bank, word_index};
                        ram_wr_data_r <= {i_pixel_rgb565, wr_low_pixel};
                        ram_wr_en_r <= 1'b1;
                        wr_half <= 1'b0;
                        if (word_index < C_DATA_PER_ROW-1)
                            word_index <= word_index + 1'b1;
                    end
                end

                // row_commit: freeze the descriptor using the bank actually written.
                if (i_row_commit) begin
                    if (row_active) begin
                        if (!pending_desc_valid || !desc_wr_full) begin
                            pending_desc <= {5'b0, row_frame_id, row_row_index, active_bank};
                            pending_desc_valid <= 1'b1;
                        end else begin
                            // The row was allocated before the FIFO became
                            // full. Release its bank and abort the frame;
                            // never overwrite the older pending descriptor.
                            bank_free[active_bank] <= 1'b1;
                            buffer_abort_q <= 1'b1;
                        end
                        row_active <= 1'b0;
                    end
                    wr_half <= 1'b0;
                    word_index <= 9'd0;
                end
            end
        end
    end

    // Pixel-side drain complete (only meaningful while flushing): all banks
    // free, return FIFO empty, nothing pending.
    assign o_flush_done_pix = i_flush_pix && (bank_free == 4'b1111) &&
                              bank_rd_empty && !pending_desc_valid;
    assign o_buffer_abort = buffer_abort_q;

    // ------------------------------------------------------------------
    // Fabric domain.
    // ------------------------------------------------------------------
    localparam [1:0] DS_IDLE = 2'd0, DS_WAIT = 2'd1, DS_HOLD = 2'd2;
    reg [1:0]  desc_state;
    reg [1:0]  desc_wait;
    reg [31:0] desc_hold;

    reg [1:0]  ram_rd_wait;
    reg        ram_rd_active;

    reg [1:0]  rd_bank;
    reg [8:0]  rd_word;
    reg        reading;
    reg        return_pending;
    reg [1:0]  return_bank;

    always @(posedge i_tx_clk or negedge i_tx_rst_n) begin
        if (!i_tx_rst_n) begin
            desc_state <= DS_IDLE; desc_wait <= 2'd0; desc_hold <= 32'd0;
            desc_rd_en <= 1'b0;
            o_desc_valid <= 1'b0; o_desc_frame_id <= 16'd0;
            o_desc_row <= 9'd0; o_desc_bank <= 2'd0;
            ram_rd_addr_r <= 11'd0; ram_rd_wait <= 2'd0; ram_rd_active <= 1'b0;
            rd_bank <= 2'd0; rd_word <= 9'd0; reading <= 1'b0;
            return_pending <= 1'b0; return_bank <= 2'd0;
            o_data_valid <= 1'b0; o_data32 <= 32'd0;
            bank_wr_en <= 1'b0; bank_wr_data_r <= 2'd0;
        end else begin
            desc_rd_en <= 1'b0;
            bank_wr_en <= 1'b0;
            o_desc_valid <= 1'b0;

            if (i_flush_tx) begin
                // Abandon any in-flight read, drop pending return, and drain the
                // descriptor FIFO read side (discard tokens).
                desc_state <= DS_IDLE;
                desc_wait <= 2'd0;
                desc_hold <= 32'd0;
                o_desc_frame_id <= 16'd0;
                o_desc_row <= 9'd0;
                o_desc_bank <= 2'd0;
                ram_rd_active <= 1'b0;
                ram_rd_wait <= 2'd0;
                rd_bank <= 2'd0;
                rd_word <= 9'd0;
                reading <= 1'b0;
                return_pending <= 1'b0;
                return_bank <= 2'd0;
                o_data_valid <= 1'b0;
                o_data32 <= 32'd0;
                if (!desc_rd_empty)
                    desc_rd_en <= 1'b1;
            end else begin
                // --- Descriptor read FSM ---
                case (desc_state)
                    DS_IDLE: begin
                        if (!desc_rd_empty) begin
                            desc_rd_en <= 1'b1;
                            desc_wait <= 2'd0;
                            desc_state <= DS_WAIT;
                        end
                    end
                    DS_WAIT: begin
                        if (desc_wait == C_FIFO_RD_LAT - 1) begin
                            desc_hold <= desc_rd_data;
                            desc_state <= DS_HOLD;
                        end else
                            desc_wait <= desc_wait + 1'b1;
                    end
                    DS_HOLD: begin
                        o_desc_valid <= 1'b1;
                        o_desc_frame_id <= desc_hold[26:11];
                        o_desc_row       <= desc_hold[10:2];
                        o_desc_bank      <= desc_hold[1:0];
                        if (o_desc_valid && i_desc_ready) begin
                            o_desc_valid <= 1'b0;
                            if (i_desc_drop) begin
                                // Discarded row: release the bank without reading.
                                return_pending <= 1'b1;
                                return_bank <= desc_hold[1:0];
                            end else begin
                                rd_bank <= desc_hold[1:0];
                                rd_word <= 9'd0;
                                reading <= 1'b1;
                            end
                            desc_state <= DS_IDLE;
                        end
                    end
                    default: desc_state <= DS_IDLE;
                endcase

                // --- RAM read pipeline (addr -> C_RAM_RD_LAT -> data) ---
                // o_data_valid/o_data32 hold the delivered word until i_data_ready.
                if (reading && !ram_rd_active && !o_data_valid) begin
                    ram_rd_addr_r <= {rd_bank, rd_word};
                    ram_rd_active <= 1'b1;
                    ram_rd_wait <= 2'd0;
                end else if (ram_rd_active) begin
                    if (ram_rd_wait == C_RAM_RD_LAT - 1) begin
                        o_data32 <= ram_rd_data;
                        o_data_valid <= 1'b1;
                        ram_rd_active <= 1'b0;
                    end else
                        ram_rd_wait <= ram_rd_wait + 1'b1;
                end

                // --- Data handshake ---
                if (o_data_valid && i_data_ready) begin
                    o_data_valid <= 1'b0;
                    if (rd_word == C_DATA_PER_ROW-1) begin
                        reading <= 1'b0;
                        return_pending <= 1'b1;
                        return_bank <= rd_bank;
                        rd_word <= 9'd0;
                    end else
                        rd_word <= rd_word + 1'b1;
                end

                // --- Bank return ---
                if (return_pending && !bank_wr_full) begin
                    bank_wr_en <= 1'b1;
                    bank_wr_data_r <= return_bank;
                    return_pending <= 1'b0;
                end
            end
        end
    end

    // Fabric-side drain complete (only meaningful while flushing): descriptor
    // FIFO empty and no in-flight state.
    assign o_flush_done_tx = i_flush_tx && desc_rd_empty && (desc_state == DS_IDLE) &&
                             !reading && !return_pending && !o_desc_valid && !o_data_valid;
endmodule
