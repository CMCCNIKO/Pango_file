//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-17
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 保留7个在用SOP存储兼容模块，负责PG2 DRM叶子、宽字、分段/奇偶bank及合并量化参数存储；独立QBF8/Q14模块已归档。
// Module Name: f14_sop_ram_leaf, npu_m4k_wide_dual_leaf, npu_m4k_wide_single_leaf, npu_m4k_wide_leaf, npu_m4k_wide_sdp_bank, npu_m4k_wide_banked2, npu_m4k_qparams_bank
// File Name: model/rtl/f14_sop_pg2_ram_compat.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.01（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 保留7个在用SOP存储兼容模块，负责PG2 DRM叶子、宽字、分段/奇偶bank及合并量化参数存储；独立QBF8/Q14模块已归档。
// 保持tile原有端口和同步读语义，参数化拼接宽度与bank；单口模式写优先，读使能由现有冲突规则限定。
//
// Dependencies: f14_explicit_drm_ram；文件内f14_sop_ram_leaf及6个npu_m4k_* wrapper
//
// Revision:
// 2026-10-30 / Header 1.01 - 未用模块移入other_file，其余模块实现及指令保持不变。
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 真实模块名以文件内7个声明为准，旧头的f14_sop_pg2_ram_compat并非可实例化模块；无改变存储拍数。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-17
// Design Name: F14 SOP PG2L200H RAM compatibility layer
// Module Name: f14_sop_pg2_ram_compat
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Keeps the SOP numeric tile unchanged while replacing its
//              altsyncram-backed storage wrappers with the PG2 DRM boundary.
// Dependencies: f14_explicit_drm_ram
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`default_nettype none

// One SOP storage leaf.  The SOP tile uses synchronous read data, so the
// PG2L200H boundary is intentionally the same one-cycle contract as the
// original wrapper.  The final implementation maps each instance to a
// regenerated PG2 DRM IP; simulation uses f14_explicit_drm_ram's model.
module f14_sop_ram_leaf #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 128,
    parameter integer ADDR_WIDTH = 7,
    parameter integer SINGLE_PORT = 0
) (
    input wire clk,
    input wire rst_n,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    input wire rd_en,
    output wire [WORD_BYTES*8-1:0] rd_data
);
    wire write_fire = wr_en;
    wire read_fire = rd_en && ((SINGLE_PORT == 0) || !wr_en);

    f14_explicit_drm_ram #(
        .C_ADDR_WIDTH(ADDR_WIDTH),
        .C_DATA_WIDTH(WORD_BYTES*8),
        .C_DEPTH(DEPTH_WORDS),
        .C_INIT_FILE("NONE")
    ) u_drm (
        .i_clk(clk),
        .i_wr_en(write_fire),
        .i_wr_addr(wr_addr),
        .i_wr_data(wr_data),
        .i_rd_en(read_fire),
        .i_rd_addr(rd_addr),
        .o_rd_data(rd_data)
    );
endmodule

module npu_m4k_wide_dual_leaf #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 128,
    parameter integer ADDR_WIDTH = 7
) (
    input wire clk,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    input wire rd_en,
    output wire [WORD_BYTES*8-1:0] rd_data
);
    f14_sop_ram_leaf #(
        .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(DEPTH_WORDS),
        .ADDR_WIDTH(ADDR_WIDTH), .SINGLE_PORT(0)
    ) u_leaf (
        .clk(clk), .rst_n(1'b1), .wr_data(wr_data), .wr_addr(wr_addr),
        .wr_en(wr_en), .rd_addr(rd_addr), .rd_en(rd_en), .rd_data(rd_data)
    );
endmodule

module npu_m4k_wide_single_leaf #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 128,
    parameter integer ADDR_WIDTH = 7
) (
    input wire clk,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    input wire rd_en,
    output wire [WORD_BYTES*8-1:0] rd_data
);
    f14_sop_ram_leaf #(
        .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(DEPTH_WORDS),
        .ADDR_WIDTH(ADDR_WIDTH), .SINGLE_PORT(1)
    ) u_leaf (
        .clk(clk), .rst_n(1'b1), .wr_data(wr_data), .wr_addr(wr_addr),
        .wr_en(wr_en), .rd_addr(rd_addr), .rd_en(rd_en), .rd_data(rd_data)
    );
endmodule

module npu_m4k_wide_leaf #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 128,
    parameter integer ADDR_WIDTH = 7,
    parameter integer SINGLE_PORT = 0
) (
    input wire clk,
    input wire rst_n,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    input wire rd_en,
    output wire [WORD_BYTES*8-1:0] rd_data
);
    generate
        if (SINGLE_PORT != 0) begin : g_single_port
            npu_m4k_wide_single_leaf #(
                .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(DEPTH_WORDS),
                .ADDR_WIDTH(ADDR_WIDTH)
            ) u_leaf (
                .clk(clk), .wr_data(wr_data), .wr_addr(wr_addr),
                .wr_en(wr_en), .rd_addr(rd_addr), .rd_en(rd_en),
                .rd_data(rd_data)
            );
        end else begin : g_dual_port
            npu_m4k_wide_dual_leaf #(
                .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(DEPTH_WORDS),
                .ADDR_WIDTH(ADDR_WIDTH)
            ) u_leaf (
                .clk(clk), .wr_data(wr_data), .wr_addr(wr_addr),
                .wr_en(wr_en), .rd_addr(rd_addr), .rd_en(rd_en),
                .rd_data(rd_data)
            );
        end
    endgenerate
endmodule

// The SOP split points are retained because they bound the number of PG2 DRM
// blocks used by a wide logical bank.  The data path above is otherwise
// unchanged; only the physical storage primitive is different.
module npu_m4k_wide_sdp_bank #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 1024,
    parameter integer ADDR_WIDTH = 10,
    parameter integer SINGLE_PORT = 0
) (
    input wire clk,
    input wire rst_n,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_addr,
    input wire rd_en,
    output wire [WORD_BYTES*8-1:0] rd_data
);
    function integer f14_sop_address_width;
        input integer depth;
        integer value;
        begin
            value = depth - 1;
            f14_sop_address_width = 0;
            while (value > 0) begin
                f14_sop_address_width = f14_sop_address_width + 1;
                value = value >> 1;
            end
            if (f14_sop_address_width < 1)
                f14_sop_address_width = 1;
        end
    endfunction

    localparam integer USE_SPLIT =
        (DEPTH_WORDS == 384) || (DEPTH_WORDS == 576) ||
        (DEPTH_WORDS == 768) || (DEPTH_WORDS == 1024);
    localparam integer SEGMENT0_DEPTH =
        (DEPTH_WORDS == 384) ? 256 :
        ((DEPTH_WORDS == 576) ? 512 :
         ((DEPTH_WORDS == 768) ? 512 :
          ((DEPTH_WORDS == 1024) ? 512 : DEPTH_WORDS)));
    localparam integer SEGMENT1_DEPTH = DEPTH_WORDS - SEGMENT0_DEPTH;
    localparam integer SEGMENT0_ADDR_WIDTH =
        f14_sop_address_width(SEGMENT0_DEPTH);
    localparam integer SEGMENT1_ADDR_WIDTH =
        f14_sop_address_width(SEGMENT1_DEPTH);

    generate
        if (USE_SPLIT == 0) begin : g_one_segment
            wire [SEGMENT0_ADDR_WIDTH-1:0] wr_addr_local = wr_addr;
            wire [SEGMENT0_ADDR_WIDTH-1:0] rd_addr_local = rd_addr;
            npu_m4k_wide_leaf #(
                .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(DEPTH_WORDS),
                .ADDR_WIDTH(SEGMENT0_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
            ) u_leaf (
                .clk(clk), .rst_n(rst_n), .wr_data(wr_data),
                .wr_addr(wr_addr_local),
                .wr_en(wr_en), .rd_addr(rd_addr_local),
                .rd_en(rd_en), .rd_data(rd_data)
            );
        end else begin : g_two_segments
            wire select_segment1_wr = wr_addr >= SEGMENT0_DEPTH;
            wire select_segment1_rd = rd_addr >= SEGMENT0_DEPTH;
            wire [WORD_BYTES*8-1:0] segment0_data;
            wire [WORD_BYTES*8-1:0] segment1_data;
            wire [SEGMENT0_ADDR_WIDTH-1:0] wr_addr_segment0 = wr_addr;
            wire [SEGMENT0_ADDR_WIDTH-1:0] rd_addr_segment0 = rd_addr;
            wire [SEGMENT1_ADDR_WIDTH-1:0] wr_addr_segment1 =
                wr_addr - SEGMENT0_DEPTH;
            wire [SEGMENT1_ADDR_WIDTH-1:0] rd_addr_segment1 =
                rd_addr - SEGMENT0_DEPTH;
            reg read_segment1_q;

            npu_m4k_wide_leaf #(
                .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(SEGMENT0_DEPTH),
                .ADDR_WIDTH(SEGMENT0_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
            ) u_segment0 (
                .clk(clk), .rst_n(rst_n), .wr_data(wr_data),
                .wr_addr(wr_addr_segment0),
                .wr_en(wr_en && !select_segment1_wr),
                .rd_addr(rd_addr_segment0),
                .rd_en(rd_en && !select_segment1_rd), .rd_data(segment0_data)
            );
            npu_m4k_wide_leaf #(
                .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(SEGMENT1_DEPTH),
                .ADDR_WIDTH(SEGMENT1_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
            ) u_segment1 (
                .clk(clk), .rst_n(rst_n), .wr_data(wr_data),
                .wr_addr(wr_addr_segment1),
                .wr_en(wr_en && select_segment1_wr),
                .rd_addr(rd_addr_segment1),
                .rd_en(rd_en && select_segment1_rd), .rd_data(segment1_data)
            );

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    read_segment1_q <= 1'b0;
                else if (rd_en)
                    read_segment1_q <= select_segment1_rd;
            end
            assign rd_data = read_segment1_q ? segment1_data : segment0_data;
        end
    endgenerate
endmodule

module npu_m4k_wide_banked2 #(
    parameter integer WORD_BYTES = 16,
    parameter integer DEPTH_WORDS = 768,
    parameter integer ADDR_WIDTH = 12,
    parameter integer LOCAL_ADDR_WIDTH = 9,
    parameter integer SINGLE_PORT = 0
) (
    input wire clk,
    input wire rst_n,
    input wire [WORD_BYTES*8-1:0] wr_data,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire wr_en,
    input wire [ADDR_WIDTH-1:0] rd_base_addr,
    input wire rd_en,
    output reg [(WORD_BYTES*8*2)-1:0] rd_data2
);
    localparam integer BANK_DEPTH = (DEPTH_WORDS + 1) / 2;
    function integer f14_sop_address_width;
        input integer depth;
        integer value;
        begin
            value = depth - 1;
            f14_sop_address_width = 0;
            while (value > 0) begin
                f14_sop_address_width = f14_sop_address_width + 1;
                value = value >> 1;
            end
            if (f14_sop_address_width < 1)
                f14_sop_address_width = 1;
        end
    endfunction
    localparam integer BANK_ADDR_WIDTH =
        f14_sop_address_width(BANK_DEPTH);
    wire [ADDR_WIDTH-1:0] rd_addr0 = rd_base_addr;
    wire [ADDR_WIDTH-1:0] rd_addr1 = rd_base_addr + 1'b1;
    wire [BANK_ADDR_WIDTH-1:0] rd_row0 = rd_addr0 >> 1;
    wire [BANK_ADDR_WIDTH-1:0] rd_row1 = rd_addr1 >> 1;
    wire [BANK_ADDR_WIDTH-1:0] wr_row = wr_addr >> 1;
    wire [BANK_ADDR_WIDTH-1:0] bank0_rd_addr = rd_base_addr[0]
        ? rd_row1 : rd_row0;
    // 连续两个字的奇数bank始终使用base/2，避免无效的加法及选择链。
    wire [BANK_ADDR_WIDTH-1:0] bank1_rd_addr = rd_row0;
    wire [WORD_BYTES*8-1:0] bank0_data;
    wire [WORD_BYTES*8-1:0] bank1_data;
    reg rd_base_bank_q;

    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(BANK_DEPTH),
        .ADDR_WIDTH(BANK_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
    ) u_bank0 (
        .clk(clk), .rst_n(rst_n), .wr_data(wr_data), .wr_addr(wr_row),
        .wr_en(wr_en && !wr_addr[0]), .rd_addr(bank0_rd_addr),
        .rd_en(rd_en), .rd_data(bank0_data)
    );
    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(WORD_BYTES), .DEPTH_WORDS(BANK_DEPTH),
        .ADDR_WIDTH(BANK_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
    ) u_bank1 (
        .clk(clk), .rst_n(rst_n), .wr_data(wr_data), .wr_addr(wr_row),
        .wr_en(wr_en && wr_addr[0]), .rd_addr(bank1_rd_addr),
        .rd_en(rd_en), .rd_data(bank1_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_base_bank_q <= 1'b0;
        else if (rd_en)
            rd_base_bank_q <= rd_base_addr[0];
    end
    always @* begin
        if (!rd_base_bank_q)
            rd_data2 = {bank1_data, bank0_data};
        else
            rd_data2 = {bank0_data, bank1_data};
    end
endmodule



// Packed QBF/qscale store for the F14 numeric tile.
//
// The old implementation used eight 32-bit DRM banks for QBF and another
// eight 16-bit DRM banks for qscale.  Both assets are loaded in channel order
// (eight words per row), and both are consumed together one row at a time.
// Accumulating one complete row before writing lets us keep the same logical
// read contract with only three 128-bit DRM banks:
//   bank 0: QBF channels 0..3
//   bank 1: QBF channels 4..7
//   bank 2: qscale channels 0..7
//
// The load ports remain the original 32/16-bit ports, so the ROM asset format
// and loader ABI do not change.  Reads remain synchronous with one-cycle
// latency and preserve the original low-channel-first vector layout.
module npu_m4k_qparams_bank #(
    parameter integer ROW_DEPTH = 128,
    parameter integer ROW_ADDR_WIDTH = 7,
    parameter integer SINGLE_PORT = 1
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         qbf_wr_en,
    input  wire [9:0]                   qbf_wr_addr,
    input  wire [31:0]                  qbf_wr_data,
    input  wire                         qscale_wr_en,
    input  wire [9:0]                   qscale_wr_addr,
    input  wire [15:0]                  qscale_wr_data,
    input  wire                         rd_en,
    input  wire [8:0]                   rd_row,
    output wire [255:0]                 qbf_rd_data,
    output wire [127:0]                 qscale_rd_data
);
    wire [ROW_ADDR_WIDTH-1:0] qbf_wr_row = qbf_wr_addr >> 3;
    wire [ROW_ADDR_WIDTH-1:0] qscale_wr_row = qscale_wr_addr >> 3;
    wire qbf_row_in_range = (qbf_wr_row < ROW_DEPTH);
    wire qscale_row_in_range = (qscale_wr_row < ROW_DEPTH);

    reg [127:0] qbf_pack0_q;
    reg [127:0] qbf_pack1_q;
    reg [127:0] qscale_pack_q;

    wire qbf_write_bank0 = qbf_wr_en && qbf_row_in_range
        && (qbf_wr_addr[2:0] == 3'd3);
    wire qbf_write_bank1 = qbf_wr_en && qbf_row_in_range
        && (qbf_wr_addr[2:0] == 3'd7);
    wire qscale_write_bank2 = qscale_wr_en && qscale_row_in_range
        && (qscale_wr_addr[2:0] == 3'd7);

    wire [127:0] qbf_bank0_wr_data =
        {qbf_wr_data, qbf_pack0_q[95:0]};
    wire [127:0] qbf_bank1_wr_data =
        {qbf_wr_data, qbf_pack1_q[95:0]};
    wire [127:0] qscale_bank2_wr_data =
        {qscale_wr_data, qscale_pack_q[111:0]};

    // The three physical banks have independent write enables, so QBF and
    // qscale can be committed at their natural row boundaries without a
    // read-modify-write cycle.
    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(ROW_DEPTH),
        .ADDR_WIDTH(ROW_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
    ) u_qparams_qbf0 (
        .clk(clk), .rst_n(rst_n),
        .wr_data(qbf_bank0_wr_data), .wr_addr(qbf_wr_row),
        .wr_en(qbf_write_bank0), .rd_addr(rd_row[ROW_ADDR_WIDTH-1:0]),
        .rd_en(rd_en), .rd_data(qbf_rd_data[127:0])
    );
    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(ROW_DEPTH),
        .ADDR_WIDTH(ROW_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
    ) u_qparams_qbf1 (
        .clk(clk), .rst_n(rst_n),
        .wr_data(qbf_bank1_wr_data), .wr_addr(qbf_wr_row),
        .wr_en(qbf_write_bank1), .rd_addr(rd_row[ROW_ADDR_WIDTH-1:0]),
        .rd_en(rd_en), .rd_data(qbf_rd_data[255:128])
    );
    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(ROW_DEPTH),
        .ADDR_WIDTH(ROW_ADDR_WIDTH), .SINGLE_PORT(SINGLE_PORT)
    ) u_qparams_scale (
        .clk(clk), .rst_n(rst_n),
        .wr_data(qscale_bank2_wr_data), .wr_addr(qscale_wr_row),
        .wr_en(qscale_write_bank2),
        .rd_addr(rd_row[ROW_ADDR_WIDTH-1:0]), .rd_en(rd_en),
        .rd_data(qscale_rd_data)
    );

    // Channel words arrive in increasing address order.  The low slice is
    // filled first, matching the original bank_index*width concatenation.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qbf_pack0_q <= 128'd0;
            qbf_pack1_q <= 128'd0;
            qscale_pack_q <= 128'd0;
        end else begin
            if (qbf_wr_en && qbf_row_in_range) begin
                case (qbf_wr_addr[2:0])
                    3'd0: qbf_pack0_q[31:0]   <= qbf_wr_data;
                    3'd1: qbf_pack0_q[63:32]  <= qbf_wr_data;
                    3'd2: qbf_pack0_q[95:64]  <= qbf_wr_data;
                    3'd3: qbf_pack0_q[127:96] <= qbf_wr_data;
                    3'd4: qbf_pack1_q[31:0]   <= qbf_wr_data;
                    3'd5: qbf_pack1_q[63:32]  <= qbf_wr_data;
                    3'd6: qbf_pack1_q[95:64]  <= qbf_wr_data;
                    3'd7: qbf_pack1_q[127:96] <= qbf_wr_data;
                    default: begin end
                endcase
            end
            if (qscale_wr_en && qscale_row_in_range) begin
                case (qscale_wr_addr[2:0])
                    3'd0: qscale_pack_q[15:0]    <= qscale_wr_data;
                    3'd1: qscale_pack_q[31:16]   <= qscale_wr_data;
                    3'd2: qscale_pack_q[47:32]   <= qscale_wr_data;
                    3'd3: qscale_pack_q[63:48]   <= qscale_wr_data;
                    3'd4: qscale_pack_q[79:64]   <= qscale_wr_data;
                    3'd5: qscale_pack_q[95:80]   <= qscale_wr_data;
                    3'd6: qscale_pack_q[111:96]  <= qscale_wr_data;
                    3'd7: qscale_pack_q[127:112] <= qscale_wr_data;
                    default: begin end
                endcase
            end
        end
    end
endmodule

`default_nettype wire
