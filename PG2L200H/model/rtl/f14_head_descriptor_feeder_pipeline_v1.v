//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 根据 head scheduler 的 tap、输入组和像素对，从128位 packed特征口及32位权重ROM口收集操作数。
// Module Name: f14_head_descriptor_feeder_runtime_v1
// File Name: model/rtl/f14_head_descriptor_feeder_pipeline_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 根据 head scheduler 的 tap、输入组和像素对，从128位 packed特征口及32位权重ROM口收集操作数。
// 经cache地址/命中流水重组成两个像素与八滤波器的MAC包；padding不读内存，标签FIFO维护含无效lane的有序响应。
//
// Dependencies: f14_head_operand_packer、官方 f14_ram_128x512 缓存；两路独立有序读口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 配置/clear/reset使缓存失效；外部读流水必须共同flush，不支持取消事务后继续返回旧响应。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Target Devices: PG2L200H-6FBB484; Tool: PDS 2022.2-SP6.4
// v4整字读取：特征接口为128bit packed word，单独选择源集。
// 两路独立有序读口，每拍最多一个请求/响应，延迟>=1拍。
// 每个uop读取0/1/2个word，两个pixel按奇偶半字重组；padding不读内存。
// ST_PREP寄存word地址和cache index，ST_LOOKUP寄存命中，随后驱动RAM/ROM。
// 缓存由4个本工程官方128x512 RAM并联组成，配置/clear/reset失效。
// 响应进入原operand暂存；64bit标签FIFO处理padding产生的lane空洞。
// reset/clear必须与外部读流水同步flush，不支持取消后仍返回旧事务。
`timescale 1ns/1ps
`default_nettype none
module f14_head_descriptor_feeder_runtime_v1 #(
    parameter integer FEATURE_ADDR_WIDTH = 18
) (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_clear,
    input wire i_cfg_valid,
    input wire [7:0] i_cfg_input_h,
    input wire [7:0] i_cfg_input_w,
    input wire [7:0] i_cfg_input_c,
    input wire [3:0] i_cfg_kernel_h,
    input wire [3:0] i_cfg_kernel_w,
    input wire [3:0] i_cfg_stride_x,
    input wire [16:0] i_cfg_rom_word_base,
    input wire [1:0] i_cfg_dest_kind,
    input wire [10:0] i_cfg_dest_base,
    input wire [1023:0] i_cfg_bias_bus,
    input wire i_sched_valid,
    output wire o_sched_ready,
    input wire i_sched_first,
    input wire i_sched_last,
    input wire [7:0] i_sched_output_group,
    input wire [7:0] i_sched_input_group,
    input wire [31:0] i_sched_transaction_index,
    input wire [7:0] i_sched_tap_y,
    input wire [7:0] i_sched_tap_x,
    input wire signed [15:0] i_sched_input_y,
    input wire signed [15:0] i_sched_input_x,
    input wire i_sched_pixel0_valid,
    input wire i_sched_pixel1_valid,
    output wire o_feature_rd_en,
    output wire [FEATURE_ADDR_WIDTH-1:0] o_feature_rd_addr,
    input wire i_feature_rd_valid,
    input wire [127:0] i_feature_rd_data,
    output wire o_rom_rd_en,
    output wire [16:0] o_rom_rd_addr,
    input wire i_rom_rd_ready,
    input wire i_run_enable,
    input wire i_rom_rd_valid,
    input wire [31:0] i_rom_rd_data,
    input wire i_backend_ready,
    output wire o_uop_valid,
    output wire o_uop_first,
    output wire o_uop_last,
    output wire [15:0] o_uop_lane_valid,
    output wire [127:0] o_uop_term_valid,
    output wire [1023:0] o_uop_qd,
    output wire [1023:0] o_uop_qw,
    output wire o_txn_meta_valid,
    output wire [1:0] o_txn_dest_kind,
    output wire [10:0] o_txn_dest_addr,
    output reg [255:0] o_txn_bias_f8,
    output wire o_busy,
    output wire o_error
);

    localparam [3:0] ST_IDLE=4'd0, ST_FETCH=4'd1, ST_PREP=4'd2, ST_LOOKUP=4'd3, ST_ISSUE=4'd5;
    reg [3:0] state_q, state_d;
    reg first_q, last_q, pixel0_valid_q, pixel1_valid_q, error_q;
    reg [7:0] output_group_q, input_group_q, tap_y_q, tap_x_q;
    reg [10:0] dest_addr_q;
    reg signed [15:0] input_y_q, input_x_q;
    reg [127:0] source_q;
    reg [511:0] weight_q;
    reg [7:0] input_h_q, input_w_q, input_c_q;
    reg [3:0] kernel_h_q, kernel_w_q, stride_x_q;
    reg [16:0] rom_base_q;
    reg [1:0] dest_kind_q;
    reg [10:0] dest_base_q;
    reg [1023:0] bias_bus_q;
    // 最多2个在途word；标签记录应写回的pixel掩码。
    reg [1:0] feature_tags [0:1];
    reg tag_write_q, tag_read_q;
    reg [1:0] feature_scan_q, feature_pending_q;
    reg [4:0] weight_sent_q, weight_received_q;
    reg [FEATURE_ADDR_WIDTH-1:0] word0_q, word1_q;
    reg half0_q, half1_q, same_word_q;
    reg [15:0] cache_index_q;
    reg cache_hit_q;
    reg [16:0] weight_base_q;
    integer feature_addr_calc, word0_calc, word1_calc, weight_addr_calc, tap_index_calc, cache_index_calc;
    reg [511:0] cache_valid_q;
    reg cache_read_pending_q, cache_ready_q;
    wire [511:0] cache_data;
    wire weight_bad;
    wire cache_eligible = (cache_index_q < 512);
    wire cache_hit = cache_hit_q;
    wire cache_read = i_rst_n && !i_clear && !error_q && (state_q == ST_FETCH)
        && (feature_scan_q == 0) && cache_hit;
    wire cache_write = i_rst_n && !i_clear && !error_q && (state_q == ST_FETCH)
        && i_rom_rd_valid && !weight_bad && (weight_received_q == 15) && cache_eligible;
    wire [511:0] cache_fill_data = {i_rom_rd_data, weight_q[479:0]};
    genvar cache_slice;
    generate
        for(cache_slice=0; cache_slice<4; cache_slice=cache_slice+1) begin: gen_weight_cache
            // 复用本工程PG2L200H官方IP的原配置：128bit x 512、读延迟1拍。
            f14_ram_128x512 u_weight_cache (
                .wr_data(cache_fill_data[cache_slice*128 +: 128]),
                .wr_addr(cache_index_q[8:0]), .wr_en(cache_write), .wr_clk(i_clk), .wr_rst(!i_rst_n),
                .rd_data(cache_data[cache_slice*128 +: 128]),
                .rd_addr(cache_index_q[8:0]), .rd_clk(i_clk), .rd_clk_en(cache_read), .rd_rst(!i_rst_n));
        end
    endgenerate
    integer x_calc, x1_calc, y_calc, input_groups_calc, row_words_calc;
    wire sched_fire = i_sched_valid && o_sched_ready;
    wire issue_fire = o_uop_valid && i_backend_ready;
    wire feature_pixel_valid = (feature_scan_q==0) ? pixel0_valid_q : (pixel1_valid_q && !same_word_q);
    wire [1:0] request_mask = (feature_scan_q==0) ? {same_word_q,1'b1} : 2'b10;
    wire feature_take = (state_q == ST_FETCH) && i_feature_rd_valid;
    wire weight_take = (state_q == ST_FETCH) && i_rom_rd_valid;
    wire feature_bad = feature_take && (feature_pending_q == 0);
    assign weight_bad = weight_take && (weight_received_q >= weight_sent_q);
    wire [1:0] feature_pending_next = feature_pending_q
        + {1'b0,o_feature_rd_en} - {1'b0,feature_take};
    wire [4:0] weight_received_next = weight_received_q + {4'd0,weight_take};
    wire all_returned = (feature_scan_q == 2'd2) && (feature_pending_next == 0)
        && (cache_ready_q || ((weight_sent_q == 5'd16) && (weight_received_next == 5'd16)));

    assign o_sched_ready = i_run_enable && i_rst_n && !i_clear && !error_q && (state_q == ST_IDLE);
    assign o_busy = i_rst_n && (state_q != ST_IDLE);
    assign o_error = error_q;
    assign o_feature_rd_en = i_rst_n && !i_clear && !error_q
        && (state_q == ST_FETCH) && (feature_scan_q < 2'd2) && feature_pixel_valid;
    assign o_rom_rd_en = i_rst_n && !i_clear && !error_q
        && (state_q == ST_FETCH) && (weight_sent_q < 5'd16) && !cache_hit;
    assign o_feature_rd_addr = (feature_scan_q==0) ? word0_q : word1_q;
    assign o_rom_rd_addr = weight_addr_calc[16:0];
    assign o_uop_valid = i_rst_n && !i_clear && !error_q && (state_q == ST_ISSUE);
    assign o_uop_first = first_q;
    assign o_uop_last = last_q;
    assign o_txn_meta_valid = o_uop_valid && last_q;
    assign o_txn_dest_kind = dest_kind_q;
    assign o_txn_dest_addr = dest_addr_q;
    always @(*) begin
        case(output_group_q[1:0])
            2'd0: o_txn_bias_f8 = bias_bus_q[255:0];
            2'd1: o_txn_bias_f8 = bias_bus_q[511:256];
            2'd2: o_txn_bias_f8 = bias_bus_q[767:512];
            default: o_txn_bias_f8 = bias_bus_q[1023:768];
        endcase
    end
    f14_head_operand_packer u_packer (
        .i_source_c8x2(source_q), .i_weight_f8c8(weight_q),
        .i_pixel0_valid(pixel0_valid_q), .i_pixel1_valid(pixel1_valid_q),
        .o_lane_valid(o_uop_lane_valid), .o_term_valid(o_uop_term_valid),
        .o_qd(o_uop_qd), .o_qw(o_uop_qw));
    always @(*) begin
        // 除法均为固定2/8的移位，地址在PREP寄存。
        x_calc = $signed(input_x_q);
        x1_calc = x_calc + stride_x_q;
        y_calc = $signed(input_y_q);
        input_groups_calc = (input_c_q + 7) >> 3;
        row_words_calc = ((input_w_q + 1) >> 1) * input_groups_calc;
        word0_calc = y_calc * row_words_calc + (x_calc >> 1) * input_groups_calc + input_group_q;
        word1_calc = y_calc * row_words_calc + (x1_calc >> 1) * input_groups_calc + input_group_q;
        feature_addr_calc = word0_calc;
        tap_index_calc = tap_y_q * kernel_w_q + tap_x_q;
        cache_index_calc = ((output_group_q * kernel_h_q * kernel_w_q) + tap_index_calc)
            * input_groups_calc + input_group_q;
        weight_addr_calc = weight_base_q + weight_sent_q;
    end
    always @(*) begin
        state_d = state_q;
        case(state_q)
            ST_IDLE: if(sched_fire) state_d = ST_PREP;
            ST_PREP: state_d = ST_LOOKUP;
            ST_LOOKUP: state_d = ST_FETCH;
            ST_FETCH: if(all_returned && !feature_bad && !weight_bad) state_d = ST_ISSUE;
            ST_ISSUE: if(issue_fire) state_d = ST_IDLE;
            default: state_d = ST_IDLE;
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) state_q <= ST_IDLE;
        else if(i_clear) state_q <= ST_IDLE;
        else state_q <= state_d;
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            first_q<=0; last_q<=0; pixel0_valid_q<=0; pixel1_valid_q<=0; error_q<=0;
            output_group_q<=0; input_group_q<=0; tap_y_q<=0; tap_x_q<=0;
            dest_addr_q<=0; input_y_q<=0; input_x_q<=0;
            source_q<=0; weight_q<=0;
            word0_q<=0; word1_q<=0; half0_q<=0; half1_q<=0; same_word_q<=0;
            cache_index_q<=0; cache_hit_q<=0; weight_base_q<=0;
            input_h_q<=0; input_w_q<=0; input_c_q<=0;
            kernel_h_q<=0; kernel_w_q<=0; stride_x_q<=0;
            rom_base_q<=0; dest_kind_q<=0; dest_base_q<=0; bias_bus_q<=0;
            feature_scan_q<=0; feature_pending_q<=0; tag_write_q<=0; tag_read_q<=0;
            weight_sent_q<=0; weight_received_q<=0;
            cache_valid_q<=0; cache_read_pending_q<=0; cache_ready_q<=0;
        end else if(i_clear) begin
            // 调用方同时flush存储返回流水；clear期间不接单/不发读/不交付。
            error_q<=0; feature_scan_q<=0; feature_pending_q<=0;
            tag_write_q<=0; tag_read_q<=0; weight_sent_q<=0; weight_received_q<=0;
            cache_valid_q<=0; cache_read_pending_q<=0; cache_ready_q<=0;
        end else begin
            cache_read_pending_q<=cache_read;
            if(cache_write) cache_valid_q[cache_index_q[8:0]]<=1'b1;
            if(i_cfg_valid) begin
                cache_valid_q<=0;
                input_h_q<=i_cfg_input_h; input_w_q<=i_cfg_input_w; input_c_q<=i_cfg_input_c;
                kernel_h_q<=i_cfg_kernel_h; kernel_w_q<=i_cfg_kernel_w; stride_x_q<=i_cfg_stride_x;
                rom_base_q<=i_cfg_rom_word_base; dest_kind_q<=i_cfg_dest_kind;
                dest_base_q<=i_cfg_dest_base; bias_bus_q<=i_cfg_bias_bus;
            end
            if(state_q==ST_PREP) begin
                word0_q<=word0_calc[FEATURE_ADDR_WIDTH-1:0];
                word1_q<=word1_calc[FEATURE_ADDR_WIDTH-1:0];
                half0_q<=input_x_q[0]; half1_q<=x1_calc[0];
                same_word_q<=pixel0_valid_q && pixel1_valid_q && (word0_calc==word1_calc);
                cache_index_q<=cache_index_calc[15:0];
            end
            if(state_q==ST_LOOKUP) begin
                cache_hit_q<=cache_eligible && cache_valid_q[cache_index_q[8:0]];
                weight_base_q<=rom_base_q + (cache_index_q << 4);
            end
            if(sched_fire) begin
                first_q<=i_sched_first; last_q<=i_sched_last;
                output_group_q<=i_sched_output_group; input_group_q<=i_sched_input_group;
                tap_y_q<=i_sched_tap_y; tap_x_q<=i_sched_tap_x;
                dest_addr_q<=dest_base_q+i_sched_transaction_index[10:0];
                input_y_q<=i_sched_input_y; input_x_q<=i_sched_input_x;
                pixel0_valid_q<=i_sched_pixel0_valid; pixel1_valid_q<=i_sched_pixel1_valid;
                source_q<=0; weight_q<=0; cache_ready_q<=0;
                feature_scan_q<=0; feature_pending_q<=0; tag_write_q<=0; tag_read_q<=0;
                weight_sent_q<=0; weight_received_q<=0;
            end else if(state_q==ST_FETCH) begin
                if(cache_read_pending_q) begin
                    weight_q<=cache_data;
                    cache_ready_q<=1'b1;
                end
                if(feature_scan_q<2'd2) feature_scan_q<=feature_scan_q+1'b1;
                if(o_feature_rd_en) begin
                    feature_tags[tag_write_q]<=request_mask;
                    tag_write_q<=tag_write_q+1'b1;
                end
                if(feature_take && !feature_bad) begin
                    if(feature_tags[tag_read_q][0]) begin
                        source_q[63:0]<=half0_q ? i_feature_rd_data[127:64] : i_feature_rd_data[63:0];
                    end
                    if(feature_tags[tag_read_q][1]) begin
                        source_q[127:64]<=half1_q ? i_feature_rd_data[127:64] : i_feature_rd_data[63:0];
                    end
                    tag_read_q<=tag_read_q+1'b1;
                end
                feature_pending_q<=feature_pending_next;
                if(o_rom_rd_en && i_rom_rd_ready) weight_sent_q<=weight_sent_q+1'b1;
                if(weight_take && !weight_bad) begin
                    weight_q[weight_received_q[3:0]*32 +: 32]<=i_rom_rd_data;
                    weight_received_q<=weight_received_next;
                end
                if(feature_bad || weight_bad) error_q<=1'b1;
            end
            if(state_q!=ST_IDLE && state_q!=ST_FETCH && state_q!=ST_ISSUE && state_q!=ST_PREP && state_q!=ST_LOOKUP) error_q<=1'b1;
        end
    end
endmodule
`default_nettype wire
