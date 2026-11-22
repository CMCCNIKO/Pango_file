//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 协调输入图像所有权、骨干推理、adapter快照与三检测头的独立流水，使相邻帧的采集/骨干/三头按握手衔接。
// Module Name: f14_pipeline_core
// File Name: model/rtl/f14_pipeline_core.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 协调输入图像所有权、骨干推理、adapter快照与三检测头的独立流水，使相邻帧的采集/骨干/三头按握手衔接。
// 支持内部RGB565 bank或外部capture bank，骨干完成后复制adapter快照交给head，保存结果直到release；导出S8被动写tap和首层输入释放。
//
// Dependencies: f14_backbone_bram_core、f14_head_three_runtime_v1、f14_pipeline_rom_arbiter、f14_input_image_bank、f14_result_bram等
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 状态/帧号与槽所有权靠握手而非耗时预测；C_PARTITION_RESET保留八分区接口。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: PG2L200H-6FBB484; Tool: PDS 2022.2-SP6.4
// 单输入BRAM、单独adapter快照、独立骨干/三头流水。所有权靠握手，不依赖耗时预测。
`timescale 1ns/1ps
`default_nettype none
`include "f14_bringup_contract.vh"
module f14_pipeline_core #(
    parameter integer C_EXTERNAL_IMAGE = 0,
    parameter integer C_PARTITION_RESET = 0
) (
    input wire i_clk, input wire i_rst_n,
    input wire [7:0] i_reset_partition_n,
    input wire i_frame_valid, input wire [31:0] i_frame_id,
    output wire o_frame_accept,
    input wire i_pixel_valid, input wire [15:0] i_pixel_data, output wire o_pixel_ready,
    input wire i_head_enable,
    output wire o_result_valid, input wire i_result_release,
    output wire [31:0] o_result_frame_id, output wire [31:0] o_result_cycles,
    input wire i_result_req, input wire [12:0] i_result_addr,
    output wire o_result_read_valid, output wire [15:0] o_result_data,
    output wire o_status_error,
    input wire i_boot_rd_en, input wire [16:0] i_boot_rd_addr, output wire o_boot_rd_ready,
    output wire o_boot_rd_valid, output wire [31:0] o_boot_rd_data,
    // Optional external capture bank: same one-cycle image read contract.
    output wire o_image_rd_en, output wire [15:0] o_image_rd_addr,
    input wire [15:0] i_image_rd_data,
    // Passive post-residual S8 write tap. No backpressure or extra bank read.
    output wire o_s8_wr_en,
    output wire [13:0] o_s8_wr_addr,
    output wire [7:0] o_s8_wr_data,
    output wire o_input_release,
    output wire [31:0] o_s8_frame_id
);
    localparam integer IMAGE_PIXELS=37632, SCRATCH_WORDS=2048;
    reg [31:0] clock_q, image_id_q, image_start_q, bb_id_q, bb_start_q;
    reg [31:0] adapter_id_q, adapter_start_q, head_id_q, head_start_q, result_cycles_q;
    reg capture_q,image_full_q,image_owned_q,adapter_owned_q,error_q;
    reg [2:0] bb_state_q,head_state_q;
    reg [15:0] image_wr_addr_q;
    reg [11:0] copy_count_q,copy_address_q;
    wire bb_wide_valid;
    wire copy_req=(bb_state_q==3 && copy_count_q<2352);
    wire copy_last=bb_wide_valid && copy_address_q==2351;
    wire pixel_fire=i_pixel_valid && o_pixel_ready;
    wire head_rom_ready;
    wire [127:0] adapter_data;
    assign o_frame_accept=i_rst_n && !error_q && i_frame_valid && !capture_q && !image_full_q && !image_owned_q;
    assign o_pixel_ready=i_rst_n && !error_q && capture_q;
    assign o_result_valid=(head_state_q==4);
    assign o_result_frame_id=head_id_q;
    assign o_result_cycles=result_cycles_q;
    assign o_status_error=error_q;
    // 首层结束后不再读取外部输入，允许采集下一帧。
    assign o_s8_frame_id=bb_id_q;
	wire [15:0] image_rd_data;
	wire image_rd_en;
	wire [15:0] image_rd_addr;
	reg image_rd_valid_q;
	assign o_image_rd_en = image_rd_en;
	assign o_image_rd_addr = image_rd_addr;
	generate if (C_EXTERNAL_IMAGE == 0) begin : g_internal_image
	f14_input_image_bank #(.C_DEPTH(IMAGE_PIXELS)) u_input_image (
		.i_clk(i_clk), .i_wr_en(pixel_fire), .i_wr_addr(image_wr_addr_q),
		.i_wr_data(i_pixel_data), .i_rd_en(image_rd_en),
		.i_rd_addr(image_rd_addr), .o_rd_data(image_rd_data)
	);
	end else begin : g_external_image
		assign image_rd_data = i_image_rd_data;
	end endgenerate
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n)
			image_rd_valid_q <= 1'b0;
		else
			image_rd_valid_q <= image_rd_en;
	end

	wire bb_frame_valid, bb_frame_accept, bb_busy, bb_done;
	wire [31:0] bb_frame_id, bb_cycles;
	wire [9:0] bb_error_code;
	wire bb_stage_valid, bb_stage_ready;
	wire [3:0] bb_stage_index;
	wire bb_store_valid, bb_store_ready;
	wire [31:0] bb_store_addr;
	wire [7:0] bb_store_data;
	wire bb_probe_en, bb_probe_bank_b, bb_probe_valid, bb_probe_error;
	wire bb_rom_rd_en, bb_rom_rd_valid;
	wire [16:0] bb_rom_rd_addr;
	wire [31:0] bb_rom_rd_data;
	wire [17:0] bb_probe_addr;
	wire [7:0] bb_probe_data;
	wire head_start_ready;
	wire head_feature_rd_en;
	wire [17:0] head_feature_rd_addr;
	wire [1:0] head_feature_bank;
	wire head_feature_rd_valid;
	wire [127:0] head_feature_rd_data;

	wire [127:0] bb_wide_data;
	reg feature_req_q;
	reg [11:0] feature_addr_q;
	reg [1:0] feature_bank_q;
	assign bb_frame_valid = (bb_state_q==0) && image_full_q && !error_q;
	assign bb_stage_ready = 1'b1;
	assign bb_store_ready = 1'b1;
	assign o_s8_wr_en = bb_store_valid && bb_store_ready && bb_stage_index==6 &&
		bb_store_addr>=32'h30000 && bb_store_addr<32'h33720;
	assign o_s8_wr_addr = bb_store_addr[13:0];
	assign o_s8_wr_data = bb_store_data;
	// The alignment pass already writes the 64x21x28 adapter map into
	// backbone bank B at byte addresses 0..37631.  Head input bank 0
	// aliases that existing storage instead of duplicating an adapter bank.
	assign bb_probe_en = 1'b0;
	assign bb_probe_bank_b = 1'b1;
	assign bb_probe_addr = 18'd0;

	f14_backbone_bram_core #(.C_USE_EXTERNAL_IMAGE(1),.C_PARTITION_RESET(C_PARTITION_RESET)) u_backbone (
		.clk(i_clk), .rst_n(C_PARTITION_RESET ? i_reset_partition_n[1] : i_rst_n), .i_frame_valid(bb_frame_valid),
        .i_reset_partition_n(i_reset_partition_n),
		.i_head_wide_en(copy_req), .i_head_wide_addr(copy_count_q),
		.o_head_wide_valid(bb_wide_valid), .o_head_wide_data(bb_wide_data),
		.i_frame_id(image_id_q), .o_frame_accept(bb_frame_accept), .o_busy(bb_busy),
		.o_backbone_done(bb_done), .o_frame_id(bb_frame_id), .o_cycles(bb_cycles),
		.o_error_code(bb_error_code), .o_stage_valid(bb_stage_valid),
		.i_stage_ready(bb_stage_ready), .o_stage_index(bb_stage_index),
		.i_store_ready(bb_store_ready), .o_store_valid(bb_store_valid),
		.o_store_addr(bb_store_addr), .o_store_data(bb_store_data),
		.i_probe_en(bb_probe_en), .i_probe_bank_b(bb_probe_bank_b),
		.i_probe_addr(bb_probe_addr), .o_probe_valid(bb_probe_valid),
		.o_probe_data(bb_probe_data), .o_probe_error(bb_probe_error),
		.o_image_rd_en(image_rd_en), .o_image_rd_addr(image_rd_addr),
		.i_image_rd_valid(image_rd_valid_q), .i_image_rd_data(image_rd_data),
		.o_rom_rd_en(bb_rom_rd_en), .o_rom_rd_addr(bb_rom_rd_addr),
		.i_rom_rd_valid(bb_rom_rd_valid), .i_rom_rd_data(bb_rom_rd_data)
	);

	wire head_clear = (head_state_q==1);
	wire head_start = (head_state_q==2) && head_start_ready;
	wire head_rom_rd_en;
	wire [16:0] head_rom_rd_addr;
	wire head_rom_rd_valid;
	wire [31:0] head_rom_rd_data;
	wire head_word_valid;
	wire [1:0] head_word_bank, head_word_head;
	wire [10:0] head_word_addr;
	wire [127:0] head_word_data;
	wire head_s16_valid;
	wire [1:0] head_s16_bank, head_s16_head;
	wire [10:0] head_s16_addr;
	wire [127:0] head_s16_data;
	wire head_busy, head_done, head_status_error;
	wire [9:0] head_debug_error_code;
	wire [31:0] head_uops, head_txns, head_words, head_s16_words;
	wire [3:0] head_descriptor_slot;
	f14_head_three_runtime_v1 #(.C_PARTITION_RESET(C_PARTITION_RESET)) u_heads (
		.i_clk(i_clk), .i_rst_n(C_PARTITION_RESET ? i_reset_partition_n[4] : i_rst_n), .i_clear(head_clear),
        .i_reset_partition_n(i_reset_partition_n),
		.i_start(head_start), .o_start_ready(head_start_ready),
		.o_feature_rd_en(head_feature_rd_en), .o_feature_rd_addr(head_feature_rd_addr),
		.o_feature_bank(head_feature_bank), .i_feature_rd_valid(head_feature_rd_valid),
		.i_feature_rd_data(head_feature_rd_data), .o_rom_rd_en(head_rom_rd_en),
		.o_rom_rd_addr(head_rom_rd_addr), .i_rom_rd_ready(head_rom_ready), .i_run_enable(i_head_enable), .i_rom_rd_valid(head_rom_rd_valid),
		.i_rom_rd_data(head_rom_rd_data), .o_word_valid(head_word_valid),
		.i_word_ready(1'b1), .o_word_bank(head_word_bank), .o_word_head(head_word_head),
		.o_word_dest_addr(head_word_addr), .o_word_data(head_word_data),
		.o_main_s16_valid(head_s16_valid), .i_main_s16_ready(1'b1),
		.o_main_s16_bank(head_s16_bank), .o_main_s16_head(head_s16_head),
		.o_main_s16_dest_addr(head_s16_addr), .o_main_s16_data(head_s16_data),
		.o_busy(head_busy), .o_done(head_done), .o_status_error(head_status_error),
		.o_debug_error_code(head_debug_error_code), .o_accepted_uops(head_uops),
		.o_accepted_transactions(head_txns), .o_emitted_words(head_words),
		.o_emitted_main_s16_words(head_s16_words), .o_descriptor_slot(head_descriptor_slot)
	);


    f14_pipeline_rom_arbiter u_rom_arb (
        .i_clk(i_clk),.i_rst_n(i_rst_n),
        .i_bb_valid(bb_rom_rd_en),.i_bb_addr(bb_rom_rd_addr),.o_bb_valid(bb_rom_rd_valid),.o_bb_data(bb_rom_rd_data),
        .i_head_valid(head_rom_rd_en),.i_head_addr(head_rom_rd_addr),.o_head_ready(head_rom_ready),
        .o_head_valid(head_rom_rd_valid),.o_head_data(head_rom_rd_data),
        .i_boot_valid(i_boot_rd_en),.i_boot_addr(i_boot_rd_addr),.o_boot_ready(o_boot_rd_ready),
        .o_boot_valid(o_boot_rd_valid),.o_boot_data(o_boot_rd_data));
    f14_explicit_drm_ram #(.C_ADDR_WIDTH(12),.C_DATA_WIDTH(128),.C_DEPTH(2352)) u_adapter_snapshot (
        .i_clk(i_clk),.i_wr_en(bb_wide_valid),.i_wr_addr(copy_address_q),.i_wr_data(bb_wide_data),
        .i_rd_en(feature_req_q && feature_bank_q==0),.i_rd_addr(feature_addr_q),.o_rd_data(adapter_data));
    reg [10:0] prev_s16_addr [0:2];
    reg prev_s16_valid [0:2];
    wire first_s16_word = head_s16_valid && head_s16_bank==3 && head_s16_head<=2 &&
        (!prev_s16_valid[head_s16_head] || head_s16_addr!=prev_s16_addr[head_s16_head]);
    always @(posedge i_clk) begin
        if(!i_rst_n || head_clear)begin
            prev_s16_valid[0]<=0;prev_s16_valid[1]<=0;prev_s16_valid[2]<=0;
        end else if(head_s16_valid && head_s16_bank==3 && head_s16_head<=2)begin
            prev_s16_valid[head_s16_head]<=1;
            prev_s16_addr[head_s16_head]<=head_s16_addr;
        end
    end
    f14_result_bram u_result_bram (
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_wr_en(first_s16_word),
        .i_head(head_s16_head),.i_wr_addr(head_s16_addr),.i_wr_data(head_s16_data),
        .i_error(error_q),.i_req(i_result_req && o_result_valid),.i_addr(i_result_addr),
        .o_valid(o_result_read_valid),.o_data(o_result_data));
    // 特征接口现在直接传packed word地址。地址/使能先寄存一拍，后接一拍BRAM。
    // 同一uop的最多2个读请求连续进入；bank在descriptor结束前保持稳定。
    wire scratch_rd_en_a = feature_req_q && feature_bank_q==1;
    wire scratch_rd_en_b = feature_req_q && feature_bank_q==2;
    wire [127:0] scratch_a_data, scratch_b_data;
    reg feature_rsp_q;
    reg [1:0] feature_rsp_bank_q;
    always@(posedge i_clk or negedge i_rst_n)begin
        if(!i_rst_n)begin
            feature_req_q<=0; feature_addr_q<=0; feature_bank_q<=0;
            feature_rsp_q<=0; feature_rsp_bank_q<=0;
        end else begin
            feature_req_q<=head_feature_rd_en;
            if(head_feature_rd_en)begin
                feature_addr_q<=head_feature_rd_addr[11:0]; feature_bank_q<=head_feature_bank;
            end
            feature_rsp_q<=feature_req_q;
            if(feature_req_q)begin feature_rsp_bank_q<=feature_bank_q; end
        end
    end
    f14_explicit_drm_ram #(.C_ADDR_WIDTH(11), .C_DATA_WIDTH(128), .C_DEPTH(SCRATCH_WORDS), .C_INIT_FILE("NONE")) u_scratch_a (
        .i_clk(i_clk), .i_wr_en(head_word_valid && (head_word_bank == 2'd1)),
        .i_wr_addr(head_word_addr), .i_wr_data(head_word_data),
        .i_rd_en(scratch_rd_en_a), .i_rd_addr(feature_addr_q[10:0]), .o_rd_data(scratch_a_data)
    );
    f14_explicit_drm_ram #(.C_ADDR_WIDTH(11), .C_DATA_WIDTH(128), .C_DEPTH(SCRATCH_WORDS), .C_INIT_FILE("NONE")) u_scratch_b (
        .i_clk(i_clk), .i_wr_en(head_word_valid && (head_word_bank == 2'd2)),
        .i_wr_addr(head_word_addr), .i_wr_data(head_word_data),
        .i_rd_en(scratch_rd_en_b), .i_rd_addr(feature_addr_q[10:0]), .o_rd_data(scratch_b_data)
    );
    assign head_feature_rd_valid = feature_rsp_q && 1'b1;
    assign head_feature_rd_data = (feature_rsp_bank_q==0) ? adapter_data
        : ((feature_rsp_bank_q==1) ? scratch_a_data : scratch_b_data);


    assign o_input_release=bb_stage_valid && bb_stage_index==0;
    always @(posedge i_clk) begin
        if(!i_rst_n) begin
            clock_q<=0;capture_q<=0;image_full_q<=0;image_owned_q<=0;adapter_owned_q<=0;error_q<=0;
            image_wr_addr_q<=0;bb_state_q<=0;head_state_q<=0;copy_count_q<=0;copy_address_q<=0;
            image_id_q<=0;image_start_q<=0;bb_id_q<=0;bb_start_q<=0;adapter_id_q<=0;adapter_start_q<=0;
            head_id_q<=0;head_start_q<=0;result_cycles_q<=0;
        end else begin
            clock_q<=clock_q+1;
            if(o_frame_accept) begin capture_q<=1;image_wr_addr_q<=0;image_id_q<=i_frame_id;image_start_q<=clock_q;end
            if(pixel_fire) begin
                if(image_wr_addr_q==37631)begin capture_q<=0;image_full_q<=1;end
                else image_wr_addr_q<=image_wr_addr_q+1'b1;
            end
            if(bb_frame_accept) begin
                image_full_q<=0;image_owned_q<=1;bb_state_q<=1;bb_id_q<=image_id_q;bb_start_q<=image_start_q;
            end
            // stem之后不再读输入；新帧可写同一BRAM，摄像头接入仍须尊重ready。
            if(bb_stage_valid && bb_stage_index==0) image_owned_q<=0;
            if(bb_state_q==1 && bb_done) bb_state_q<=2;
            if(bb_state_q==2 && !adapter_owned_q && !error_q) begin bb_state_q<=3;copy_count_q<=0;end
            if(copy_req) begin copy_address_q<=copy_count_q;copy_count_q<=copy_count_q+1'b1;end
            if(copy_last) begin
                bb_state_q<=0;adapter_owned_q<=1;adapter_id_q<=bb_id_q;adapter_start_q<=bb_start_q;
            end
            case(head_state_q)
                0: if(adapter_owned_q && !error_q)begin head_state_q<=1;head_id_q<=adapter_id_q;head_start_q<=adapter_start_q;end
                1: head_state_q<=2;
                2: if(head_start) head_state_q<=3;
                3: if(head_done) begin
                    head_state_q<=4;adapter_owned_q<=0;result_cycles_q<=clock_q-head_start_q;
                end
                4: if(i_result_release) head_state_q<=0;
                default:begin head_state_q<=0;error_q<=1;end
            endcase
            if(bb_error_code!=0 || bb_probe_error || head_status_error) error_q<=1;
            if(bb_wide_valid && (adapter_owned_q || copy_address_q>=2352)) error_q<=1;
        end
    end
endmodule
`default_nettype wire
