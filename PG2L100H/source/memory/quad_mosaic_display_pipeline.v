//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-15
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四宫格显示读流水，按 drain、双域 FIFO flush、显示帧槽快照、预取和新帧启动的顺序协调 DDR 与 HDMI。
// Module Name: quad_mosaic_display_pipeline
// File Name: source/memory/quad_mosaic_display_pipeline.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四宫格显示读流水，按 drain、双域 FIFO flush、显示帧槽快照、预取和新帧启动的顺序协调 DDR 与 HDMI。
// 发出 HMIC burst 读请求、输出 256-bit 到像素 FIFO，并为四宫格内容提供匹配输出帧 ID 的源槽/epoch 标签。
//
// Dependencies: quad_mosaic_read_scheduler、ddr_read_burst_engine、quad_ddr_to_hdmi_fifo_wrapper、quad_frame_content_cdc、c6_event_sync。
//
// Revision:
// 2026-11-16 - 统计侧带叠加真实显示提交新鲜度，保留旧帧读取与显示。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 此模块是 quad 读客户端；全工程还有 monitor 与 AVM 读客户端，不能把原头“唯一 HMIC reader”解释为整个工程只有它读取 DDR。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-09-15
// Module Name: quad_mosaic_display_pipeline
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 唯一 HMIC reader；drain、FIFO双域清空、快照、启动。
// Dependencies: quad_mosaic_read_scheduler, ddr_read_burst_engine,
//               quad_ddr_to_hdmi_fifo_wrapper, c6_event_sync
// Contract: 125MHz DDR / 74.25MHz HDMI；100000 DDR拍小于顶部40行黑边。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module quad_mosaic_display_pipeline #(
	parameter integer C_START_BUDGET_CYCLES = 100000
)(
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_ddr_init_done,
	input i_frame_boundary,
	input [7:0] i_display_slot,
	input [3:0] i_display_valid,
	input [3:0] i_source_fresh,
	output o_snapshot,
	output [3:0] o_retire_done,
	output [27:0] o_axi_araddr,
	output [3:0] o_axi_arlen,
	output o_axi_arvalid,
	input i_axi_arready,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_pixel_pop,
	output [15:0] o_pixel_rgb565,
	output o_pixel_valid,
	output [31:0] o_underflow_count,
	output o_underflow_event,
	output [15:0] o_wr_water_level,
	output [15:0] o_rd_water_level,
	output o_read_urgent,
	input i_pixel_frame_boundary,
	output [3:0] o_pixel_source_present,
	output o_pixel_source_present_valid
);
	`include "quad_frame_address_map.vh"
	localparam [3:0] ST_WAIT_FRAME=0, ST_DRAIN=1, ST_FLUSH=2, ST_WAIT_FLUSH=3;
	localparam [3:0] ST_SNAPSHOT=4, ST_SETTLE=5, ST_START=6, ST_RUN=7;
	reg [3:0] state_current;
	reg [3:0] state_next;
	reg [16:0] cnt_start_budget;
	reg flag_scheduler_run;
	wire [27:0] source0_base, source1_base, source2_base, source3_base;
	wire [15:0] free_words;
	wire fifo_full, fifo_flush_busy, fifo_flush_done;
	wire engine_busy, engine_done, engine_failed, engine_word_valid, engine_word_ready;
	wire [255:0] engine_word_data;
	wire burst_req;
	wire [27:0] burst_addr;
	wire [3:0] burst_len;
	wire mosaic_valid, mosaic_ready, scheduler_error, scheduler_urgent, underflow_ddr, run_active;
	wire [255:0] mosaic_data;
	wire [3:0] scheduler_source_present;
	wire [3:0] statistics_source_present;
	// 仅收紧统计侧带；scheduler仍读取保留的DISPLAYING旧帧。
	// 输入是DDR源编号0/1/2/3，CDC侧带是显示tile顺序0/3/1/2。
	assign statistics_source_present = scheduler_source_present &
		{i_source_fresh[2],i_source_fresh[1],i_source_fresh[3],i_source_fresh[0]};
	assign source0_base = `QUAD_SRC0_BASE + (i_display_slot[1:0] * `QUAD_SLOT_STRIDE);
	assign source1_base = `QUAD_SRC1_BASE + (i_display_slot[3:2] * `QUAD_SLOT_STRIDE);
	assign source2_base = `QUAD_SRC2_BASE + (i_display_slot[5:4] * `QUAD_SLOT_STRIDE);
	assign source3_base = `QUAD_SRC3_BASE + (i_display_slot[7:6] * `QUAD_SLOT_STRIDE);
	assign run_active = (state_current == ST_RUN);
	assign free_words = (o_wr_water_level < 16'd512) ? (16'd512-o_wr_water_level) : 16'd0;
	assign mosaic_ready = run_active && !fifo_full;
	assign o_snapshot = (state_current == ST_SNAPSHOT);
	// 旧AR已排空且旧FIFO/serializer已清空，才释放RETIRING。
	assign o_retire_done = {4{state_current == ST_SETTLE}};
	assign o_read_urgent = run_active && scheduler_urgent;
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin cnt_start_budget <= 0; end
		else if (i_frame_boundary) begin cnt_start_budget <= C_START_BUDGET_CYCLES; end
		else if (cnt_start_budget != 0) begin cnt_start_budget <= cnt_start_budget-1'b1; end
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin state_current <= ST_WAIT_FRAME; end
		else begin state_current <= state_next; end
	end
	always @(*) begin
		state_next = state_current;
		case (state_current)
			ST_WAIT_FRAME: begin
				if (i_frame_boundary && i_ddr_init_done) begin state_next = ST_DRAIN; end
			end
			ST_DRAIN: begin
				if (!engine_busy && !fifo_flush_busy) begin state_next = ST_FLUSH; end
			end
			ST_FLUSH: begin state_next = ST_WAIT_FLUSH; end
			ST_WAIT_FLUSH: begin
				if (fifo_flush_done) begin state_next = ST_SNAPSHOT; end
			end
			ST_SNAPSHOT: begin state_next = ST_SETTLE; end
			ST_SETTLE: begin state_next = ST_START; end
			ST_START: begin state_next = ST_RUN; end
			ST_RUN: begin
				if (i_frame_boundary) begin state_next = ST_DRAIN; end
				else if (underflow_ddr || scheduler_error) begin state_next = ST_WAIT_FRAME; end
			end
			default: begin state_next = ST_WAIT_FRAME; end
		endcase
		// 长时间HMIC停顿后不从有效行中部启动，等下一输出帧边界。
		if ((state_current != ST_RUN) && (state_current != ST_WAIT_FRAME) &&
			(cnt_start_budget == 0)) begin state_next = ST_WAIT_FRAME; end
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin flag_scheduler_run <= 0; end
		else begin flag_scheduler_run <= (state_next == ST_START) || (state_next == ST_RUN); end
	end
	c6_event_sync u_underflow_sync(
		.i_src_clk(i_pixel_clk), .i_src_rst_n(i_pixel_rst_n), .i_src_event(o_underflow_event),
		.i_dst_clk(i_ddr_clk), .i_dst_rst_n(i_ddr_rst_n), .o_dst_event(underflow_ddr)
	);
	quad_mosaic_read_scheduler u_scheduler(
		.i_clk(i_ddr_clk), .i_rstn(flag_scheduler_run), .i_frame_start(state_current == ST_START),
		// scheduler索引是显示位置，只在此处将源编号重排为TL/TR/BL/BR。
		.i_display_base({source2_base,source1_base,source3_base,source0_base}),
		.i_display_valid({i_display_valid[2],i_display_valid[1],i_display_valid[3],i_display_valid[0]}),
		.i_fifo_free_words(free_words), .i_fifo_used_words(o_wr_water_level), .i_output_ready(mosaic_ready),
		.i_read_engine_busy(engine_busy), .i_read_burst_done(engine_done), .i_read_burst_failed(engine_failed),
		.i_read_word_valid(engine_word_valid), .i_read_word_data(engine_word_data), .o_read_word_ready(engine_word_ready),
		.o_read_burst_req(burst_req), .o_read_burst_addr(burst_addr), .o_read_burst_len(burst_len),
		.o_output_word_valid(mosaic_valid), .o_output_word_data(mosaic_data), .o_busy(), .o_frame_done(),
		.o_frame_overrun(), .o_frame_error(scheduler_error), .o_flush_output_fifo(), .o_read_urgent(scheduler_urgent),
		.o_display_valid_snapshot(scheduler_source_present)
	);
	ddr_read_burst_engine u_engine(
		.i_clk(i_ddr_clk), .i_rst_n(i_ddr_rst_n), .i_abort(!run_active),
		.i_burst_req(burst_req && run_active), .i_start_addr(burst_addr), .i_burst_len(burst_len),
		.i_output_capacity(free_words), .o_busy(engine_busy), .o_burst_done(engine_done), .o_burst_failed(engine_failed),
		.o_axi_araddr(o_axi_araddr), .o_axi_arlen(o_axi_arlen), .o_axi_arvalid(o_axi_arvalid),
		.i_axi_arready(i_axi_arready), .i_axi_rdata(i_axi_rdata), .i_axi_rvalid(i_axi_rvalid),
		.o_word_valid(engine_word_valid), .o_word_data(engine_word_data), .i_word_ready(engine_word_ready)
	);
	quad_ddr_to_hdmi_fifo_wrapper u_fifo(
		.i_ddr_clk(i_ddr_clk), .i_ddr_rst_n(i_ddr_rst_n), .i_frame_switch(state_current == ST_FLUSH),
		.o_flush_busy(fifo_flush_busy), .o_flush_done(fifo_flush_done),
		.i_word_push(mosaic_valid && mosaic_ready), .i_word_data(mosaic_data), .o_word_full(fifo_full),
		.i_pixel_clk(i_pixel_clk), .i_pixel_rst_n(i_pixel_rst_n), .i_measure_enable(run_active),
		.i_pixel_pop(i_pixel_pop), .o_pixel_rgb565(o_pixel_rgb565), .o_pixel_valid(o_pixel_valid),
		.o_underflow_count(o_underflow_count), .o_underflow_event(o_underflow_event),
		.o_wr_water_level(o_wr_water_level), .o_rd_water_level(o_rd_water_level)
	);
	// ST_RUN首拍scheduler已经捕获ST_START快照；与该帧ID一起发送侧带。
	reg reg_run_previous;
	always@(posedge i_ddr_clk or negedge i_ddr_rst_n)begin
		if(!i_ddr_rst_n)begin reg_run_previous <= 1'b0; end
		else begin reg_run_previous <= run_active; end
	end
	quad_frame_content_cdc quad_frame_content_cdc_Inst_source_presence(
		.i_pixel_clk(i_pixel_clk),
		.i_pixel_rstn(i_pixel_rst_n),
		.i_pixel_frame_boundary(i_pixel_frame_boundary),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rstn(i_ddr_rst_n),
		.i_snapshot(run_active && !reg_run_previous),
		.i_source_present(statistics_source_present),
		.o_source_present(o_pixel_source_present),
		.o_source_present_valid(o_pixel_source_present_valid)
	);
endmodule
