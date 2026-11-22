//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 完整 AVM 帧渲染数据通路，将 DDR LUT burst 解包为 96-bit entry，经 64KiB 官方源缓存采集八邻域，再经 golden 算术 lane 融合并打包输出 DDR 帧。
// Module Name: avm_frame_renderer
// File Name: source/avm/avm_frame_renderer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 完整 AVM 帧渲染数据通路，将 DDR LUT burst 解包为 96-bit entry，经 64KiB 官方源缓存采集八邻域，再经 golden 算术 lane 融合并打包输出 DDR 帧。
// busy 期间冻结源槽、epoch 和输出描述符；cancel 后继续排空已接受的 AR 返回及 writer，再结束事务，不发布半帧输出。
//
// Dependencies: avm_lut_stream_unpack、avm_neighbor_gather、avm_source_cache、avm_output_word_pack、ddr_read_burst_engine、hdmi_ddr_read_arbiter。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-13
// Module Name: avm_frame_renderer
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 真实LUT DDR burst→64KiB官方cache→golden lane→完整DDR输出。
// Contract: source/output descriptor在busy期间冻结；cancel排空已接受AR与writer。
//////////////////////////////////////////////////////////////////////////////////
module avm_frame_renderer #(
	parameter [27:0] C_LUT_BASE=28'h0300000,
	parameter [1:0] C_SOURCE_SLOT0=2'd0,
	parameter [1:0] C_SOURCE_SLOT1=2'd1,
	parameter [1:0] C_SOURCE_SLOT2=2'd2,
	parameter [1:0] C_SOURCE_SLOT3=2'd3
)(
	input i_clk,
	input i_rst_n,
	input i_start,
	input i_cancel,
	input [111:0] i_source_base,
	input [63:0] i_source_epoch,
	input [3:0] i_source_present,
	input [27:0] i_output_base,
	output o_ready,
	output o_busy,
	output o_done,
	output o_aborted,
	output [31:0] o_render_cycles,
	output [31:0] o_cache_hits,
	output [31:0] o_cache_misses,
	output [27:0] o_axi_araddr,
	output [3:0] o_axi_arlen,
	output o_axi_arvalid,
	input i_axi_arready,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	output o_write_valid,
	output [27:0] o_write_address,
	output [255:0] o_write_data,
	input i_write_ready,
	input i_write_drained
);
	localparam [2:0] ST_IDLE=0,ST_INVALIDATE=1,ST_WAIT_CLEAR=2,ST_START=3,ST_RUN=4,ST_FINISH=5,ST_CANCEL=6;
	reg [2:0] state_current,state_next;
	reg [17:0] lut_words_done;
	reg [31:0] render_cycles_o;
	wire rendering=state_current==ST_RUN;
	wire aborting=state_current==ST_CANCEL;
	wire cache_initialized,cache_idle,cache_req,cache_req_ready,cache_resp,cache_resp_ready,cache_resp_error;
	wire [1:0] cache_source;
	wire [17:0] cache_addr;
	wire [15:0] cache_epoch,cache_rgb;
	wire [27:0] cache_base,cache_araddr,lut_araddr;
	wire [3:0] cache_arlen,lut_arlen;
	wire cache_arvalid,lut_arvalid;
	wire [2:0] client_ready,client_rvalid;
	wire [255:0] client_rdata;
	wire lut_engine_busy,lut_engine_done,lut_engine_failed,lut_word_valid,lut_word_ready;
	wire [255:0] lut_word_data;
	wire lut_valid,lut_ready,lut_frame_done;
	wire [95:0] lut_data;
	wire [18:0] lut_index;
	wire pixel_valid,pixel_ready,pixel_error,gather_idle;
	wire [15:0] pixel_data;
	wire [18:0] pixel_index;
	wire packed_valid,packed_done,packed_error;
	wire [255:0] packed_data;
	wire [14:0] packed_index;
	wire all_drained=cache_idle && !lut_engine_busy && i_write_drained && !packed_valid;
	assign o_ready=state_current==ST_IDLE && cache_initialized && all_drained;
	assign o_busy=state_current!=ST_IDLE;
	assign o_done=state_current==ST_FINISH && all_drained;
	assign o_aborted=state_current==ST_CANCEL && all_drained;
	assign o_render_cycles=render_cycles_o;
	assign o_write_valid=packed_valid && (rendering || aborting);
	assign o_write_data=packed_data;
	assign o_write_address=i_output_base+{10'd0,packed_index,3'd0};
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_current<=ST_IDLE;
		else state_current<=state_next;
	end
	always @(*) begin
		state_next=state_current;
		case(state_current)
			ST_IDLE: if(i_start && o_ready) state_next=ST_INVALIDATE;
			ST_INVALIDATE: state_next=ST_WAIT_CLEAR;
			ST_WAIT_CLEAR: if(cache_initialized) state_next=ST_START;
			ST_START: state_next=ST_RUN;
			ST_RUN: if(packed_done) state_next=ST_FINISH;
			ST_FINISH: if(all_drained) state_next=ST_IDLE;
			ST_CANCEL: if(all_drained) state_next=ST_IDLE;
			default: state_next=ST_CANCEL;
		endcase
		if(i_cancel && state_current!=ST_IDLE) state_next=ST_CANCEL;
		if(rendering && (packed_error || lut_engine_failed)) state_next=ST_CANCEL;
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin lut_words_done<=0; render_cycles_o<=0; end
		else begin
			if(i_start && o_ready) render_cycles_o<=0;
			else if(o_busy) render_cycles_o<=render_cycles_o+1'b1;
			if(state_current==ST_START) lut_words_done<=0;
			else if(lut_engine_done && rendering) lut_words_done<=lut_words_done+18'd16;
		end
	end
	avm_source_cache u_cache(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_cancel(aborting),.i_invalidate(state_current==ST_INVALIDATE),
		.i_request_valid(cache_req && rendering),.o_request_ready(cache_req_ready),.i_request_source(cache_source),
		.i_request_pixel_addr(cache_addr),.i_request_frame_epoch(cache_epoch),.i_request_source_base(cache_base),
		.o_response_valid(cache_resp),.i_response_ready(rendering ? cache_resp_ready : 1'b1),.o_response_rgb565(cache_rgb),.o_response_error(cache_resp_error),
		.o_initialized(cache_initialized),.o_idle(cache_idle),.o_axi_araddr(cache_araddr),.o_axi_arlen(cache_arlen),.o_axi_arvalid(cache_arvalid),
		.i_axi_arready(client_ready[0]),.i_axi_rdata(client_rdata),.i_axi_rvalid(client_rvalid[0]),.o_hit_count(o_cache_hits),.o_miss_count(o_cache_misses));
	ddr_read_burst_engine #(.C_WRITE_ADDR_HALF_CYCLE(1)) u_lut_engine(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_abort(!rendering),
		.i_burst_req(rendering && !lut_engine_busy && !lut_engine_done && lut_words_done<153600),
		.i_start_addr(C_LUT_BASE + {7'd0,lut_words_done,3'd0}),.i_burst_len(4'd15),.i_output_capacity(16'd16),
		.o_busy(lut_engine_busy),.o_burst_done(lut_engine_done),.o_burst_failed(lut_engine_failed),
		.o_axi_araddr(lut_araddr),.o_axi_arlen(lut_arlen),.o_axi_arvalid(lut_arvalid),
		.i_axi_arready(client_ready[1]),.i_axi_rdata(client_rdata),.i_axi_rvalid(client_rvalid[1]),
		.o_word_valid(lut_word_valid),.o_word_data(lut_word_data),.i_word_ready(lut_word_ready && rendering));
	hdmi_ddr_read_arbiter u_local_read_arbiter(.i_clk(i_clk),.i_rst_n(i_rst_n),
		.i_client_araddr({28'd0,lut_araddr,cache_araddr}),.i_client_arlen({4'd0,lut_arlen,cache_arlen}),
		.i_client_arvalid({1'b0,lut_arvalid,cache_arvalid}),.i_display_urgent(2'b01),.i_avm_budget_ok(1'b0),
		.o_client_arready(client_ready),.o_client_rvalid(client_rvalid),.o_client_rdata(client_rdata),
		.o_axi_araddr(o_axi_araddr),.o_axi_arlen(o_axi_arlen),.o_axi_arvalid(o_axi_arvalid),
		.i_axi_arready(i_axi_arready),.i_axi_rdata(i_axi_rdata),.i_axi_rvalid(i_axi_rvalid),.o_busy());
	avm_lut_stream_unpack u_lut_unpack(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_frame_start(state_current==ST_START),
		.i_word_valid(lut_word_valid && rendering),.o_word_ready(lut_word_ready),.i_word_data(lut_word_data),
		.o_lut_valid(lut_valid),.i_lut_ready(lut_ready && rendering),.o_lut(lut_data),.o_lut_index(lut_index),.o_frame_done(lut_frame_done));
	avm_neighbor_gather #(
		.C_SOURCE_SLOT0(C_SOURCE_SLOT0),
		.C_SOURCE_SLOT1(C_SOURCE_SLOT1),
		.C_SOURCE_SLOT2(C_SOURCE_SLOT2),
		.C_SOURCE_SLOT3(C_SOURCE_SLOT3)
	) u_gather(.i_clk(i_clk),.i_rst_n(i_rst_n && (state_current==ST_START || rendering)),
		.i_lut_valid(lut_valid && rendering),.o_lut_ready(lut_ready),.i_lut(lut_data),.i_lut_index(lut_index),
		.i_source_base(i_source_base),.i_source_epoch(i_source_epoch),.i_source_present(i_source_present),
		.o_cache_request_valid(cache_req),.i_cache_request_ready(cache_req_ready),.o_cache_source(cache_source),.o_cache_pixel_addr(cache_addr),
		.o_cache_frame_epoch(cache_epoch),.o_cache_source_base(cache_base),.i_cache_response_valid(cache_resp),.o_cache_response_ready(cache_resp_ready),
		.i_cache_response_rgb565(cache_rgb),.i_cache_response_error(cache_resp_error),.o_pixel_valid(pixel_valid),.i_pixel_ready(pixel_ready),
		.o_pixel_rgb565(pixel_data),.o_pixel_index(pixel_index),.o_pixel_error(pixel_error),.o_idle(gather_idle));
	avm_output_word_pack u_pack(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_frame_start(state_current==ST_START),
		.i_pixel_valid(pixel_valid && rendering),.o_pixel_ready(pixel_ready),.i_pixel_rgb565(pixel_data),.i_pixel_index(pixel_index),.i_pixel_error(pixel_error),
		.o_word_valid(packed_valid),.i_word_ready(i_write_ready && (rendering || aborting)),.o_word_data(packed_data),
		.o_word_index(packed_index),.o_words_done(packed_done),.o_frame_error(packed_error));
endmodule

