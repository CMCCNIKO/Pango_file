//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: AVM 渲染与显示总控制，锁存四源 lease/epoch，驱动完整帧 renderer，将成功结果发布到三槽输出缓冲，像素端用官方 FIFO 连续显示。
// Module Name: avm_video_subsystem
// File Name: source/avm/avm_video_subsystem.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// AVM 渲染与显示总控制，锁存四源 lease/epoch，驱动完整帧 renderer，将成功结果发布到三槽输出缓冲，像素端用官方 FIFO 连续显示。
// 已接受的 AR/W 均排空后才释放源 lease，显示槽不交给 renderer 覆写；超时或取消不发布半帧。
// C_RENDER_DEADLINE 默认 8333332 个 DDR 时钟周期，在 125MHz 下约 66.7ms；具体渲染吞吐以实际验证为准。
//
// Dependencies: avm_frame_renderer、avm_ddr_to_hdmi_fifo_wrapper、ddr_read_burst_engine、hdmi_ddr_read_arbiter、源槽 lease 接口。
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
// Module Name: avm_video_subsystem
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 30fps真实DDR渲染、三槽完整输出发布、60Hz官方FIFO显示。
// Contract: 已接受AR/W排空后才释放源引用；显示槽绝不被renderer覆写。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module avm_video_subsystem #(
	// 125 MHz DDR域下允许一次30 fps渲染跨越一个60 Hz显示周期；
	// 独立仿真约25.7 ms，主工程共享DDR时保留到66.7 ms避免竞争误取消。
	parameter integer C_RENDER_DEADLINE=8333332,
	parameter integer C_PREFETCH_WORDS=128,
	parameter [1:0] C_SOURCE_SLOT0=2'd0,
	parameter [1:0] C_SOURCE_SLOT1=2'd1,
	parameter [1:0] C_SOURCE_SLOT2=2'd2,
	parameter [1:0] C_SOURCE_SLOT3=2'd3
)(
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_ddr_init_done,
	input i_lut_ready,
	input i_lut_update_pending,
	input [7:0] i_display_slot,
	input [3:0] i_display_present,
	input [63:0] i_display_frame_epoch,
	input [3:0] i_lease_ready,
	output [3:0] o_lease_acquire,
	output [3:0] o_lease_release,
	output [7:0] o_lease_slot,
	output [63:0] o_lease_frame_epoch,
	input [159:0] i_tag,
	input i_pixel_pop,
	output [15:0] o_pixel_rgb565,
	output o_pixel_valid,
	output o_pixel_frame_ready,
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
	input i_write_drained,
	output o_read_urgent,
	output o_render_busy,
	output o_output_valid,
	output o_render_done,
	output [31:0] o_render_drop_count,
	output [31:0] o_underflow_count,
	output [31:0] o_last_render_cycles,
	output o_debug_frame_toggle,
	output o_debug_renderer_ready,
	output o_debug_free_available,
	output o_debug_pixel_alive
);
	localparam [2:0] ST_DRAIN=0,ST_FLUSH=1,ST_WAIT_FLUSH=2,ST_SELECT=3,ST_RUN=4;
	reg [2:0] display_state;
	reg frame_toggle;
	reg [15:0] pixel_frame_id;
	reg [1:0] frame_sync /* synthesis syn_preserve = 1 */;
	reg frame_seen,frame_phase;
	reg [1:0] pixel_alive_sync /* synthesis syn_preserve = 1 */;
	reg [2:0] ready_slots;
	reg [1:0] newest_slot,display_slot,render_slot;
	reg display_valid,render_reserved,render_start;
	reg [7:0] render_source_slot;
	reg [63:0] render_source_epoch;
	reg [3:0] render_source_present;
	reg [111:0] render_source_base;
	reg [31:0] drop_count_o,last_render_cycles_o;
	reg [14:0] display_words;
	reg prepared_sent,prepared_toggle;
	reg [15:0] descriptor_frame_id,prepared_frame_hold;
	reg [1:0] prepared_sync /* synthesis syn_preserve = 1 */;
	reg prepared_seen,pixel_pending,pixel_frame_ready_o;
	reg [15:0] pixel_prepared_frame;
	wire frame_boundary=i_tag[157] && i_tag[154:144]==0 && i_tag[143:134]==0;
	wire top_black=i_tag[157] && i_tag[143:134]<40;
	wire frame_event=frame_sync[1]!=frame_seen;
	wire renderer_ready,renderer_busy,renderer_done,renderer_aborted;
	wire [31:0] render_cycles;
	wire [27:0] render_araddr,display_araddr;
	wire [3:0] render_arlen,display_arlen;
	wire render_arvalid,display_arvalid;
	wire [2:0] client_ready,client_rvalid;
	wire [255:0] client_rdata;
	wire display_engine_busy,display_engine_done,display_engine_failed;
	wire display_word_valid,fifo_full,flush_busy,flush_done;
	wire [255:0] display_word_data;
	wire [15:0] fifo_water;
	wire [15:0] fifo_rgb;
	wire fifo_pixel_valid;
	wire display_running=display_state==ST_RUN && display_valid && pixel_alive_sync[1];
	wire [2:0] free_mask=~ready_slots & ~(display_valid ? (3'b001<<display_slot) : 3'b000);
	wire free_available=|free_mask;
	wire [1:0] free_slot=free_mask[0] ? 2'd0 : free_mask[1] ? 2'd1 : 2'd2;
	wire sources_ready=(i_display_present & ~i_lease_ready)==0;
	wire launch_render=frame_event && !frame_phase && !render_reserved && renderer_ready &&
		i_ddr_init_done && i_lut_ready && !i_lut_update_pending && sources_ready && free_available && pixel_alive_sync[1];
	wire finish_render=render_reserved && (renderer_done || renderer_aborted);
	wire cancel_render=render_reserved && (render_cycles>=C_RENDER_DEADLINE || i_lut_update_pending || !pixel_alive_sync[1]);
	wire display_push=display_word_valid && display_running && !fifo_full;
	localparam integer C_DISPLAY_WORDS=19200;
	wire display_request=display_running && !display_engine_busy && !display_engine_done && display_words<C_DISPLAY_WORDS && fifo_water<=496;
	// LUT occupies 0x0300000..0x042BFFF for the 640x640 package.
	// Keep all three rendered output slots above that range.
	wire [27:0] display_base=28'h0500000+28'h0040000*display_slot;
	wire [27:0] render_base=28'h0500000+28'h0040000*render_slot;
	assign o_lease_acquire=launch_render ? i_display_present : 4'd0;
	assign o_lease_release=finish_render ? render_source_present : 4'd0;
	// Idle时先发布待申请的当前完整帧描述符，使lease ready能够形成闭环；
	// render启动后再冻结同一描述符，避免launch_render与ready组合反馈。
	assign o_lease_slot=!render_reserved ? i_display_slot : render_source_slot;
	assign o_lease_frame_epoch=!render_reserved ? i_display_frame_epoch : render_source_epoch;
	assign o_read_urgent=display_running && display_words<C_DISPLAY_WORDS && fifo_water<C_PREFETCH_WORDS;
	assign o_render_busy=render_reserved || renderer_busy;
	assign o_output_valid=display_valid || (|ready_slots);
	assign o_render_done=render_reserved && renderer_done;
	assign o_render_drop_count=drop_count_o;
	assign o_last_render_cycles=last_render_cycles_o;
	assign o_pixel_frame_ready=pixel_frame_ready_o;
	// 调试状态保持为电平，供系统UART周期采样；frame_toggle代表已观察到的帧边界。
	assign o_debug_frame_toggle=frame_sync[1];
	assign o_debug_renderer_ready=renderer_ready;
	assign o_debug_free_available=free_available;
	assign o_debug_pixel_alive=pixel_alive_sync[1];
	assign o_pixel_valid=pixel_frame_ready_o && fifo_pixel_valid;
	assign o_pixel_rgb565=o_pixel_valid ? fifo_rgb : 16'd0;
	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if(!i_pixel_rst_n) begin
			frame_toggle<=0;pixel_frame_id<=0;prepared_sync<=0;prepared_seen<=0;
			pixel_pending<=0;pixel_prepared_frame<=0;pixel_frame_ready_o<=0;
		end else begin
			prepared_sync<={prepared_sync[0],prepared_toggle};
			if(frame_boundary) begin frame_toggle<=!frame_toggle;pixel_frame_id<=pixel_frame_id+1'b1;pixel_pending<=0;pixel_frame_ready_o<=0;end
			if(prepared_sync[1]!=prepared_seen) begin prepared_seen<=prepared_sync[1];pixel_prepared_frame<=prepared_frame_hold;pixel_pending<=1;end
			// prepared_toggle已经跨域同步，且只在ST_RUN预取完成后产生；这里在
			//下一次顶部黑边统一接受，避免DDR域直接采样pixel_frame_id造成一帧
			//竞态时把合法prepared令牌永久丢弃。frame_id仍保留用于观测。
			if(!frame_boundary && top_black && pixel_pending && fifo_pixel_valid) begin
				pixel_frame_ready_o<=1;pixel_pending<=0;
			end
		end
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) begin frame_sync<=0;frame_seen<=0;frame_phase<=0;pixel_alive_sync<=0;end
		else begin
			frame_sync<={frame_sync[0],frame_toggle};pixel_alive_sync<={pixel_alive_sync[0],i_pixel_rst_n};
			if(frame_event) begin frame_seen<=frame_sync[1];frame_phase<=!frame_phase;end
		end
	end
	// 完成输出才发布READY；换显示前先排空旧burst并跨域flush旧像素。
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) begin
			ready_slots<=0;newest_slot<=0;display_slot<=0;render_slot<=0;display_valid<=0;
			render_reserved<=0;render_start<=0;render_source_slot<=0;render_source_epoch<=0;
			render_source_present<=0;render_source_base<=0;drop_count_o<=0;last_render_cycles_o<=0;
		end else begin
			render_start<=0;
			if(display_state==ST_SELECT && |ready_slots) begin
				display_slot<=newest_slot;display_valid<=1;ready_slots<=ready_slots & ~(3'b001<<newest_slot);
			end
			if(launch_render) begin
				render_reserved<=1;render_start<=1;render_slot<=free_slot;
				render_source_slot<=i_display_slot;render_source_epoch<=i_display_frame_epoch;render_source_present<=i_display_present;
				render_source_base[27:0]<=28'h0040000*i_display_slot[1:0];
				render_source_base[55:28]<=28'h00c0000+28'h0040000*i_display_slot[3:2];
				render_source_base[83:56]<=28'h0180000+28'h0040000*i_display_slot[5:4];
				render_source_base[111:84]<=28'h0240000+28'h0040000*i_display_slot[7:6];
			end
			if(finish_render) begin
				render_reserved<=0;last_render_cycles_o<=render_cycles;
				if(renderer_done) begin ready_slots<=3'b001<<render_slot;newest_slot<=render_slot;end
				else drop_count_o<=drop_count_o+1'b1;
			end
		end
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) begin
			display_state<=ST_DRAIN;display_words<=0;descriptor_frame_id<=0;
			prepared_toggle<=0;prepared_sent<=0;prepared_frame_hold<=0;
		end else begin
			case(display_state)
				ST_DRAIN: if(!display_engine_busy && !flush_busy) display_state<=ST_FLUSH;
				ST_FLUSH: display_state<=ST_WAIT_FLUSH;
				ST_WAIT_FLUSH: if(flush_done) display_state<=ST_SELECT;
				ST_SELECT: begin display_state<=ST_RUN;display_words<=0;descriptor_frame_id<=pixel_frame_id;end
				ST_RUN: begin
					if(display_engine_done) display_words<=display_words+15'd16;
					if(frame_event || !pixel_alive_sync[1]) display_state<=ST_DRAIN;
				end
				default:display_state<=ST_DRAIN;
			endcase
			if(display_state!=ST_RUN) prepared_sent<=0;
			else if(!prepared_sent && fifo_water>=C_PREFETCH_WORDS && display_valid && pixel_alive_sync[1]) begin
				prepared_frame_hold<=descriptor_frame_id;prepared_toggle<=!prepared_toggle;prepared_sent<=1;
			end
		end
	end
	avm_frame_renderer #(
		.C_SOURCE_SLOT0(C_SOURCE_SLOT0),
		.C_SOURCE_SLOT1(C_SOURCE_SLOT1),
		.C_SOURCE_SLOT2(C_SOURCE_SLOT2),
		.C_SOURCE_SLOT3(C_SOURCE_SLOT3)
	) u_renderer(.i_clk(i_ddr_clk),.i_rst_n(i_ddr_rst_n),.i_start(render_start),.i_cancel(cancel_render),
		.i_source_base(render_source_base),.i_source_epoch(render_source_epoch),.i_source_present(render_source_present),.i_output_base(render_base),
		.o_ready(renderer_ready),.o_busy(renderer_busy),.o_done(renderer_done),.o_aborted(renderer_aborted),.o_render_cycles(render_cycles),.o_cache_hits(),.o_cache_misses(),
		.o_axi_araddr(render_araddr),.o_axi_arlen(render_arlen),.o_axi_arvalid(render_arvalid),.i_axi_arready(client_ready[1]),.i_axi_rdata(client_rdata),.i_axi_rvalid(client_rvalid[1]),
		.o_write_valid(o_write_valid),.o_write_address(o_write_address),.o_write_data(o_write_data),.i_write_ready(i_write_ready),.i_write_drained(i_write_drained));
	ddr_read_burst_engine u_display_engine(.i_clk(i_ddr_clk),.i_rst_n(i_ddr_rst_n),.i_abort(!display_running),
		.i_burst_req(display_request),.i_start_addr(display_base+{10'd0,display_words,3'd0}),.i_burst_len(4'd15),.i_output_capacity(16'd512-fifo_water),
		.o_busy(display_engine_busy),.o_burst_done(display_engine_done),.o_burst_failed(display_engine_failed),
		.o_axi_araddr(display_araddr),.o_axi_arlen(display_arlen),.o_axi_arvalid(display_arvalid),.i_axi_arready(client_ready[0]),.i_axi_rdata(client_rdata),.i_axi_rvalid(client_rvalid[0]),
		.o_word_valid(display_word_valid),.o_word_data(display_word_data),.i_word_ready(display_running && !fifo_full));
	hdmi_ddr_read_arbiter u_local_read_arbiter(.i_clk(i_ddr_clk),.i_rst_n(i_ddr_rst_n),
		.i_client_araddr({28'd0,render_araddr,display_araddr}),.i_client_arlen({4'd0,render_arlen,display_arlen}),.i_client_arvalid({1'b0,render_arvalid,display_arvalid}),
		.i_display_urgent({1'b0,o_read_urgent}),.i_avm_budget_ok(1'b0),.o_client_arready(client_ready),.o_client_rvalid(client_rvalid),.o_client_rdata(client_rdata),
		.o_axi_araddr(o_axi_araddr),.o_axi_arlen(o_axi_arlen),.o_axi_arvalid(o_axi_arvalid),.i_axi_arready(i_axi_arready),.i_axi_rdata(i_axi_rdata),.i_axi_rvalid(i_axi_rvalid),.o_busy());
	avm_ddr_to_hdmi_fifo_wrapper u_fifo(.i_ddr_clk(i_ddr_clk),.i_ddr_rst_n(i_ddr_rst_n),.i_frame_switch(display_state==ST_FLUSH),
		.o_flush_busy(flush_busy),.o_flush_done(flush_done),.i_word_push(display_push),.i_word_data(display_word_data),.o_word_full(fifo_full),
		.i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(i_pixel_rst_n),.i_measure_enable(display_running),.i_pixel_pop(i_pixel_pop && pixel_frame_ready_o),
		.o_pixel_rgb565(fifo_rgb),.o_pixel_valid(fifo_pixel_valid),.o_underflow_count(o_underflow_count),.o_underflow_event(),.o_wr_water_level(fifo_water),.o_rd_water_level());
endmodule

