//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: monitor 独立 DDR 读取/官方 FIFO/像素流水，每帧执行旧请求 drain、FIFO flush、源 lease、冻结路由快照与预取。
// Module Name: monitor_display_pipeline
// File Name: source/memory/monitor_display_pipeline.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// monitor 独立 DDR 读取/官方 FIFO/像素流水，每帧执行旧请求 drain、FIFO flush、源 lease、冻结路由快照与预取。
// prepared 与同一输出 frame token 绑定，只有 compositor 实际锁存回传后才报告 applied；已呈现 AR 保持到接受并排空，flush 完成后才释放 lease。
//
// Dependencies: monitor_read_scheduler、ddr_read_burst_engine、monitor_ddr_to_hdmi_fifo_wrapper、monitor_ddr_to_hdmi_fifo 官方 IP。
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
// Module Name: monitor_display_pipeline
// Project Name: PG2L100H HDMI AVM Monitor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 独立monitor真实DDR→官方FIFO→像素；每帧drain/flush/lease/prefetch。
// Contract: prepared仅属于同一输出frame token；真实compositor锁存回传才applied。
// Dependencies: monitor_ddr_to_hdmi_fifo official IP, ddr_read_burst_engine。
// Revision: 2026-10-28 route复位改为DDR域同步控制，避免组合复位长线；
//           已呈现AR保持至接收并排空，lease仅在FIFO flush完成后释放。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module monitor_display_pipeline #(
	parameter integer C_PREFETCH_WORDS=128
)(
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_route_rst_n,
	input i_ddr_init_done,
	input i_route_valid,
	input [35:0] i_route,
	output o_route_ready,
	input [7:0] i_display_slot,
	input [3:0] i_display_valid,
	input [63:0] i_display_frame_epoch,
	input [3:0] i_lease_ready,
	output [3:0] o_lease_acquire,
	output [7:0] o_lease_slot,
	output [63:0] o_lease_frame_epoch,
	output [3:0] o_lease_release,
	output [27:0] o_axi_araddr,
	output [3:0] o_axi_arlen,
	output o_axi_arvalid,
	input i_axi_arready,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	input i_pixel_clk,
	input i_pixel_rst_n,
	input [159:0] i_tag,
	input i_pixel_pop,
	output [15:0] o_pixel_rgb565,
	output o_pixel_valid,
	output o_prepared_valid,
	output [35:0] o_prepared_route,
	input i_pixel_active_valid,
	input [35:0] i_pixel_active_route,
	output o_active_valid,
	output [35:0] o_active_route,
	input i_active_ready,
	output [31:0] o_underflow_count,
	output [15:0] o_wr_water_level,
	output o_read_urgent
);
	localparam [3:0] ST_WAIT=0, ST_DRAIN=1, ST_FLUSH=2, ST_WAIT_FLUSH=3,
		ST_SNAPSHOT=4, ST_LEASE=5, ST_START=6, ST_RUN=7;
	reg [3:0] state_current,state_next;
	reg pending_valid;
	reg [35:0] pending_route, active_route_ddr, descriptor_route;
	reg [55:0] descriptor_base;
	reg [1:0] descriptor_present;
	reg [3:0] descriptor_mask, held_mask;
	reg [7:0] descriptor_slot;
	reg [63:0] descriptor_epoch;
	reg [15:0] pixel_frame_id, descriptor_frame_id;
	reg frame_toggle;
	reg [1:0] frame_sync;
	reg frame_seen;
	reg prepared_toggle,prepared_sent;
	reg [35:0] prepared_hold;
	reg [15:0] prepared_frame_hold;
	reg [1:0] prepared_sync;
	reg prepared_seen,pixel_prepared_pending;
	reg [35:0] pixel_prepared_route;
	reg [15:0] pixel_prepared_frame;
	reg applied_toggle;
	reg [35:0] applied_hold;
	reg [1:0] applied_sync;
	reg applied_seen,active_valid_o;
	reg [35:0] active_route_o;
	// 状态机使用专用低扇出DDR复位释放链，避免共享复位网直接驱动
	// 状态寄存器异步清零端。DDR硬复位异步断言；route低同步进入DRAIN。
	reg [1:0] state_reset_sync /* synthesis syn_preserve = 1 */;
	// tag[156] 是 video_hdmi_tag_adapter 侧带的真实 SOF 边界。
	// 不再使用 DE 与坐标组合推断帧首；quad_video_window 的边界发生在消隐期，
	// 此时 tag[157] 必为 0，且注册后的 DE/坐标可能相差一拍。
	wire frame_boundary=i_tag[156];
	wire state_reset_n=state_reset_sync[1];
	wire ddr_frame_event=(frame_sync[1]!=frame_seen);
	wire [35:0] candidate_route=pending_valid ? pending_route : active_route_ddr;
	wire [1:0] top_source=candidate_route[19:18], bottom_source=candidate_route[17:16];
	wire [1:0] top_slot=i_display_slot[top_source*2+:2], bottom_slot=i_display_slot[bottom_source*2+:2];
	wire engine_busy,engine_done,engine_failed,word_valid,word_ready;
	wire [255:0] word_data;
	wire burst_req;
	wire [27:0] burst_addr;
	wire [3:0] burst_len;
	wire output_valid, scheduler_done,scheduler_error,scheduler_busy;
	wire [255:0] output_data;
	wire fifo_full,flush_busy,flush_done,underflow_event;
	wire [15:0] free_words=(o_wr_water_level<512) ? (16'd512-o_wr_water_level) : 16'd0;
	wire output_ready=i_route_rst_n && (state_current==ST_RUN) && !fifo_full;
	wire all_leases_ready=((descriptor_mask & ~i_lease_ready)==0);
	assign o_route_ready=!pending_valid && !active_valid_o && i_route_rst_n;
	assign o_lease_slot=descriptor_slot;
	assign o_lease_frame_epoch=descriptor_epoch;
	assign o_lease_acquire=(i_route_rst_n && state_current==ST_LEASE && all_leases_ready) ? descriptor_mask : 4'd0;
	assign o_lease_release=(state_current==ST_WAIT_FLUSH && flush_done) ? held_mask : 4'd0;
	assign o_read_urgent=(i_route_rst_n && state_current==ST_RUN && o_wr_water_level<128);
	assign o_active_valid=active_valid_o && i_route_rst_n;
	assign o_active_route=active_route_o;
	// prepared 表示整帧描述符/FIFO 已预取完成，不应依赖像素 pop。
	// 旧逻辑把 o_pixel_valid 作为门控，而 compositor 在 route_enabled=0
	// 时不会 pop monitor FIFO，形成 prepared 到 route_enabled 的闭环死锁。
	assign o_prepared_valid=pixel_prepared_pending && pixel_prepared_frame==pixel_frame_id;
	assign o_prepared_route=pixel_prepared_route;
	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if(!i_pixel_rst_n) begin
			pixel_frame_id<=0; frame_toggle<=0; prepared_sync<=0; prepared_seen<=0;
			pixel_prepared_pending<=0; pixel_prepared_route<=0; pixel_prepared_frame<=0;
			applied_toggle<=0; applied_hold<=0;
		end else begin
			prepared_sync<={prepared_sync[0],prepared_toggle};
			if(frame_boundary) begin pixel_frame_id<=pixel_frame_id+1'b1; frame_toggle<=!frame_toggle; pixel_prepared_pending<=0; end
			if(prepared_sync[1]!=prepared_seen) begin
				prepared_seen<=prepared_sync[1]; pixel_prepared_route<=prepared_hold;
				pixel_prepared_frame<=prepared_frame_hold; pixel_prepared_pending<=1;
			end
			if(i_pixel_active_valid) begin
				applied_hold<=i_pixel_active_route; applied_toggle<=!applied_toggle; pixel_prepared_pending<=0;
			end
		end
	end
	// i_route_rst_n由同DDR域两FF同步释放，低电平至少跨两拍。
	// route低立刻禁止新事务，控制寄存器最晚在下个DDR沿清零。
	// DDR硬复位仍异步断言；像素侧沿用独立i_pixel_rst_n复位契约。
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) begin
			frame_sync<=0; frame_seen<=0; applied_sync<=0; applied_seen<=0;
			pending_valid<=0; pending_route<=0; active_route_ddr<={16'd0,2'd3,2'd2,16'd0};
			active_valid_o<=0; active_route_o<={16'd0,2'd3,2'd2,16'd0};
		end else if(!i_route_rst_n) begin
			frame_sync<=0; frame_seen<=0; applied_sync<=0; applied_seen<=0;
			pending_valid<=0; pending_route<=0; active_route_ddr<={16'd0,2'd3,2'd2,16'd0};
			active_valid_o<=0; active_route_o<={16'd0,2'd3,2'd2,16'd0};
		end else begin
			frame_sync<={frame_sync[0],frame_toggle}; applied_sync<={applied_sync[0],applied_toggle};
			if(ddr_frame_event) frame_seen<=frame_sync[1];
			if(active_valid_o && i_active_ready) active_valid_o<=0;
			if(i_route_valid && o_route_ready) begin pending_valid<=1; pending_route<=i_route; end
			if(applied_sync[1]!=applied_seen) begin
				applied_seen<=applied_sync[1]; active_route_ddr<=applied_hold;
				if(pending_valid && applied_hold==pending_route) begin
					active_valid_o<=1; active_route_o<=applied_hold; pending_valid<=0;
				end
			end
		end
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) state_reset_sync<=2'b00;
		else state_reset_sync<={state_reset_sync[0],1'b1};
	end
	always @(posedge i_ddr_clk or negedge state_reset_n) begin
		if(!state_reset_n) state_current<=ST_DRAIN;
		else if(!i_route_rst_n) state_current<=ST_DRAIN;
		else state_current<=state_next;
	end
	always @(*) begin
		state_next=state_current;
		case(state_current)
			ST_WAIT: if(ddr_frame_event && i_ddr_init_done) state_next=ST_DRAIN;
			ST_DRAIN: if(!engine_busy && !flush_busy) state_next=ST_FLUSH;
			ST_FLUSH: state_next=ST_WAIT_FLUSH;
			ST_WAIT_FLUSH: if(flush_done) state_next=ST_SNAPSHOT;
			ST_SNAPSHOT: state_next=ST_LEASE;
			ST_LEASE: state_next=all_leases_ready ? ST_START : ST_SNAPSHOT;
			ST_START: state_next=ST_RUN;
			ST_RUN: if(ddr_frame_event) state_next=ST_DRAIN;
			default: state_next=ST_WAIT;
		endcase
		// 任意准备阶段遇下一帧重新drain，旧frame token永不生效。
		if(ddr_frame_event && state_current!=ST_WAIT) state_next=ST_DRAIN;
	end
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) begin
			descriptor_route<=0; descriptor_base<=0; descriptor_present<=0; descriptor_mask<=0;
			descriptor_slot<=0; descriptor_epoch<=0; descriptor_frame_id<=0;
			prepared_toggle<=0; prepared_sent<=0; prepared_hold<=0; prepared_frame_hold<=0;
		end else if(!i_route_rst_n) begin
			// 控制/PLL复位禁止新prepared；HMIC engine保持DDR复位独立以排空已接受burst。
			prepared_toggle<=0; prepared_sent<=0; prepared_hold<=0; prepared_frame_hold<=0;
		end else begin
			if(state_current==ST_SNAPSHOT) begin
				descriptor_route<=candidate_route; descriptor_frame_id<=pixel_frame_id;
				descriptor_base[27:0]<=28'hc0000*top_source+28'h40000*top_slot;
				descriptor_base[55:28]<=28'hc0000*bottom_source+28'h40000*bottom_slot;
				descriptor_present<={i_display_valid[bottom_source],i_display_valid[top_source]};
				descriptor_mask<=((4'b0001<<top_source)&i_display_valid)|((4'b0001<<bottom_source)&i_display_valid);
				descriptor_slot<=i_display_slot; descriptor_epoch<=i_display_frame_epoch;
			end
			if(state_current!=ST_RUN) prepared_sent<=0;
			else if(!prepared_sent && o_wr_water_level>=C_PREFETCH_WORDS) begin
				prepared_hold<=descriptor_route; prepared_frame_hold<=descriptor_frame_id;
				prepared_toggle<=!prepared_toggle; prepared_sent<=1;
			end
		end
	end
	// route复位不能清掉尚未排空burst所持的lease。独立记录释放，
	// 防止route低与flush_done同拍时跳过held_mask清零并重复释放。
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if(!i_ddr_rst_n) held_mask<=4'd0;
		else if(state_current==ST_WAIT_FLUSH && flush_done) held_mask<=4'd0;
		else if(i_route_rst_n && state_current==ST_LEASE && all_leases_ready) held_mask<=descriptor_mask;
	end
	monitor_read_scheduler u_scheduler(
		.i_clk(i_ddr_clk),.i_rst_n(i_ddr_rst_n && (state_current==ST_START || state_current==ST_RUN)),
		.i_frame_start(i_route_rst_n && state_current==ST_START),.i_source_base(descriptor_base),.i_source_present(descriptor_present),
		.i_fifo_free_words(free_words),.i_output_ready(output_ready),.i_engine_busy(engine_busy),
		.i_burst_done(engine_done),.i_burst_failed(engine_failed),.i_word_valid(word_valid),.i_word_data(word_data),
		.o_word_ready(word_ready),.o_burst_req(burst_req),.o_burst_addr(burst_addr),.o_burst_len(burst_len),
		.o_output_valid(output_valid),.o_output_data(output_data),.o_busy(scheduler_busy),.o_frame_done(scheduler_done),.o_frame_error(scheduler_error));
	ddr_read_burst_engine u_engine(
		.i_clk(i_ddr_clk),.i_rst_n(i_ddr_rst_n),.i_abort(!i_route_rst_n || state_current!=ST_RUN),
		.i_burst_req(i_route_rst_n && burst_req && state_current==ST_RUN),.i_start_addr(burst_addr),.i_burst_len(burst_len),
		.i_output_capacity(free_words),.o_busy(engine_busy),.o_burst_done(engine_done),.o_burst_failed(engine_failed),
		.o_axi_araddr(o_axi_araddr),.o_axi_arlen(o_axi_arlen),.o_axi_arvalid(o_axi_arvalid),
		.i_axi_arready(i_axi_arready),.i_axi_rdata(i_axi_rdata),.i_axi_rvalid(i_axi_rvalid),
		.o_word_valid(word_valid),.o_word_data(word_data),.i_word_ready(word_ready));
	monitor_ddr_to_hdmi_fifo_wrapper u_fifo(
		.i_ddr_clk(i_ddr_clk),.i_ddr_rst_n(i_ddr_rst_n),.i_frame_switch(state_current==ST_FLUSH),
		.o_flush_busy(flush_busy),.o_flush_done(flush_done),.i_word_push(output_valid && output_ready),
		.i_word_data(output_data),.o_word_full(fifo_full),.i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(i_pixel_rst_n),
		.i_measure_enable(state_current==ST_RUN),.i_pixel_pop(i_pixel_pop),.o_pixel_rgb565(o_pixel_rgb565),
		.o_pixel_valid(o_pixel_valid),.o_underflow_count(o_underflow_count),.o_underflow_event(underflow_event),
		.o_wr_water_level(o_wr_water_level),.o_rd_water_level());
endmodule
