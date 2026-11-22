//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-15
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: DDR 域提前为每源预约一个帧槽，经 grant toggle 允许源域采集一帧；started 仅确认源开始采集。
// Module Name: quad_source_ddr_write_ingress
// File Name: source/memory/quad_source_ddr_write_ingress.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// DDR 域提前为每源预约一个帧槽，经 grant toggle 允许源域采集一帧；started 仅确认源开始采集。
// 等待 terminal、FIFO 排空和 HMIC 事务结束后才再预约，短尾不足配置 burst 时丢弃；source0/source3 在读 urgent 下具有受限尾批救援。
// 计数不含 show-ahead 可见字，排空判定同时检查 count 与 visible；默认 guard 为 16 个 DDR 周期，CDC 必须在此窗口收敛。
//
// Dependencies: quad_source_ddr_write_pipeline；源端官方 FIFO 与帧事件 CDC。
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
// Create Date: 2026-09-15
// Design Name: Quad Video Mosaic
// Module Name: quad_source_ddr_write_ingress
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4 / ModelSim 10.6e
// Description: DDR 域提前预约帧槽，grant toggle 送源域，started 仅确认采集。
//              每源最多一帧 outstanding；terminal、FIFO 排空和 HMIC 结束后
//              才允许下一次预约。短尾不足 burst 的字仅丢弃，不写入 DDR。
// Dependencies: quad_source_ddr_write_pipeline；调用方提供事件 CDC 和官方 FIFO。
// Revision: 0.01 - File Created
// Revision: 0.02 - source0 terminal tail bounded rescue during read urgent
// Revision: 0.03 - source3 terminal tail uses the same bounded rescue
// Revision: 0.04 - register terminal settle before the shared write router
// Contract: C_FRAME_WORDS 为 C_BURST_BEATS 整数倍，C_DRAIN_GUARD_CYCLES >= 8。
//           FIFO count 不含 show-ahead 字；空必须同时检查 count 与 visible。
//           grant 后源只能采一帧，停止写入后发送 end/error；reset 须各域同步释放。
//           源 FIFO/event CDC 路径须在 guard 窗口内收敛；默认 16 个 DDR 周期。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_source_ddr_write_ingress
#(
	parameter integer C_ADDR_WIDTH = 28,
	parameter integer C_BURST_BEATS = 16,
	parameter integer C_FRAME_WORDS = 12800,
	parameter integer C_DRAIN_GUARD_CYCLES = 16,
	parameter integer C_EXTERNAL_LEASE_ENABLE = 0
)
(
	input i_clk,
	input i_rst_n,
	input [3:0] i_source_enable,
	input [3:0] i_frame_started,
	input [3:0] i_frame_end,
	input [3:0] i_frame_error,
	input i_hdmi_frame_boundary,
	input [3:0] i_retire_done,
	input [11:0] i_external_slot_held,
	output [63:0] o_display_epoch,
	input [63:0] i_fifo_word_count,
	input [3:0] i_fifo_empty,
	input [1023:0] i_fifo_data,
	input [3:0] i_fifo_almost_full,
	input i_read_urgent,
	input i_engine_ready,
	input i_axi_awready,
	input i_axi_wready,
	output [3:0] o_word_pop,
	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	output [255:0] o_axi_wdata,
	output o_axi_wvalid,
	output o_burst_done,
	output o_burst_failed,
	output o_busy,
	output [1:0] o_active_source,
	output [3:0] o_grant_valid,
	output [3:0] o_grant_toggle,
	output [7:0] o_grant_slot,
	output [3:0] o_frame_ready,
	output [3:0] o_frame_aborted,
	output [7:0] o_display_slot,
	output [3:0] o_display_valid,
	output [3:0] o_display_switch,
	output [127:0] o_valid_frame_count,
	output [127:0] o_dropped_frame_count,
	output [127:0] o_display_commit_count,
	output [35:0] o_slot_state,
	output [63:0] o_written_words
);
	wire [3:0] reserve_request;
	wire [3:0] pipeline_empty;
	wire [3:0] pipeline_pop;
	wire [3:0] discard_pop;
	wire [3:0] frame_drained;
	wire [3:0] frame_error;
	wire [3:0] frame_end;
	wire [3:0] terminal_settled_vec;
	wire [3:0] fifo_almost_full_effective;
	wire source0_starve_rescue;
	wire source3_starve_rescue;
	reg [7:0] cnt_source0_starve;
	reg [7:0] cnt_source3_starve;
	reg [3:0] grant_toggle_o;
	genvar source_index;

	assign o_grant_toggle = grant_toggle_o;
	assign o_word_pop = pipeline_pop | discard_pop;
	// source0 Ethernet 和 source3 HDMI 均无法背压。读侧 urgent 持续时尾部
	// 不会满足原 near-full 救援条件；达到 128 个 DDR 周期后临时借用同一
	// near-full 通道，只放行一个实际 pop，避免整段尾部长期压过读侧。
	assign source0_starve_rescue =
		(i_read_urgent && terminal_settled_vec[0] && !i_fifo_empty[0] &&
		 (cnt_source0_starve >= 8'd128));
	assign source3_starve_rescue =
		(i_read_urgent && terminal_settled_vec[3] && !i_fifo_empty[3] &&
		 (cnt_source3_starve >= 8'd128));
	assign fifo_almost_full_effective =
		i_fifo_almost_full | {source3_starve_rescue, 2'd0, source0_starve_rescue};

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_source0_starve <= 8'd0;
		end else if(reserve_request[0] || o_frame_ready[0] || o_frame_aborted[0] ||
			!i_read_urgent || !terminal_settled_vec[0] || i_fifo_empty[0] ||
			pipeline_pop[0] || discard_pop[0])begin
			cnt_source0_starve <= 8'd0;
		end else if(cnt_source0_starve != 8'd128)begin
			cnt_source0_starve <= cnt_source0_starve + 1'b1;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_source3_starve <= 8'd0;
		end else if(reserve_request[3] || o_frame_ready[3] || o_frame_aborted[3] ||
			!i_read_urgent || !terminal_settled_vec[3] || i_fifo_empty[3] ||
			pipeline_pop[3] || discard_pop[3])begin
			cnt_source3_starve <= 8'd0;
		end else if(cnt_source3_starve != 8'd128)begin
			cnt_source3_starve <= cnt_source3_starve + 1'b1;
		end
	end

	// 描述符在下一次 grant 前保持稳定，可直接复用 c7_slot_grant_sync。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			grant_toggle_o <= 4'd0;
		end else begin
			grant_toggle_o <= grant_toggle_o ^ o_grant_valid;
		end
	end

	generate
		for(source_index = 0; source_index < 4; source_index = source_index + 1)begin: gen_source
			reg flag_reserved;
			reg flag_started;
			reg flag_terminal;
			reg flag_error;
			reg [15:0] cnt_start_guard;
			reg [15:0] cnt_terminal_guard;
			reg [15:0] cnt_empty_guard;
			reg terminal_settled_r;
			wire has_free_slot;
			wire fifo_quiet;
			wire source_settled;
			wire terminal_settled;
			wire short_tail;
			wire frame_limit_reached;
			wire discard_tail;
			wire source_engine_busy;

			assign has_free_slot =
				((o_slot_state[source_index * 9 +: 3] == 3'd0) &&
				 (C_EXTERNAL_LEASE_ENABLE == 0 || !i_external_slot_held[source_index * 3])) ||
				((o_slot_state[source_index * 9 + 3 +: 3] == 3'd0) &&
				 (C_EXTERNAL_LEASE_ENABLE == 0 || !i_external_slot_held[source_index * 3 + 1])) ||
				((o_slot_state[source_index * 9 + 6 +: 3] == 3'd0) &&
				 (C_EXTERNAL_LEASE_ENABLE == 0 || !i_external_slot_held[source_index * 3 + 2]));
			assign reserve_request[source_index] = i_source_enable[source_index] &&
				i_engine_ready && !flag_reserved && has_free_slot;
			assign source_settled = flag_started && (cnt_start_guard == 16'd0);
			// The terminal counter is local control state.  Register the settled
			// indication before it reaches the shared router so the counter compare
			// cannot drive the wide FIFO-empty/router control cone in one cycle.
			// The extra cycle is covered by the existing terminal and drain guards.
			assign terminal_settled = terminal_settled_r;
			assign terminal_settled_vec[source_index] = terminal_settled;
			assign fifo_quiet = i_fifo_empty[source_index] &&
				(i_fifo_word_count[source_index * 16 +: 16] == 16'd0);
			// 等价于 count+1 < burst，移除同域 empty->abort 路径的进位加法。
			assign short_tail =
				(i_fifo_word_count[source_index * 16 +: 16] < C_BURST_BEATS-1);
			assign frame_limit_reached =
				(o_written_words[source_index * 16 +: 16] >= C_FRAME_WORDS);
			assign source_engine_busy = o_busy && (o_active_source == source_index);
			// 全局 burst 空闲后丢弃，避免抢走已被仲裁器选定的 source 字。
			assign discard_tail = terminal_settled && !o_busy &&
				(flag_error || short_tail || frame_limit_reached);
			assign discard_pop[source_index] = discard_tail && !i_fifo_empty[source_index];
			assign pipeline_empty[source_index] = i_fifo_empty[source_index] ||
				!source_settled || discard_tail;
			assign frame_error[source_index] = flag_error |
				(i_frame_error[source_index] && flag_reserved);
			assign frame_end[source_index] = i_frame_end[source_index] && flag_reserved;
			// visible empty 可能只是 show-ahead 重填空隙；要求 count=0 连续稳定。
			// guard 不受其他源 burst 打断，最终释放仍要求全局空闲，覆盖仲裁交接。
			assign frame_drained[source_index] = terminal_settled && fifo_quiet &&
				!o_busy && (cnt_empty_guard == C_DRAIN_GUARD_CYCLES);

			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					flag_reserved <= 1'b0;
				end else if(reserve_request[source_index])begin
					flag_reserved <= 1'b1;
				end else if(o_frame_ready[source_index] || o_frame_aborted[source_index])begin
					flag_reserved <= 1'b0;
				end
			end

			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					flag_started <= 1'b0;
					flag_terminal <= 1'b0;
					flag_error <= 1'b0;
					cnt_start_guard <= 16'd0;
					cnt_terminal_guard <= 16'd0;
					terminal_settled_r <= 1'b0;
				end else if(reserve_request[source_index])begin
					flag_started <= 1'b0;
					flag_terminal <= 1'b0;
					flag_error <= 1'b0;
					cnt_start_guard <= 16'd0;
					cnt_terminal_guard <= 16'd0;
				end else begin
					if(i_frame_started[source_index] && flag_reserved && !flag_started)begin
						flag_started <= 1'b1;
						cnt_start_guard <= C_DRAIN_GUARD_CYCLES;
					end else if(cnt_start_guard != 16'd0)begin
						cnt_start_guard <= cnt_start_guard - 16'd1;
					end
					if(flag_reserved && (i_frame_end[source_index] || i_frame_error[source_index]))begin
						flag_terminal <= 1'b1;
						cnt_terminal_guard <= C_DRAIN_GUARD_CYCLES;
					end else if(cnt_terminal_guard != 16'd0)begin
						cnt_terminal_guard <= cnt_terminal_guard - 16'd1;
					end
					terminal_settled_r <= flag_terminal && (cnt_terminal_guard == 16'd0);
					if(flag_reserved && (i_frame_error[source_index] ||
						(o_burst_failed && o_active_source == source_index) ||
						(terminal_settled && frame_limit_reached && !fifo_quiet)))begin
						flag_error <= 1'b1;
					end
				end
			end

			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					cnt_empty_guard <= 16'd0;
				end else if(reserve_request[source_index] || !terminal_settled ||
					!fifo_quiet || source_engine_busy || pipeline_pop[source_index] || discard_pop[source_index])begin
					cnt_empty_guard <= 16'd0;
				end else if(cnt_empty_guard < C_DRAIN_GUARD_CYCLES)begin
					cnt_empty_guard <= cnt_empty_guard + 16'd1;
				end
			end
		end
	endgenerate

	quad_source_ddr_write_pipeline
	#(
		.C_ADDR_WIDTH(C_ADDR_WIDTH),
		.C_BURST_BEATS(C_BURST_BEATS),
		.C_FRAME_WORDS(C_FRAME_WORDS),
		.C_EXTERNAL_LEASE_ENABLE(C_EXTERNAL_LEASE_ENABLE)
	)
	u_pipeline
	(
		.i_frame_prepare(reserve_request),
		.i_frame_drained(frame_drained),
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_frame_end(frame_end),
		.i_frame_error(frame_error),
		.i_hdmi_frame_boundary(i_hdmi_frame_boundary),
		.i_retire_done(i_retire_done),
		.i_external_slot_held(i_external_slot_held),
		.o_display_epoch(o_display_epoch),
		.i_fifo_word_count(i_fifo_word_count),
		.i_fifo_empty(pipeline_empty),
		.i_fifo_data(i_fifo_data),
		.i_fifo_almost_full(fifo_almost_full_effective),
		.i_read_urgent(i_read_urgent),
		.i_engine_ready(i_engine_ready),
		.i_axi_awready(i_axi_awready),
		.i_axi_wready(i_axi_wready),
		.o_word_pop(pipeline_pop),
		.o_axi_awaddr(o_axi_awaddr),
		.o_axi_awlen(o_axi_awlen),
		.o_axi_awvalid(o_axi_awvalid),
		.o_axi_wdata(o_axi_wdata),
		.o_axi_wvalid(o_axi_wvalid),
		.o_burst_done(o_burst_done),
		.o_burst_failed(o_burst_failed),
		.o_busy(o_busy),
		.o_active_source(o_active_source),
		.o_grant_valid(o_grant_valid),
		.o_grant_slot(o_grant_slot),
		.o_frame_ready(o_frame_ready),
		.o_frame_aborted(o_frame_aborted),
		.o_display_slot(o_display_slot),
		.o_display_valid(o_display_valid),
		.o_display_switch(o_display_switch),
		.o_valid_frame_count(o_valid_frame_count),
		.o_dropped_frame_count(o_dropped_frame_count),
		.o_display_commit_count(o_display_commit_count),
		.o_slot_state(o_slot_state),
		.o_written_words(o_written_words)
	);
endmodule
