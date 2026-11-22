//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-06
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 协调 PCIe 与像素域之间的单帧 grant、隔帧采样和稳定帧描述符，跟踪已接受帧及元数据 token。
// Module Name: pcie_video_frame_ctrl
// File Name: source/pcie/pcie_video_frame_ctrl.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 协调 PCIe 与像素域之间的单帧 grant、隔帧采样和稳定帧描述符，跟踪已接受帧及元数据 token。
// 串行处理双域 flush，在下游确认前保持帧元数据，避免旧授权或旧完成跨越新会话。
//
// Dependencies: 无子模块；连接像素 tap、FIFO flush 及 PCIe 消费端控制。
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
// Create Date: 2026-10-06
// Design Name: pcie_video_frame_ctrl
// Module Name: pcie_video_frame_ctrl
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 单帧 grant、稳定描述符握手、隔帧采样及串行双域 flush。
// Revision: 0.03 - Track accepted frames; serialize flush and retain metadata.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_video_frame_ctrl(
	input i_hdmi_clk,
	input i_hdmi_rst_n,
	input i_pcie_clk,
	input i_pcie_rst_n,
	input i_link_up,
	input i_enable,
	input i_stop,
	input i_rate_30,
	input i_host_buffer_ready,
	input i_tap_frame_start,
	input i_tap_frame_accept,
	input i_tap_frame_end,
	input i_tap_frame_drop,
	input i_tap_frame_error,
	input [31:0] i_tap_frame_id,
	input [31:0] i_tap_byte_count,
	input [17:0] i_tap_word_count,
	input [31:0] i_tap_crc32,
	input i_tap_meta_toggle,
	input i_tap_boundary_toggle,
	input i_tap_flush_ack_toggle,
	input i_fifo_flush_ack_toggle,
	input i_meta_consume_ack_toggle,
	output o_hdmi_capture_enable,
	output o_hdmi_flush,
	output o_flush_req_toggle,
	output o_flush_done,
	output o_pcie_busy,
	output o_frame_grant_toggle,
	output o_meta_valid,
	output [31:0] o_meta_frame_id,
	output [31:0] o_meta_byte_count,
	output [17:0] o_meta_word_count,
	output [31:0] o_meta_crc32,
	output o_meta_drop,
	output o_meta_error,
	output o_meta_ack_toggle
);
	// 复位契约：外层将任一域的复位合并后同时复位 ctrl/tap/FIFO/consumer，
	// 各域同步释放。单独清一个 toggle 端点不受支持。
	// start/accept/end 均为 HDMI 域单拍；accept 与 start 同拍，包含 FIFO
	// 首拍已满的获准错误帧。descriptor 在 end 同拍有效。控制输入均属 PCIe 域。
	// meta_valid 期间总线/token 保持；消费端仅在成功帧已取完数据或错误帧
	// 已停读后，将 ack 置为 o_meta_ack_toggle；flush 时取消旧 ack 事务。
	// i_tap_meta_toggle/boundary_toggle 保留旧接口兼容，不用于 descriptor token。
	reg [1:0] grant_sync;
	reg [1:0] grant_epoch_sync;
	reg [1:0] rate_sync;
	reg [1:0] flush_sync_hdmi;
	reg grant_seen_hdmi;
	reg grant_settle_hdmi;
	reg grant_pending_hdmi;
	reg frame_owned_hdmi;
	reg rate_skip_hdmi;
	reg flush_seen_hdmi;
	reg hdmi_flush_o;
	reg [31:0] hdmi_meta_frame_id;
	reg [31:0] hdmi_meta_byte_count;
	reg [17:0] hdmi_meta_word_count;
	reg [31:0] hdmi_meta_crc32;
	reg hdmi_meta_drop;
	reg hdmi_meta_error;
	reg hdmi_meta_toggle;
	reg grant_active;
	reg grant_toggle_o;
	reg grant_epoch_o;
	reg flush_req_toggle_o;
	reg flush_condition_prev;
	reg flush_waiting;
	reg flush_again;
	reg flush_done_o;
	reg [1:0] meta_sync;
	reg meta_seen;
	reg meta_pending;
	reg meta_valid_o;
	reg [31:0] meta_frame_id_o;
	reg [31:0] meta_byte_count_o;
	reg [17:0] meta_word_count_o;
	reg [31:0] meta_crc32_o;
	reg meta_drop_o;
	reg meta_error_o;
	reg meta_ack_toggle_o;
	reg [1:0] ctrl_flush_ack_sync;
	reg [1:0] tap_flush_ack_sync;
	reg [1:0] fifo_flush_ack_sync;
	wire flush_condition;
	wire flush_complete;
	wire consume_now;
	wire flush_trigger;

	assign flush_condition = !i_link_up || !i_enable || i_stop;
	assign flush_complete = (ctrl_flush_ack_sync[1] == flush_req_toggle_o) &&
		(tap_flush_ack_sync[1] == flush_req_toggle_o) &&
		(fifo_flush_ack_sync[1] == flush_req_toggle_o);
	assign consume_now = meta_valid_o &&
		(i_meta_consume_ack_toggle == meta_ack_toggle_o);
	assign flush_trigger = (flush_condition && !flush_condition_prev) ||
		(consume_now && (meta_drop_o || meta_error_o));
	assign o_hdmi_capture_enable = grant_pending_hdmi && !rate_skip_hdmi;
	assign o_hdmi_flush = hdmi_flush_o;
	assign o_flush_req_toggle = flush_req_toggle_o;
	assign o_flush_done = flush_done_o;
	assign o_pcie_busy = grant_active || meta_pending || meta_valid_o ||
		flush_waiting || flush_condition;
	assign o_frame_grant_toggle = grant_toggle_o;
	assign o_meta_valid = meta_valid_o;
	assign o_meta_frame_id = meta_frame_id_o;
	assign o_meta_byte_count = meta_byte_count_o;
	assign o_meta_word_count = meta_word_count_o;
	assign o_meta_crc32 = meta_crc32_o;
	assign o_meta_drop = meta_drop_o;
	assign o_meta_error = meta_error_o;
	assign o_meta_ack_toggle = meta_ack_toggle_o;

	// 稳定总线 CDC：descriptor 在接受帧结束时锁存，在下次 grant 帧结束前不变。
	// grant 附带 flush epoch，额外等待一拍后才启用，迟到的旧 grant 被丢弃。
	always @(posedge i_hdmi_clk or negedge i_hdmi_rst_n) begin
		if (!i_hdmi_rst_n) begin
			grant_sync <= 2'b00;
			grant_epoch_sync <= 2'b00;
			rate_sync <= 2'b00;
			flush_sync_hdmi <= 2'b00;
			grant_seen_hdmi <= 1'b0;
			grant_settle_hdmi <= 1'b0;
			grant_pending_hdmi <= 1'b0;
			frame_owned_hdmi <= 1'b0;
			rate_skip_hdmi <= 1'b0;
			flush_seen_hdmi <= 1'b0;
			hdmi_flush_o <= 1'b0;
			hdmi_meta_frame_id <= 32'd0;
			hdmi_meta_byte_count <= 32'd0;
			hdmi_meta_word_count <= 18'd0;
			hdmi_meta_crc32 <= 32'd0;
			hdmi_meta_drop <= 1'b0;
			hdmi_meta_error <= 1'b0;
			hdmi_meta_toggle <= 1'b0;
		end else begin
			grant_sync <= {grant_sync[0], grant_toggle_o};
			grant_epoch_sync <= {grant_epoch_sync[0], grant_epoch_o};
			rate_sync <= {rate_sync[0], i_rate_30};
			flush_sync_hdmi <= {flush_sync_hdmi[0], flush_req_toggle_o};
			hdmi_flush_o <= 1'b0;
			if (flush_sync_hdmi[1] != flush_seen_hdmi) begin
				flush_seen_hdmi <= flush_sync_hdmi[1];
				hdmi_flush_o <= 1'b1;
				grant_seen_hdmi <= grant_sync[1];
				grant_settle_hdmi <= 1'b0;
				grant_pending_hdmi <= 1'b0;
				frame_owned_hdmi <= 1'b0;
				rate_skip_hdmi <= 1'b0;
			end else begin
				if (grant_sync[1] != grant_seen_hdmi) begin
					grant_seen_hdmi <= grant_sync[1];
					grant_settle_hdmi <= 1'b1;
				end else if (grant_settle_hdmi) begin
					grant_settle_hdmi <= 1'b0;
					if (grant_epoch_sync[1] == flush_seen_hdmi)
						grant_pending_hdmi <= 1'b1;
				end
				// 采样后屏蔽紧随的一帧；该帧的首像素已经未获准才解除屏蔽。
				// accept 分支优先，避免同一帧 start 把刚设定的 skip 清掉。
				if (i_tap_frame_accept) begin
					frame_owned_hdmi <= 1'b1;
					grant_pending_hdmi <= 1'b0;
					rate_skip_hdmi <= rate_sync[1];
				end else if (i_tap_frame_start) begin
					rate_skip_hdmi <= 1'b0;
				end
				if (i_tap_frame_end && (frame_owned_hdmi || i_tap_frame_accept)) begin
					frame_owned_hdmi <= 1'b0;
					hdmi_meta_frame_id <= i_tap_frame_id;
					hdmi_meta_byte_count <= i_tap_byte_count;
					hdmi_meta_word_count <= i_tap_word_count;
					hdmi_meta_crc32 <= i_tap_crc32;
					hdmi_meta_drop <= i_tap_frame_drop;
					hdmi_meta_error <= i_tap_frame_error;
					hdmi_meta_toggle <= ~hdmi_meta_toggle;
				end
			end
		end
	end

	// flush 分支独占当前拍：不与 metadata 发布/grant 分支并行执行。
	// 连续 stop/start/stop 只排队一次重刷，前一次双域 ack 到齐再翻转请求。
	always @(posedge i_pcie_clk or negedge i_pcie_rst_n) begin
		if (!i_pcie_rst_n) begin
			grant_active <= 1'b0;
			grant_toggle_o <= 1'b0;
			grant_epoch_o <= 1'b0;
			flush_req_toggle_o <= 1'b0;
			flush_condition_prev <= 1'b0;
			flush_waiting <= 1'b0;
			flush_again <= 1'b0;
			flush_done_o <= 1'b0;
			meta_sync <= 2'b00;
			meta_seen <= 1'b0;
			meta_pending <= 1'b0;
			meta_valid_o <= 1'b0;
			meta_frame_id_o <= 32'd0;
			meta_byte_count_o <= 32'd0;
			meta_word_count_o <= 18'd0;
			meta_crc32_o <= 32'd0;
			meta_drop_o <= 1'b0;
			meta_error_o <= 1'b0;
			meta_ack_toggle_o <= 1'b0;
			ctrl_flush_ack_sync <= 2'b00;
			tap_flush_ack_sync <= 2'b00;
			fifo_flush_ack_sync <= 2'b00;
		end else begin
			meta_sync <= {meta_sync[0], hdmi_meta_toggle};
			ctrl_flush_ack_sync <= {ctrl_flush_ack_sync[0], flush_seen_hdmi};
			tap_flush_ack_sync <= {tap_flush_ack_sync[0], i_tap_flush_ack_toggle};
			fifo_flush_ack_sync <= {fifo_flush_ack_sync[0], i_fifo_flush_ack_toggle};
			flush_done_o <= 1'b0;
			flush_condition_prev <= flush_condition;
			if (flush_waiting) begin
				grant_active <= 1'b0;
				meta_pending <= 1'b0;
				meta_valid_o <= 1'b0;
				meta_seen <= meta_sync[1];
				if (flush_trigger) flush_again <= 1'b1;
				if (flush_complete) begin
					if (flush_again || flush_trigger) begin
						flush_req_toggle_o <= ~flush_req_toggle_o;
						flush_again <= 1'b0;
					end else begin
						flush_waiting <= 1'b0;
						flush_done_o <= 1'b1;
					end
				end
			end else if (flush_trigger) begin
				flush_req_toggle_o <= ~flush_req_toggle_o;
				flush_waiting <= 1'b1;
				grant_active <= 1'b0;
				meta_pending <= 1'b0;
				meta_valid_o <= 1'b0;
				meta_seen <= meta_sync[1];
			end else if (flush_condition) begin
				grant_active <= 1'b0;
				meta_pending <= 1'b0;
				meta_valid_o <= 1'b0;
				meta_seen <= meta_sync[1];
			end else begin
				if (meta_sync[1] != meta_seen && grant_active &&
					!meta_valid_o && !meta_pending) begin
					meta_seen <= meta_sync[1];
					meta_pending <= 1'b1;
				end else if (meta_pending) begin
					meta_frame_id_o <= hdmi_meta_frame_id;
					meta_byte_count_o <= hdmi_meta_byte_count;
					meta_word_count_o <= hdmi_meta_word_count;
					meta_crc32_o <= hdmi_meta_crc32;
					meta_drop_o <= hdmi_meta_drop;
					meta_error_o <= hdmi_meta_error;
					meta_valid_o <= 1'b1;
					// 旧 ack 即使一直保持，也不能自动消费 flush 后的首个描述符。
					meta_ack_toggle_o <= ~i_meta_consume_ack_toggle;
					meta_pending <= 1'b0;
					grant_active <= 1'b0;
				end
				if (consume_now) meta_valid_o <= 1'b0;
				if (!grant_active && !meta_pending && !meta_valid_o &&
					i_host_buffer_ready) begin
					grant_toggle_o <= ~grant_toggle_o;
					grant_epoch_o <= flush_req_toggle_o;
					grant_active <= 1'b1;
				end
			end
		end
	end
endmodule
