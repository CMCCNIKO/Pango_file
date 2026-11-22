//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 组合算法命令接收、配置 CDC 和状态 ACK 队列，源域处理控制包与应答，像素域在帧边界原子提交算法配置。
// Module Name: algorithm_control_v2
// File Name: source/video/control_v2/algorithm_control_v2.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 组合算法命令接收、配置 CDC 和状态 ACK 队列，源域处理控制包与应答，像素域在帧边界原子提交算法配置。
// Ethernet 与 PCIe 共用有效配置基线及提交过程，各自保留独立应答；调用方提供双域协调复位。
//
// Dependencies: algo_command_rx_v2、algo_config_cdc_v2、algo_status_queue_v2
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
// Create Date: 2026-10-10
// Design Name: HDMI v1/v2 control subsystem
// Module Name: algorithm_control_v2
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 独立控制子系统。源域收包及ACK，像素域帧边界原子提交。
//              双域协调异步断言、同步释放复位由调用方提供。
// Dependencies: algo_command_rx_v2, algo_config_cdc_v2, algo_status_queue_v2
// Revision: 0.02 - 2026-10-23，Ethernet/PCIe共享帧边界提交及独立应答
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module algorithm_control_v2
(
	input i_control_clk,
	input i_control_rstn,
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	input i_frame_boundary,
	input i_ext_valid,
	input i_ext_query,
	input [63:0] i_ext_config,
	output o_ext_ready,
	output o_ext_done,
	output [7:0] o_ext_status,
	output [63:0] o_ext_snapshot,
	output [15:0] o_ext_epoch,
	output [1:0] o_ext_source,
	output [63:0] o_active_config,
	output [15:0] o_config_epoch,
	output o_busy,
	output o_pending,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	input i_ack_ready,
	output [31:0] o_dropped_status_count
);
	wire flag_accepted;
	wire flag_rejected;
	wire [7:0] event_error;
	wire [7:0] event_version;
	wire [7:0] event_opcode;
	wire [15:0] event_sequence;
	wire [63:0] shadow_config;
	wire flag_cdc_busy;
	wire flag_applied;
	wire flag_credit;
	reg [7:0] reg_version;
	reg [7:0] reg_opcode;
	reg [15:0] reg_sequence;
	reg [63:0] reg_active_snapshot;
	wire [63:0] active_ack_snapshot;
	reg [39:0] reg_deferred [0:3];
	reg [1:0] reg_read;
	reg [1:0] reg_write;
	reg [2:0] cnt_deferred;
	reg [31:0] reg_deferred_dropped;
	wire [31:0] queue_dropped;
	wire flag_defer;
	wire flag_pop_deferred;
	wire flag_push_deferred;
	wire flag_reject_dispatch;
	wire [39:0] reject_event;
	reg [63:0] reg_authoritative;
	reg [63:0] reg_eth_pending_config;
	reg flag_eth_pending;
	reg [63:0] reg_submit;
	reg update_o;
	reg [1:0] reg_inflight_source;
	reg [1:0] reg_applied_source;
	reg [15:0] reg_applied_epoch;
	reg ext_done_o;
	reg [7:0] ext_status_o;
	reg [63:0] ext_snapshot_o;
	reg [15:0] ext_epoch_o;
	reg [1:0] ext_source_o;
	wire flag_receiving;
	wire flag_eth_busy;
	wire flag_eth_applied;
	wire flag_ext_accept;
	wire flag_ext_config_valid;
	assign flag_eth_busy = !flag_credit || flag_eth_pending || flag_accepted ||
		(reg_inflight_source == 2'd1);
	assign o_busy = flag_cdc_busy || update_o || reg_inflight_source != 2'd0 ||
		flag_eth_pending || !flag_credit || flag_accepted || cnt_deferred != 3'd0;
	// Ethernet has priority from packet start through parsing and pending dispatch.
	// External requests never reserve or consume UDP ACK credit.
	assign o_ext_ready = !ext_done_o && reg_inflight_source == 2'd0 && !flag_cdc_busy && !update_o &&
		!flag_eth_pending && !flag_receiving && !i_start && !i_end &&
		!flag_accepted && !flag_rejected;
	assign flag_ext_accept = i_ext_valid && o_ext_ready;
	assign flag_eth_applied = flag_applied && reg_inflight_source == 2'd1;
	assign flag_ext_config_valid = i_ext_config[63:60] == 4'd0 &&
		i_ext_config[55:52] == 4'd0 && i_ext_config[39:32] != 8'd0 &&
		i_ext_config[31:24] != 8'd0 && i_ext_config[31:24] <= 8'd250 &&
		i_ext_config[23:17] == 7'd0 && i_ext_config[15:0] <= 16'd1443;
	assign o_ext_done = ext_done_o;
	assign o_ext_status = ext_status_o;
	assign o_ext_snapshot = ext_snapshot_o;
	assign o_ext_epoch = ext_epoch_o;
	assign o_ext_source = ext_source_o;

	// Shared baseline includes the frozen in-flight update. Partial UDP updates
	// must not restore stale parser-local parameters after a PCIe transaction.
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_authoritative <= 64'h000fe619dc010080;
		end else if(flag_accepted)begin
			reg_authoritative <= shadow_config;
		end else if(flag_ext_accept && !i_ext_query && flag_ext_config_valid)begin
			reg_authoritative <= {i_ext_config[63:16],
				i_ext_config[15:0] == 16'd0 ? 16'd128 : i_ext_config[15:0]};
		end
	end
	// One Ethernet update may wait behind an external transaction. The request
	// source is stable until the unique existing pixel CDC acknowledges it.
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_eth_pending_config <= 64'h000fe619dc010080;
			flag_eth_pending <= 1'b0;
			reg_submit <= 64'h000fe619dc010080;
			update_o <= 1'b0;
			reg_inflight_source <= 2'd0;
		end else begin
			update_o <= 1'b0;
			if(flag_accepted)begin
				reg_eth_pending_config <= shadow_config;
				flag_eth_pending <= 1'b1;
			end
			if(flag_applied)begin
				reg_inflight_source <= 2'd0;
			end
			if(reg_inflight_source == 2'd0 && !flag_cdc_busy && !update_o)begin
				if(flag_eth_pending)begin
					reg_submit <= reg_eth_pending_config;
					update_o <= 1'b1;
					reg_inflight_source <= 2'd1;
					flag_eth_pending <= 1'b0;
				end else if(flag_ext_accept && !i_ext_query && flag_ext_config_valid)begin
					reg_submit <= {i_ext_config[63:16],
						i_ext_config[15:0] == 16'd0 ? 16'd128 : i_ext_config[15:0]};
					update_o <= 1'b1;
					reg_inflight_source <= 2'd2;
				end
			end
		end
	end
	// Epoch follows the existing CDC's change-only rule. The source-held submit
	// image is used instead of an unsynchronized pixel-domain configuration bus.
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_applied_source <= 2'd0;
			reg_applied_epoch <= 16'd0;
			ext_done_o <= 1'b0;
			ext_status_o <= 8'd0;
			ext_snapshot_o <= 64'h000fe619dc010080;
			ext_epoch_o <= 16'd0;
			ext_source_o <= 2'd0;
		end else begin
			ext_done_o <= 1'b0;
			if(flag_applied)begin
				reg_applied_source <= reg_inflight_source;
				if(reg_active_snapshot != reg_submit)begin
					reg_applied_epoch <= reg_applied_epoch + 16'd1;
				end
				if(reg_inflight_source == 2'd2)begin
					ext_done_o <= 1'b1;
					ext_status_o <= 8'd0;
					ext_snapshot_o <= reg_submit;
					ext_epoch_o <= reg_applied_epoch + (reg_active_snapshot != reg_submit ? 16'd1 : 16'd0);
					ext_source_o <= 2'd2;
				end
			end
			if(flag_ext_accept && (i_ext_query || !flag_ext_config_valid))begin
				ext_done_o <= 1'b1;
				ext_status_o <= i_ext_query ? 8'd0 : 8'd2;
				ext_snapshot_o <= reg_active_snapshot;
				ext_epoch_o <= reg_applied_epoch;
				ext_source_o <= reg_applied_source;
			end
		end
	end
	assign o_dropped_status_count = queue_dropped + reg_deferred_dropped;
	// 源域无法安全观察尚未同步返回的目标提交；busy期间拒绝先暂存。
	// 停像素钟时拒绝ACK也延后，主机超时必须标记未知而非推断未执行。
	assign flag_defer = flag_cdc_busy || update_o || reg_inflight_source != 2'd0 || flag_accepted || cnt_deferred != 3'd0;
	assign flag_pop_deferred = cnt_deferred != 3'd0 && !flag_cdc_busy && !update_o && reg_inflight_source == 2'd0 && !flag_accepted;
	assign flag_push_deferred = flag_rejected && flag_defer && (cnt_deferred < 3'd4 || flag_pop_deferred);
	assign flag_reject_dispatch = flag_pop_deferred || (flag_rejected && !flag_defer);
	assign reject_event = flag_pop_deferred ? reg_deferred[reg_read] : {event_error,event_version,event_opcode,event_sequence};
	// Four fixed-index, asynchronously reset FF slots. This 160-bit event
	// queue must not infer distributed RAM: the routed RAM32 DIL path had
	// negative hold slack. Count=0 hides reset contents; queue timing is unchanged.
	genvar deferred_slot;
	generate
		for(deferred_slot = 0; deferred_slot < 4; deferred_slot = deferred_slot + 1)begin: gen_deferred_ff
			always@(posedge i_control_clk or negedge i_control_rstn)begin
				if(!i_control_rstn)begin
					reg_deferred[deferred_slot] <= 40'd0;
				end else if(flag_push_deferred && reg_write == deferred_slot)begin
					reg_deferred[deferred_slot] <= {event_error,event_version,event_opcode,event_sequence};
				end
			end
		end
	endgenerate
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_read <= 2'd0;
			reg_write <= 2'd0;
			cnt_deferred <= 3'd0;
			reg_deferred_dropped <= 32'd0;
		end else begin
			if(flag_push_deferred)begin reg_write <= reg_write + 2'd1; end
			if(flag_pop_deferred)begin reg_read <= reg_read + 2'd1; end
			case({flag_push_deferred,flag_pop_deferred})
				2'b10:begin cnt_deferred <= cnt_deferred + 3'd1; end
				2'b01:begin cnt_deferred <= cnt_deferred - 3'd1; end
				default:begin cnt_deferred <= cnt_deferred; end
			endcase
			if(flag_rejected && flag_defer && !flag_push_deferred)begin
				reg_deferred_dropped <= reg_deferred_dropped + 32'd1;
			end
		end
	end
	// ACK返回同拍的拒绝必须报告已提交的新配置，而非前一个源域快照。
	assign active_ack_snapshot = flag_applied ? reg_submit : reg_active_snapshot;
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_version <= 8'd1;
			reg_opcode <= 8'd1;
			reg_sequence <= 16'd0;
		end else if(flag_accepted)begin
			reg_version <= event_version;
			reg_opcode <= event_opcode;
			reg_sequence <= event_sequence;
		end
	end
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_active_snapshot <= 64'h000fe619dc010080;
		end else if(flag_applied)begin
			reg_active_snapshot <= reg_submit;
		end
	end
	algo_command_rx_v2 algo_command_rx_v2_Inst_rx(
		.i_clk(i_control_clk),
		.i_rstn(i_control_rstn),
		.i_start(i_start),
		.i_valid(i_valid),
		.i_data(i_data),
		.i_end(i_end),
		.i_length(i_length),
		.i_error(i_error),
		.i_busy(flag_eth_busy),
		.i_base_config(reg_authoritative),
		.o_receiving(flag_receiving),
		.o_accepted(flag_accepted),
		.o_rejected(flag_rejected),
		.o_error(event_error),
		.o_version(event_version),
		.o_opcode(event_opcode),
		.o_sequence(event_sequence),
		.o_shadow(shadow_config)
	);
	algo_config_cdc_v2 algo_config_cdc_v2_Inst_config(
		.i_src_clk(i_control_clk),
		.i_src_rst_n(i_control_rstn),
		.i_dst_clk(i_pixel_clk),
		.i_dst_rst_n(i_pixel_rstn),
		.i_update(update_o),
		.i_config(reg_submit),
		.i_frame_boundary(i_frame_boundary),
		.o_busy(flag_cdc_busy),
		.o_applied(flag_applied),
		.o_active_config(o_active_config),
		.o_config_epoch(o_config_epoch),
		.o_pending(o_pending)
	);
	algo_status_queue_v2 algo_status_queue_v2_Inst_ack(
		.i_clk(i_control_clk),
		.i_rst_n(i_control_rstn),
		.i_accepted(flag_accepted),
		.i_applied(flag_eth_applied),
		.i_rejected(flag_reject_dispatch),
		.i_error_code(reject_event[39:32]),
		.i_sequence(flag_accepted ? event_sequence : reg_sequence),
		.i_event_sequence(reject_event[15:0]),
		.i_version(flag_accepted ? event_version : reg_version),
		.i_opcode(flag_accepted ? event_opcode : reg_opcode),
		.i_config(flag_accepted ? shadow_config : reg_submit),
		.i_event_version(reject_event[31:24]),
		.i_event_opcode(reject_event[23:16]),
		.i_active_config(active_ack_snapshot),
		.o_cmd_ready(flag_credit),
		.o_ack_valid(o_ack_valid),
		.o_ack_payload(o_ack_payload),
		.o_ack_length(o_ack_length),
		.i_ack_ready(i_ack_ready),
		.o_dropped_status_count(queue_dropped)
	);
endmodule
