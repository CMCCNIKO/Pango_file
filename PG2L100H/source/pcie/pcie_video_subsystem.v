//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 组合视频寄存器、HDMI 帧采集、MWr 引擎与官方组包适配，将完整 HDMI 帧写入主机授权缓冲区。
// Module Name: pcie_video_subsystem
// File Name: source/pcie/pcie_video_subsystem.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 组合视频寄存器、HDMI 帧采集、MWr 引擎与官方组包适配，将完整 HDMI 帧写入主机授权缓冲区。
// 连接描述符所有权、采集启停、流数据、帧元数据及 PCIe TLP 输出，统一处理链路掉线、flush 与错误统计。
//
// Dependencies: pcie_video_regs、pcie_video_capture、pcie_video_mwr_engine、pcie_video_packet_bridge。
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
// Create Date: 2026-10-07
// Module Name: pcie_video_subsystem
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4 / ModelSim 10.6e
// Description: T04寄存器、取流、帧DMA、官方MWr组包的单向视频分支。
// Dependencies: 当前官方ASYN_FIFO/rt_bridge_top与pcie_video_*模块。
// Contract: PCIe侧控制全属i_clk；HDMI信号只读，任何取消均不复位视频源。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module pcie_video_subsystem(
	input i_clk,
	input i_rst_n,
	input i_link_up,
	input i_bus_master_enable,
	input [15:0] i_requester_id,
	input [2:0] i_cfg_max_payload_size,
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_hdmi_vs,
	input i_hdmi_hs,
	input i_hdmi_de,
	input [23:0] i_hdmi_rgb,
	input i_legacy_busy,
	input i_legacy_reject_event,
	input i_csr_wr_en,
	input [11:0] i_csr_wr_addr,
	input [31:0] i_csr_wr_data,
	input [3:0] i_csr_wr_strb,
	input [11:0] i_csr_rd_addr,
	output [127:0] o_csr_rd_data,
	output o_video_mode,
	output o_axis_valid,
	output [127:0] o_axis_data,
	output o_axis_last,
	output o_axis_user,
	input i_axis_ready
);
	wire enable,stop,rate30;
	wire [31:0] generation;
	wire candidate_valid,claim,complete,engine_busy,engine_flush_done;
	wire [1:0] candidate_index,complete_index;
	wire [63:0] candidate_addr;
	wire [31:0] candidate_capacity,candidate_generation;
	wire [31:0] complete_generation,complete_status,complete_frame_id,complete_length,complete_crc32;
	wire stream_valid,stream_ready;
	wire [127:0] stream_data;
	wire meta_valid,meta_drop,meta_error,meta_token,meta_ack;
	wire [31:0] meta_id,meta_bytes,meta_crc;
	wire [17:0] meta_words;
	wire capture_buffer_ready,capture_busy,capture_flush_busy,capture_flush_done,capture_reset_n;
	wire [12:0] rd_level;
	wire packet_valid,packet_ready,payload_valid,payload_ready,payload_last,packet_done,packet_busy;
	wire [63:0] packet_addr;
	wire [9:0] packet_dw_count;
	wire [127:0] payload_data;
	wire effective_link,capture_stop;
	reg meta_valid_d,pressure_d;
	wire pressure;
	assign effective_link=i_link_up && i_bus_master_enable && capture_reset_n;
	// 计数矛盾即便tap未置error也必须清FIFO，不能让下一帧继承残留。
	assign capture_stop=stop || (complete && complete_status!=32'd1);
	assign pressure=(rd_level>=13'd4088);
	always @(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin meta_valid_d<=1'b0; pressure_d<=1'b0; end
		else begin meta_valid_d<=meta_valid; pressure_d<=pressure; end
	end
	pcie_video_regs u_regs(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_link_up(effective_link),
		.i_busy(engine_busy),
		.i_legacy_busy(i_legacy_busy),
		.i_flush_done(engine_flush_done),
		.i_legacy_reject_event(i_legacy_reject_event),
		.i_wr_en(i_csr_wr_en),
		.i_wr_addr(i_csr_wr_addr),
		.i_wr_data(i_csr_wr_data),
		.i_wr_strb(i_csr_wr_strb),
		.i_rd_addr(i_csr_rd_addr),
		.o_rd_data(),
		.o_rd_window_data(o_csr_rd_data),
		.o_enable(enable),
		.o_stop(stop),
		.o_rate_30(rate30),
		.o_generation(generation),
		.o_video_mode(o_video_mode),
		.o_candidate_valid(candidate_valid),
		.o_candidate_index(candidate_index),
		.o_candidate_addr(candidate_addr),
		.o_candidate_capacity(candidate_capacity),
		.o_candidate_generation(candidate_generation),
		.i_claim(claim),
		.i_complete(complete),
		.i_complete_index(complete_index),
		.i_complete_generation(complete_generation),
		.i_complete_status(complete_status),
		.i_complete_frame_id(complete_frame_id),
		.i_complete_length(complete_length),
		.i_complete_crc32(complete_crc32),
		.i_drop_event(meta_valid && !meta_valid_d && meta_drop),
		.i_overflow_event(pressure && !pressure_d)
	);
	pcie_video_capture u_capture(
		.i_pixel_clk(i_pixel_clk),
		.i_pixel_rst_n(i_pixel_rst_n),
		.i_hdmi_vs(i_hdmi_vs),
		.i_hdmi_hs(i_hdmi_hs),
		.i_hdmi_de(i_hdmi_de),
		.i_hdmi_rgb(i_hdmi_rgb),
		.i_pcie_clk(i_clk),
		.i_pcie_rst_n(i_rst_n),
		.i_link_up(i_link_up && i_bus_master_enable),
		.i_enable(enable),
		.i_stop(capture_stop),
		.i_rate_30(rate30),
		.i_host_buffer_ready(capture_buffer_ready),
		.i_stream_ready(stream_ready),
		.i_meta_consume_ack_toggle(meta_ack),
		.o_stream_valid(stream_valid),
		.o_stream_data(stream_data),
		.o_meta_valid(meta_valid),
		.o_meta_frame_id(meta_id),
		.o_meta_byte_count(meta_bytes),
		.o_meta_word_count(meta_words),
		.o_meta_crc32(meta_crc),
		.o_meta_drop(meta_drop),
		.o_meta_error(meta_error),
		.o_meta_ack_toggle(meta_token),
		.o_busy(capture_busy),
		.o_flush_busy(capture_flush_busy),
		.o_flush_done(capture_flush_done),
		.o_pcie_reset_n(capture_reset_n),
		.o_capture_enabled(),
		.o_capture_active(),
		.o_wr_water_level(),
		.o_rd_water_level(rd_level)
	);
	// 唯一候选来自ARM已原子验证的寄存器组，复用验证结果以缩短claim路径。
	// 独立engine默认仍防御性验证；此处不允许外部未验证descriptor直通。
	pcie_video_mwr_engine #(.C_TRUST_VALIDATED_DESCRIPTOR(1)) u_engine(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_link_up(i_link_up),
		.i_enable(enable && effective_link),
		.i_stop(stop),
		.i_generation(generation),
		.i_cfg_max_payload_size(i_cfg_max_payload_size),
		.i_candidate_valid(candidate_valid),
		.i_candidate_index(candidate_index),
		.i_candidate_addr(candidate_addr),
		.i_candidate_capacity(candidate_capacity),
		.i_candidate_generation(candidate_generation),
		.o_claim(claim),
		.i_stream_valid(stream_valid),
		.i_stream_data(stream_data),
		.o_stream_ready(stream_ready),
		.i_meta_valid(meta_valid),
		.i_meta_frame_id(meta_id),
		.i_meta_byte_count(meta_bytes),
		.i_meta_word_count(meta_words),
		.i_meta_crc32(meta_crc),
		.i_meta_drop(meta_drop),
		.i_meta_error(meta_error),
		.i_meta_ack_toggle(meta_token),
		.o_meta_consume_ack_toggle(meta_ack),
		.i_capture_busy(capture_busy),
		.i_capture_flush_busy(capture_flush_busy),
		.i_capture_flush_done(capture_flush_done),
		.i_capture_reset_n(capture_reset_n),
		.o_capture_buffer_ready(capture_buffer_ready),
		.o_packet_valid(packet_valid),
		.o_packet_addr(packet_addr),
		.o_packet_dw_count(packet_dw_count),
		.i_packet_ready(packet_ready),
		.o_payload_valid(payload_valid),
		.o_payload_data(payload_data),
		.o_payload_last(payload_last),
		.i_payload_ready(payload_ready),
		.i_packet_done(packet_done),
		.i_packet_busy(packet_busy),
		.o_busy(engine_busy),
		.o_flush_done(engine_flush_done),
		.o_complete(complete),
		.o_complete_index(complete_index),
		.o_complete_generation(complete_generation),
		.o_complete_status(complete_status),
		.o_complete_frame_id(complete_frame_id),
		.o_complete_length(complete_length),
		.o_complete_crc32(complete_crc32)
	);
	pcie_video_packet_bridge u_packet_bridge(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_link_up(i_link_up),
		.i_requester_id(i_requester_id),
		.i_packet_valid(packet_valid),
		.o_packet_ready(packet_ready),
		.i_packet_addr(packet_addr),
		.i_packet_dw_count(packet_dw_count),
		.i_payload_valid(payload_valid),
		.o_payload_ready(payload_ready),
		.i_payload_data(payload_data),
		.i_payload_last(payload_last),
		.o_packet_done(packet_done),
		.o_busy(packet_busy),
		.o_axis_valid(o_axis_valid),
		.o_axis_data(o_axis_data),
		.o_axis_last(o_axis_last),
		.o_axis_user(o_axis_user),
		.i_axis_ready(i_axis_ready)
	);
endmodule
