//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 以太网视频输入层次封装，将 RGMII 双沿物理数据转换为 GMII 字节，经过 UDP 协议栈、控制/视频分流和 PGV1 解包得到 RGB565 视频。
// Module Name: ethernet_top
// File Name: source/subsystems/ethernet_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 以太网视频输入层次封装，将 RGMII 双沿物理数据转换为 GMII 字节，经过 UDP 协议栈、控制/视频分流和 PGV1 解包得到 RGB565 视频。
// 小端 RGB565 字节组成 256-bit DDR 字，经官方异步 FIFO 跨到 DDR；网络帧事件仍属于 PHY RXC 域，帧槽 grant 由外部完成同步。
//
// Dependencies: rgmii_io、eth_udp_test、udp_packet_demux、pgv1_rx、rgb565_byte_packer、c6_net_to_ddr_fifo_wrapper。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 实际 eth_udp_test 的 i_test_request 固定为 0；原头所称周期示例报文不代表当前开启周期测试发送。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Design Name: Ethernet video ingress subsystem
// Module Name: ethernet_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: RGMII、UDP分流、PGV1解析与网络到DDR FIFO的纯层次封装。
//              网络域沿用PHY RXC；RGB565小端字节流组成256-bit DDR字。
//              不新增寄存器、复位逻辑、流水级或跨时钟域通道。
//
// Dependencies: rgmii_io, eth_udp_test, udp_packet_demux, pgv1_rx,
//               rgb565_byte_packer, c6_net_to_ddr_fifo_wrapper
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//  - grant/slot须由外部在o_rgmii_clk域完成同步；帧事件输出仍属网络域。
//  - 保留旧UDP栈的周期示例报文；MDIO与控制命令处理仍在本模块之外。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module ethernet_top
#(
	parameter [47:0] C_LOCAL_MAC = 48'h02_00_00_00_00_01,
	parameter [31:0] C_LOCAL_IP = 32'hC0_A8_01_0B,
	parameter [15:0] C_LOCAL_PORT = 16'd5000,
	parameter [15:0] C_CONTROL_PORT = 16'd5002,
	parameter [31:0] C_DEST_IP = 32'hC0_A8_01_69,
	parameter [15:0] C_DEST_PORT = 16'd5001
)
(
	input i_rstn,
	input i_ddr_clk,
	input i_ddr_rstn,

	input i_rgmii_rxc,
	input i_rgmii_rx_ctl,
	input [3:0] i_rgmii_rxd,
	output o_rgmii_clk,
	output o_rgmii_txc,
	output o_rgmii_tx_ctl,
	output [3:0] o_rgmii_txd,

	input i_ack_valid,
	input [143:0] i_ack_payload,
	input [15:0] i_ack_length,
	output o_ack_ready,

	output o_video_start,
	output o_video_valid,
	output [7:0] o_video_data,
	output o_video_end,
	output [15:0] o_video_length,
	output o_ctrl_start,
	output o_ctrl_valid,
	output [7:0] o_ctrl_data,
	output o_ctrl_end,
	output [15:0] o_ctrl_length,
	output o_ctrl_error,

	input i_frame_grant,
	input [1:0] i_frame_grant_slot,
	output o_frame_prepare,
	output o_frame_commit,
	output o_frame_abort,
	output o_frame_active,
	output [1:0] o_frame_slot,
	output [31:0] o_frame_sequence,
	output [31:0] o_crc_error_count,
	output [31:0] o_sequence_error_count,
	output [31:0] o_length_error_count,
	output [31:0] o_dropped_frame_count,
	output o_packer_overflow,

	input i_word_pop,
	output [255:0] o_word_data,
	output o_word_empty,
	output [15:0] o_wr_water_level,
	output [15:0] o_rd_water_level
);
	//------------模块实例化信号------------//
	wire gmii_rx_dv;
	wire [7:0] gmii_rxd;
	wire gmii_rx_error;
	wire gmii_tx_en;
	wire [7:0] gmii_txd;
	wire udp_raw_valid;
	wire [7:0] udp_raw_data;
	wire [15:0] udp_raw_length;
	wire udp_raw_start;
	wire udp_raw_end;
	wire udp_raw_is_control;
	wire udp_raw_error;
	wire payload_valid;
	wire [7:0] payload_data;
	wire packer_word_valid;
	wire [255:0] packer_word_data;
	wire packer_word_ready;

	//------------模块实例化区域------------//
	// 保留原实例名，使约束迁移只增加ethernet_top实例的层级前缀。
	rgmii_io u_rgmii_io(
		.i_rst_n(i_rstn),
		.i_rgmii_rxc(i_rgmii_rxc),
		.i_rgmii_rx_ctl(i_rgmii_rx_ctl),
		.i_rgmii_rxd(i_rgmii_rxd),
		.o_rgmii_clk(o_rgmii_clk),
		.o_gmii_rx_dv(gmii_rx_dv),
		.o_gmii_rxd(gmii_rxd),
		.o_gmii_rx_error(gmii_rx_error),
		.i_gmii_tx_en(gmii_tx_en),
		.i_gmii_txd(gmii_txd),
		.o_rgmii_txc(o_rgmii_txc),
		.o_rgmii_tx_ctl(o_rgmii_tx_ctl),
		.o_rgmii_txd(o_rgmii_txd)
	);

	// 旧栈保留每125000000个网络周期请求一次示例UDP发送的行为。
	eth_udp_test #(
		.LOCAL_MAC(C_LOCAL_MAC),
		.LOCAL_IP(C_LOCAL_IP),
		.LOCL_PORT(C_LOCAL_PORT),
		.CONTROL_PORT(C_CONTROL_PORT),
		.DEST_IP(C_DEST_IP),
		.DEST_PORT(C_DEST_PORT)
	) u_eth_udp_test(
		.rgmii_clk(o_rgmii_clk),
		.rstn(i_rstn),
		.gmii_rx_dv(gmii_rx_dv),
		.i_gmii_rx_error(gmii_rx_error),
		.gmii_rxd(gmii_rxd),
		.gmii_tx_en(gmii_tx_en),
		.gmii_txd(gmii_txd),
		.i_ack_valid(i_ack_valid),
		.i_ack_payload(i_ack_payload),
		.i_ack_length(i_ack_length),
		.o_ack_ready(o_ack_ready),
		.udp_rec_data_valid(udp_raw_valid),
		.udp_rec_rdata(udp_raw_data),
		.udp_rec_data_length(udp_raw_length),
		.udp_rec_data_start(udp_raw_start),
		.udp_rec_data_end(udp_raw_end),
		.udp_rec_is_control(udp_raw_is_control),
		.udp_rec_error(udp_raw_error)
	);

	// 视频错误出口沿用原顶层悬空契约，控制错误随控制字节流输出。
	udp_packet_demux udp_packet_demux_Inst_rx(
		.i_is_control(udp_raw_is_control),
		.i_start(udp_raw_start),
		.i_valid(udp_raw_valid),
		.i_data(udp_raw_data),
		.i_end(udp_raw_end),
		.i_length(udp_raw_length),
		.i_error(udp_raw_error),
		.o_video_start(o_video_start),
		.o_video_valid(o_video_valid),
		.o_video_data(o_video_data),
		.o_video_end(o_video_end),
		.o_video_length(o_video_length),
		.o_video_error(),
		.o_ctrl_start(o_ctrl_start),
		.o_ctrl_valid(o_ctrl_valid),
		.o_ctrl_data(o_ctrl_data),
		.o_ctrl_end(o_ctrl_end),
		.o_ctrl_length(o_ctrl_length),
		.o_ctrl_error(o_ctrl_error)
	);

	// 仅通过协议与CRC检查的完整PGV1帧可提交；FIFO阻塞仍触发原有作废路径。
	pgv1_rx u_pgv1_rx(
		.i_clk(o_rgmii_clk),
		.i_rst_n(i_rstn),
		.i_packet_start(o_video_start),
		.i_data_valid(o_video_valid),
		.i_data(o_video_data),
		.i_packet_end(o_video_end),
		.i_frame_grant(i_frame_grant),
		.i_frame_grant_slot(i_frame_grant_slot),
		.i_packer_overflow(o_packer_overflow),
		.o_frame_prepare(o_frame_prepare),
		.o_payload_valid(payload_valid),
		.o_payload_data(payload_data),
		.o_frame_commit(o_frame_commit),
		.o_frame_abort(o_frame_abort),
		.o_frame_active(o_frame_active),
		.o_frame_slot(o_frame_slot),
		.o_frame_sequence(o_frame_sequence),
		.o_crc_error_count(o_crc_error_count),
		.o_sequence_error_count(o_sequence_error_count),
		.o_length_error_count(o_length_error_count),
		.o_dropped_frame_count(o_dropped_frame_count)
	);

	// 每32字节产生一个256-bit字，word[15:0]仍为首个RGB565像素。
	rgb565_byte_packer u_rgb565_byte_packer(
		.i_clk(o_rgmii_clk),
		.i_rst_n(i_rstn),
		.i_frame_prepare(o_frame_prepare),
		.i_frame_abort(o_frame_abort),
		.i_payload_valid(payload_valid),
		.i_payload_data(payload_data),
		.i_word_ready(packer_word_ready),
		.o_word_valid(packer_word_valid),
		.o_word_data(packer_word_data),
		.o_overflow(o_packer_overflow)
	);

	// prepare/abort清FIFO及读端预取拍数完全由既有官方FIFO封装保持。
	c6_net_to_ddr_fifo_wrapper u_c6_net_to_ddr_fifo_wrapper(
		.i_net_clk(o_rgmii_clk),
		.i_net_rst_n(i_rstn),
		.i_frame_prepare(o_frame_prepare),
		.i_frame_abort(o_frame_abort),
		.i_word_valid(packer_word_valid),
		.i_word_data(packer_word_data),
		.o_word_ready(packer_word_ready),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rstn),
		.i_word_pop(i_word_pop),
		.o_word_data(o_word_data),
		.o_word_empty(o_word_empty),
		.o_wr_water_level(o_wr_water_level),
		.o_rd_water_level(o_rd_water_level)
	);
endmodule
