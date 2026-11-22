//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 锁存视频 DMA 的包地址、长度与命令，向已有官方 rt_bridge_top MWr 组包通路提供有效载荷。
// Module Name: pcie_video_packet_bridge
// File Name: source/pcie/pcie_video_packet_bridge.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 锁存视频 DMA 的包地址、长度与命令，向已有官方 rt_bridge_top MWr 组包通路提供有效载荷。
// 隔离 TLP 头部背压并保持命令至当前包完成，输出 128 位 TLP 流及包级完成状态。
//
// Dependencies: 官方 rt_bridge_top（合成 rt_bridge_top_v1_3_vpAll.vp，仿真使用对应 simulation 模型）。
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
// Design Name: T04 video PCIe transport
// Module Name: pcie_video_packet_bridge
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 锁存包命令，复用官方 MWr 组包器，并隔离头部背压。
// Dependencies: Existing registered rt_bridge_top; no replacement TLP formatter.
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_video_packet_bridge (
	input i_clk,
	input i_rst_n,
	input i_link_up,
	input i_packet_valid,
	output o_packet_ready,
	input [63:0] i_packet_addr,
	input [9:0] i_packet_dw_count,
	input [15:0] i_requester_id,
	input i_payload_valid,
	output o_payload_ready,
	input [127:0] i_payload_data,
	input i_payload_last,
	output o_packet_done,
	output o_busy,
	output o_axis_valid,
	output [127:0] o_axis_data,
	output o_axis_last,
	output o_axis_user,
	input i_axis_ready
);
	reg busy_o;
	reg packet_done_o;
	reg [63:0] packet_addr;
	reg [9:0] packet_dw_count;
	reg [15:0] packet_requester_id;
	reg payload_finished;
	wire bridge_rst_n = i_rst_n && i_link_up;
	wire bridge_axis_valid;
	wire [127:0] bridge_axis_data;
	wire bridge_axis_last;
	wire bridge_axis_user;
	wire bridge_ready;
	wire bridge_axis_ready;
	wire axis_stall = bridge_axis_valid && !bridge_axis_ready;
	wire bridge_valid = busy_o && !payload_finished && i_payload_valid;
	wire payload_take = bridge_valid && bridge_ready && !axis_stall;
	wire bridge_last = payload_take && i_payload_last;
	wire packet_tx_done = o_axis_valid && i_axis_ready && o_axis_last;
	wire [7:0] packet_dwbe = packet_dw_count == 10'd1 ? 8'h0f : 8'hff;
	assign o_packet_ready = bridge_rst_n && !busy_o;
	assign o_payload_ready = bridge_rst_n && busy_o && !payload_finished && bridge_ready && !axis_stall;
	assign o_packet_done = packet_done_o;
	assign o_busy = busy_o;
	// 两拍弹性寄存器保证官方组包器的头部阶段不被回压，防止重复头。
	reg [129:0] axis_head;
	reg [129:0] axis_tail;
	reg [1:0] axis_count;
	wire axis_pop = o_axis_valid && i_axis_ready;
	wire axis_push = bridge_axis_valid && bridge_axis_ready;
	assign bridge_axis_ready = axis_count < 2'd2 || axis_pop;
	assign o_axis_valid = bridge_rst_n && axis_count != 2'd0;
	assign o_axis_data = axis_head[127:0];
	assign o_axis_last = axis_head[128];
	assign o_axis_user = axis_head[129];
	always @(posedge i_clk or negedge bridge_rst_n) begin
		if (!bridge_rst_n) begin
			axis_count <= 2'd0;
			axis_head <= 130'd0;
			axis_tail <= 130'd0;
		end else begin
			case ({axis_push,axis_pop})
				2'b10: begin
					axis_count <= axis_count + 2'd1;
					if (axis_count == 2'd0) axis_head <= {bridge_axis_user,bridge_axis_last,bridge_axis_data};
					else axis_tail <= {bridge_axis_user,bridge_axis_last,bridge_axis_data};
				end
				2'b01: begin
					axis_count <= axis_count - 2'd1;
					axis_head <= axis_tail;
				end
				2'b11: begin
					if (axis_count == 2'd1) axis_head <= {bridge_axis_user,bridge_axis_last,bridge_axis_data};
					else begin
						axis_head <= axis_tail;
						axis_tail <= {bridge_axis_user,bridge_axis_last,bridge_axis_data};
					end
				end
				default: begin end
			endcase
		end
	end
	always @(posedge i_clk or negedge bridge_rst_n) begin
		if (!bridge_rst_n) begin
			busy_o <= 1'b0;
			packet_done_o <= 1'b0;
			packet_addr <= 64'd0;
			packet_dw_count <= 10'd0;
			packet_requester_id <= 16'd0;
			payload_finished <= 1'b0;
		end else begin
			packet_done_o <= 1'b0;
			if (i_packet_valid && o_packet_ready) begin
				busy_o <= 1'b1;
				packet_addr <= i_packet_addr;
				packet_dw_count <= i_packet_dw_count;
				packet_requester_id <= i_requester_id;
				payload_finished <= 1'b0;
			end else if (packet_tx_done) begin
				busy_o <= 1'b0;
				packet_done_o <= 1'b1;
			end
			if (bridge_last) payload_finished <= 1'b1;
		end
	end
rt_bridge_top u_official_mwr_bridge (
	.clk(i_clk),
	.rst_n(bridge_rst_n),
	.i_axis_master_tvld(1'b0),
	.i_axis_master_trdy(1'b0),
	.i_axis_master_tdata(128'b0),
	.i_axis_master_tkeep(4'b0),
	.i_axis_master_tlast(1'b0),
	.i_axis_master_tuser(8'b0),
	.o_tlp_common_length(),
	.o_tlp_common_at(),
	.o_tlp_common_attr(),
	.o_tlp_common_ep(),
	.o_tlp_common_td(),
	.o_tlp_common_th(),
	.o_tlp_common_tc(),
	.o_tlp_common_type(),
	.o_tlp_common_fmt(),
	.o_tlp_req_dwbe(),
	.o_tlp_req_tag(),
	.o_tlp_req_requester_id(),
	.o_tlp_req_addr(),
	.o_tlp_cpld_byte_cnt(),
	.o_tlp_cpld_bcm(),
	.o_tlp_cpld_status(),
	.o_tlp_cpld_completer_id(),
	.o_tlp_cpld_low_addr(),
	.o_tlp_cpld_tag(),
	.o_tlp_cpld_requester_id(),
	.o_bar_sel(),
	.o_req_mrd_en(),
	.o_req_mwr_en(),
	.o_req_mwr_data(),
	.o_req_mwr_dw_vld(),
	.o_req_mwr_en_flag(),
	.o_cpld_en(),
	.o_cpld_data(),
	.o_cpld_dw_vld(),
	.o_cpld_en_flag(),
	.o_cpld_multi_flag(),
	.i_mwr_data_valid(bridge_valid),
	.o_mwr_data_ready(bridge_ready),
	.i_mwr_data(i_payload_data),
	.i_mwr_last_data_en(bridge_last),
	.i_mwr_length(packet_dw_count),
	.i_mwr_at(2'b0),
	.i_mwr_attr(3'b0),
	.i_mwr_ep(1'b0),
	.i_mwr_td(1'b0),
	.i_mwr_th(1'b0),
	.i_mwr_tc(3'b0),
	.i_mwr_tlp_type(5'b0),
	.i_mwr_fmt(3'b011),
	.i_mwr_dwbe(packet_dwbe),
	.i_mwr_tag(8'b0),
	.i_mwr_requester_id(packet_requester_id),
	.i_mwr_addr(packet_addr),
	.i_axis_slave2_trdy(bridge_axis_ready),
	.o_axis_slave2_tvld(bridge_axis_valid),
	.o_axis_slave2_tdata(bridge_axis_data),
	.o_axis_slave2_tlast(bridge_axis_last),
	.o_axis_slave2_tuser(bridge_axis_user),
	.i_tx_mrd_en(1'b0),
	.i_mrd_length(10'b0),
	.i_mrd_at(2'b0),
	.i_mrd_attr(3'b0),
	.i_mrd_ep(1'b0),
	.i_mrd_td(1'b0),
	.i_mrd_th(1'b0),
	.i_mrd_tc(3'b0),
	.i_mrd_tlp_type(5'b0),
	.i_mrd_fmt(3'b0),
	.i_mrd_dwbe(8'b0),
	.i_mrd_tag(8'b0),
	.i_mrd_requester_id(16'b0),
	.i_mrd_addr(64'b0),
	.i_axis_slave1_trdy(1'b0),
	.o_axis_slave1_tvld(),
	.o_axis_slave1_tdata(),
	.o_axis_slave1_tlast(),
	.o_axis_slave1_tuser(),
	.i_cpld_tx_data_valid(1'b0),
	.o_cpld_tx_data_ready(),
	.i_cpld_tx_data(128'b0),
	.i_cpld_tx_last_data_en(1'b0),
	.i_cpld_length(10'b0),
	.i_cpld_at(2'b0),
	.i_cpld_attr(3'b0),
	.i_cpld_ep(1'b0),
	.i_cpld_td(1'b0),
	.i_cpld_th(1'b0),
	.i_cpld_tc(3'b0),
	.i_cpld_tlp_type(5'b0),
	.i_cpld_fmt(3'b0),
	.i_cpld_byte_cnt(12'b0),
	.i_cpld_bcm(1'b0),
	.i_cpld_compl_status(3'b0),
	.i_cpld_completer_id(16'b0),
	.i_cpld_low_addr(7'b0),
	.i_cpld_tag(8'b0),
	.i_cpld_requester_id(16'b0),
	.i_axis_slave0_trdy(1'b0),
	.o_axis_slave0_tvld(),
	.o_axis_slave0_tdata(),
	.o_axis_slave0_tlast(),
	.o_axis_slave0_tuser());
endmodule

