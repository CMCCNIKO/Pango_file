//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将显示图像 y=135..319 的二值斑马线掩码按 32 位字打包到双 bank 存储，像素域采集、系统域串行扫描。
// Module Name: host_f14_crosswalk_banks
// File Name: host/rtl/video/host_f14_crosswalk_banks.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将显示图像 y=135..319 的二值斑马线掩码按 32 位字打包到双 bank 存储，像素域采集、系统域串行扫描。
// 用完成/确认 toggle 交接 bank 和源帧号；禁止覆盖扫描中的 bank，非法坐标次序、帧中止或忙时拒收整帧并累计丢帧诊断。
//
// Dependencies: f14_ram_8x4096；像素/系统两个时钟域及共同复位；扫描端 frame_ready/read_req/release 所有权协议。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// ROI135..319，32位打包，两bank。两域共同复位，toggle完成后数据和帧号保持至ack。
// 不覆盖正在扫描的bank；非法像素序列/忙时拒收整帧并累计诊断。
`timescale 1ns/1ps
`default_nettype none
module host_f14_crosswalk_banks (
	input wire i_pixel_clk,
	input wire i_pixel_rst_n,
	input wire i_abort,
	input wire i_valid,
	input wire i_mask,
	input wire [8:0] i_row,
	input wire [9:0] i_col,
	input wire [15:0] i_frame_id,
	input wire i_sys_clk,
	input wire i_sys_rst_n,
	output wire o_frame_valid,
	output wire [15:0] o_frame_id,
	input wire i_frame_ready,
	input wire i_read_req,
	input wire [11:0] i_read_addr,
	output wire o_read_valid,
	output wire [31:0] o_read_data,
	input wire i_release,
	output wire [15:0] o_drop_count
);
	reg [1:0] completed_q,ack_q;
	(* async_reg="true" *) reg [1:0] ack_meta_q,ack_sync_q,complete_meta_q,complete_sync_q;
	reg [15:0] tags[0:1];
	reg capture_q,write_bank_q,next_bank_q;
	reg [15:0] frame_q,drop_q;
	reg [9:0] next_col_q;
	reg [8:0] next_row_q;
	reg [11:0] word_addr_q;
	reg [31:0] packed_q;
	wire first=i_valid && i_row==135 && i_col==0;
	wire [1:0] free_banks=~(completed_q^ack_sync_q);
	wire chosen_bank=free_banks[next_bank_q] ? next_bank_q : !next_bank_q;
	wire take_first=first && |free_banks && !i_abort;
	wire ordered=capture_q && i_frame_id==frame_q && i_row==next_row_q && i_col==next_col_q;
	wire pixel_write=i_valid && ordered && !i_abort && i_col[4:0]==31;
	wire [31:0] word_data={i_mask,packed_q[31:1]};
	reg read_bank_q,reading_q,read_valid_q;
	wire [31:0] read_data[0:1];
	assign o_frame_valid=!reading_q && complete_sync_q[read_bank_q]!=ack_q[read_bank_q];
	assign o_frame_id=tags[read_bank_q];
	assign o_read_valid=read_valid_q;
	assign o_read_data=read_data[read_bank_q];
	assign o_drop_count=drop_q;
	genvar bank,byte_lane;
	generate for(bank=0;bank<2;bank=bank+1) begin:g_bank
`ifdef F14_SIMULATION
		reg [31:0] memory[0:4095];
		reg [31:0] rd_q;
		always @(posedge i_pixel_clk) if(pixel_write && write_bank_q==bank) memory[word_addr_q]<=word_data;
		always @(posedge i_sys_clk) if(i_read_req && reading_q && read_bank_q==bank) rd_q<=memory[i_read_addr];
		assign read_data[bank]=rd_q;
`else
		for(byte_lane=0;byte_lane<4;byte_lane=byte_lane+1) begin:g_byte
			f14_ram_8x4096 u_ram(
				.wr_clk(i_pixel_clk),.wr_rst(1'b0),.wr_en(pixel_write && write_bank_q==bank),
				.wr_addr(word_addr_q),.wr_data(word_data[byte_lane*8 +: 8]),
				.rd_clk(i_sys_clk),.rd_rst(1'b0),.rd_clk_en(i_read_req && reading_q && read_bank_q==bank),
				.rd_addr(i_read_addr),.rd_data(read_data[bank][byte_lane*8 +: 8]));
		end
`endif
	end endgenerate
	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if(!i_pixel_rst_n) begin
			completed_q<=0;ack_meta_q<=0;ack_sync_q<=0;capture_q<=0;write_bank_q<=0;next_bank_q<=0;
			tags[0]<=0;tags[1]<=0;frame_q<=0;drop_q<=0;next_col_q<=0;next_row_q<=135;word_addr_q<=0;packed_q<=0;
		end else begin
			ack_meta_q<=ack_q;ack_sync_q<=ack_meta_q;
			if(take_first) begin
				capture_q<=1;write_bank_q<=chosen_bank;frame_q<=i_frame_id;tags[chosen_bank]<=i_frame_id;
				next_col_q<=1;next_row_q<=135;word_addr_q<=0;packed_q<={i_mask,31'd0};
			end else if(first) begin
				capture_q<=0;if(drop_q!=65535)drop_q<=drop_q+1'b1;
			end else if(i_valid && capture_q) begin
				if(!ordered || i_abort) begin capture_q<=0;if(drop_q!=65535)drop_q<=drop_q+1'b1;end
				else begin
					packed_q<=word_data;
					if(i_col==639) begin next_col_q<=0;next_row_q<=i_row+1'b1;end
					else next_col_q<=i_col+1'b1;
					if(i_col[4:0]==31)word_addr_q<=word_addr_q+1'b1;
					if(i_row==319 && i_col==639) begin
						capture_q<=0;completed_q[write_bank_q]<=!completed_q[write_bank_q];next_bank_q<=!write_bank_q;
					end
				end
			end
			if(i_abort)capture_q<=0;
		end
	end
	always @(posedge i_sys_clk or negedge i_sys_rst_n) begin
		if(!i_sys_rst_n) begin ack_q<=0;complete_meta_q<=0;complete_sync_q<=0;reading_q<=0;read_bank_q<=0;read_valid_q<=0;end
		else begin
			complete_meta_q<=completed_q;complete_sync_q<=complete_meta_q;
			read_valid_q<=i_read_req && reading_q;
			if(o_frame_valid && i_frame_ready)reading_q<=1;
			if(i_release && reading_q)begin ack_q[read_bank_q]<=complete_sync_q[read_bank_q];reading_q<=0;read_bank_q<=!read_bank_q;end
		end
	end
endmodule
`default_nettype wire
