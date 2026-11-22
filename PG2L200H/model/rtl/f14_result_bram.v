//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 把三检测头共609个128位事务映射到共享结果DRM，按head及局部地址写入cls/vertical/offset区域。
// Module Name: f14_result_bram
// File Name: model/rtl/f14_result_bram.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 把三检测头共609个128位事务映射到共享结果DRM，按head及局部地址写入cls/vertical/offset区域。
// 读口把线性CHW地址转换为事务地址与16位lane，三拍返回；越界或上游error请求返回零。
//
// Dependencies: f14_explicit_drm_ram：128位×1024
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 事务布局cls[0:293]、vertical[294:314]、offset[315:608]；复位清valid流水，不清结果存储内容。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 三头结果存储；609个128位事务，共用4块官方DRM，三拍CHW读回。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module f14_result_bram (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_wr_en,
	input wire [1:0] i_head,
	input wire [10:0] i_wr_addr,
	input wire [127:0] i_wr_data,
	input wire i_error,
	input wire i_req,
	input wire [12:0] i_addr,
	output wire o_valid,
	output wire [15:0] o_data
);
	// 事务布局：cls[0:293]、vertical[294:314]、offset[315:608]。
	wire legal_write = i_wr_en && i_rst_n &&
		(((i_head == 0 || i_head == 2) && i_wr_addr < 294) || (i_head == 1 && i_wr_addr < 21));
	wire [9:0] write_addr = (i_head == 0) ? i_wr_addr[9:0] :
		((i_head == 1) ? (10'd294 + i_wr_addr[9:0]) : (10'd315 + i_wr_addr[9:0]));
	reg [11:0] local_index, pixel_index;
	reg [1:0] channel;
	reg [9:0] read_addr;
	reg [2:0] lane;
	reg [9:0] read_addr_q;
	reg [2:0] lane1_q, lane2_q;
	reg [2:0] valid_q, invalid_q;
	reg [15:0] data_q;
	wire [127:0] ram_data;
	always @(*) begin
		local_index = 0; pixel_index = 0; channel = 0; read_addr = 0; lane = 0;
		if (i_addr >= 2352 && i_addr < 2436) begin
			local_index = i_addr - 13'd2352;
			if (local_index >= 63) begin channel = 3; pixel_index = local_index - 12'd63; end
			else if (local_index >= 42) begin channel = 2; pixel_index = local_index - 12'd42; end
			else if (local_index >= 21) begin channel = 1; pixel_index = local_index - 12'd21; end
			else pixel_index = local_index;
			read_addr = 10'd294 + pixel_index[9:0];
			lane = {channel,1'b0};
		end else begin
			local_index = (i_addr < 2352) ? i_addr[11:0] : (i_addr - 13'd2436);
			if (local_index >= 1764) begin channel = 3; pixel_index = local_index - 12'd1764; end
			else if (local_index >= 1176) begin channel = 2; pixel_index = local_index - 12'd1176; end
			else if (local_index >= 588) begin channel = 1; pixel_index = local_index - 12'd588; end
			else pixel_index = local_index;
			read_addr = {1'b0,pixel_index[9:1]} + ((i_addr < 2352) ? 10'd0 : 10'd315);
			lane = {channel,pixel_index[0]};
		end
	end
	f14_explicit_drm_ram #(.C_ADDR_WIDTH(10),.C_DATA_WIDTH(128),.C_DEPTH(1024)) u_result_mem (
		.i_clk(i_clk),.i_wr_en(legal_write),.i_wr_addr(write_addr),.i_wr_data(i_wr_data),
		.i_rd_en(valid_q[0] && !invalid_q[0]),.i_rd_addr(read_addr_q),.o_rd_data(ram_data)
	);
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			valid_q <= 0; invalid_q <= 0;
			read_addr_q <= 0; lane1_q <= 0; lane2_q <= 0; data_q <= 0;
		end else begin
			valid_q <= {valid_q[1:0],i_req};
			invalid_q <= {invalid_q[1:0],(i_addr >= 4788 || i_error)};
			if (i_req) begin read_addr_q <= read_addr; lane1_q <= lane; end
			if (valid_q[0]) lane2_q <= lane1_q;
			if (valid_q[1]) data_q <= (invalid_q[1] || i_error) ? 16'd0 : ram_data[lane2_q*16+:16];
		end
	end
	assign o_valid = valid_q[2];
	assign o_data = data_q;
endmodule
