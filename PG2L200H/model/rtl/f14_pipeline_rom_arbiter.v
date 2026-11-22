//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 仲裁骨干、head和boot三路共享运行资产ROM请求，优先级为骨干→head→boot。
// Module Name: f14_pipeline_rom_arbiter
// File Name: model/rtl/f14_pipeline_rom_arbiter.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 仲裁骨干、head和boot三路共享运行资产ROM请求，优先级为骨干→head→boot。
// 寄存请求地址/使能及owner，再按响应owner分发32位数据和valid，避免不同消费者混收返回。
//
// Dependencies: f14_runtime_asset_rom
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 骨干请求无背压；head/boot仅在valid&&ready时推进；复位清除请求和owner流水。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: PG2L200H-6FBB484; Tool: PDS 2022.2-SP6.4
// 骨干请求无背压；head/boot仅在valid&&ready推进。仲裁后地址/使能寄存。
`timescale 1ns/1ps
`default_nettype none
module f14_pipeline_rom_arbiter (
	input wire i_clk,input wire i_rst_n,
	input wire i_bb_valid,input wire [16:0] i_bb_addr,
	output wire o_bb_valid,output wire [31:0] o_bb_data,
	input wire i_head_valid,input wire [16:0] i_head_addr,output wire o_head_ready,
	output wire o_head_valid,output wire [31:0] o_head_data,
	input wire i_boot_valid,input wire [16:0] i_boot_addr,output wire o_boot_ready,
	output wire o_boot_valid,output wire [31:0] o_boot_data
);
	reg request_q;
	reg [16:0] address_q;
	reg [1:0] owner_q,response_owner_q;
	wire valid;
	wire [31:0] data;
	assign o_head_ready=i_rst_n && !i_bb_valid;
	assign o_boot_ready=i_rst_n && !i_bb_valid && !i_head_valid;
	always @(posedge i_clk) begin
		if(!i_rst_n)begin request_q<=0;owner_q<=0;response_owner_q<=0;end
		else begin
			request_q<=i_bb_valid || i_head_valid || i_boot_valid;
			if(i_bb_valid)begin address_q<=i_bb_addr;owner_q<=0;end
			else if(i_head_valid)begin address_q<=i_head_addr;owner_q<=1;end
			else if(i_boot_valid)begin address_q<=i_boot_addr;owner_q<=2;end
			if(request_q) response_owner_q<=owner_q;
		end
	end
	f14_runtime_asset_rom u_shared_rom (
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_rd_en(request_q),.i_rd_addr(address_q),.o_rd_valid(valid),.o_rd_data(data));
	assign o_bb_valid=valid && response_owner_q==0;
	assign o_head_valid=valid && response_owner_q==1;
	assign o_boot_valid=valid && response_owner_q==2;
	assign o_bb_data=data;
	assign o_head_data=data;
	assign o_boot_data=data;
endmodule
`default_nettype wire
