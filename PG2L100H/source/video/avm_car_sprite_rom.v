//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-17
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 为 AVM 中央车身图像提供 RGB565 及透明掩码 ROM 适配，坐标对应车头朝上的已生成位图。
// Module Name: avm_car_sprite_rom_adapter
// File Name: source/video/avm_car_sprite_rom.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 为 AVM 中央车身图像提供 RGB565 及透明掩码 ROM 适配，坐标对应车头朝上的已生成位图。
// 将图像读取委托给官方 avm_car_sprite_rom；本文件真实模块名为 avm_car_sprite_rom_adapter。
//
// Dependencies: 官方 avm_car_sprite_rom；assets/avm/car_sprite/car_rgb565_mask17_head_up.dat。
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
// Create Date: 2026-10-17
// Module Name: avm_car_sprite_rom_adapter
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 图二车辆位图的 RGB565 + 透明掩码 ROM；输入坐标已按车头向上生成。
//              数据文件由 tools/generate_car_sprite.ps1 生成，替换图片时重新生成即可。
// Dependencies: official ipcore/avm_car_sprite_rom/avm_car_sprite_rom.v
// Revision: 0.01 - 删除比较器车辆图形，改为图像位图读取。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_car_sprite_rom_adapter(
	input i_clk,
	input i_rst_n,
	input [14:0] i_addr,
	output [15:0] o_rgb565,
	output o_valid
);
	wire [16:0] sprite_data;
	avm_car_sprite_rom u_official_car_sprite_rom(
		.addr(i_addr),
		.rd_data(sprite_data),
		.clk(i_clk),
		.rst(!i_rst_n)
	);
	assign o_rgb565 = sprite_data[15:0];
	assign o_valid = sprite_data[16];
endmodule
