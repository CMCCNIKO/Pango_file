//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在算法 tail 之后、HUD 之前根据真实像素 tag 坐标选择 AVM、监视源或算法图像，并在中央黑区叠加带透明掩码的车身位图。
// Module Name: avm_monitor_compositor
// File Name: source/video/avm_monitor_compositor.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在算法 tail 之后、HUD 之前根据真实像素 tag 坐标选择 AVM、监视源或算法图像，并在中央黑区叠加带透明掩码的车身位图。
// RGB 与标签注册一拍，路由应用事件由真实输出锁存产生；prepared 描述符和 FIFO 必须在输出原点前稳定。
//
// Dependencies: avm_car_sprite_rom_adapter；上层 AVM/monitor 像素与路由描述符。
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
// Create Date: 2026-10-13
// Module Name: avm_monitor_compositor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: tail之后HUD之前；真实tag坐标选择AVM/monitor或算法；注册一拍；中央黑区叠加车辆位图。
// Contract: prepared描述符/FIFO必须在输出x0/y0之前稳定；应用事件由输出锁存产生。
// Revision: 0.01; 中间态可C_LEFT_AVM_ENABLE=0保留左公共像素，不能作为最终AVM。
//////////////////////////////////////////////////////////////////////////////////
module avm_monitor_compositor #(
	parameter integer C_LEFT_AVM_ENABLE=1,
	parameter integer C_EXTERNAL_COMMIT=0,
	// 车辆位图开关；图像尺寸和坐标由下列参数控制，图像数据在外部ROM mem文件中。
	parameter integer C_CENTER_CAR_ENABLE=1
)(
	input i_clk,
	input i_rst_n,
	input [159:0] i_tag,
	input [23:0] i_rgb,
	input [8:0] i_hud_state,
	input i_prepared_valid,
	input [35:0] i_prepared_route,
	input i_route_applied,
	input [35:0] i_active_route,
	input [15:0] i_monitor_rgb565,
	input i_monitor_valid,
	input [15:0] i_avm_rgb565,
	input i_avm_valid,
	output o_monitor_pop,
	output o_avm_pop,
	output [159:0] o_tag,
	output [23:0] o_rgb,
	output [8:0] o_hud_state,
	output o_active_valid,
	output [35:0] o_active_route
);
	wire [10:0] x=i_tag[154:144];
	wire [9:0] y=i_tag[143:134];
	wire active_area=i_tag[157] && y>=40 && y<680;
	wire monitor_area=active_area && x>=640;
	wire avm_area=active_area && x>=80 && x<560;
	wire center_area=avm_area;
	wire output_boundary=i_tag[157] && x==0 && y==0;
	reg [35:0] active_route_o;
	reg active_valid_o;
	reg route_enabled;
	reg display_algo_enabled;
	reg commit_sent;
	reg [159:0] tag_o;
	reg [23:0] rgb_o;
	reg [8:0] hud_state_o;
	reg [23:0] rgb_selected;
	wire car_pixel;
	wire car_area;
	wire [10:0] car_x;
	wire [9:0] car_y;
	wire [14:0] car_addr;
	wire [15:0] car_rgb565;
	wire car_mask_valid;
	wire [23:0] car_rgb;
	localparam integer C_CAR_X = 272;
	localparam integer C_CAR_Y = 240;
	localparam integer C_CAR_WIDTH = 96;
	localparam integer C_CAR_HEIGHT = 240;
	wire [15:0] selected565=monitor_area ? i_monitor_rgb565 : i_avm_rgb565;
	wire [23:0] expanded565={selected565[15:11],selected565[15:13],selected565[10:5],selected565[10:9],selected565[4:0],selected565[4:2]};
	assign o_monitor_pop=monitor_area && route_enabled && !display_algo_enabled;
	assign o_avm_pop=avm_area && C_LEFT_AVM_ENABLE && !display_algo_enabled;
	assign o_tag=tag_o;
	assign o_rgb=rgb_o;
	assign o_hud_state=hud_state_o;
	assign o_active_valid=active_valid_o;
	assign o_active_route=active_route_o;
	assign car_area = C_CENTER_CAR_ENABLE && center_area &&
		(x >= C_CAR_X) && (x < C_CAR_X + C_CAR_WIDTH) &&
		(y >= C_CAR_Y) && (y < C_CAR_Y + C_CAR_HEIGHT);
	assign car_x = car_area ? x - C_CAR_X : 11'd0;
	assign car_y = car_area ? y - C_CAR_Y : 10'd0;
	assign car_addr = car_area ? (({5'd0, car_y} << 6) + ({5'd0, car_y} << 5) + car_x) : 15'd0;
	assign car_pixel = car_area && car_mask_valid;
	assign car_rgb = {car_rgb565[15:11], car_rgb565[15:13],
		car_rgb565[10:5], car_rgb565[10:9],
		car_rgb565[4:0], car_rgb565[4:2]};
	avm_car_sprite_rom_adapter
		avm_car_sprite_rom_Inst_sprite(
		.i_clk(i_clk), .i_rst_n(i_rst_n), .i_addr(car_addr),
		.o_rgb565(car_rgb565), .o_valid(car_mask_valid));
	always @(*) begin
		rgb_selected=24'd0;
		if(active_area) begin
			if(display_algo_enabled) begin
				rgb_selected=i_rgb;
			end else if(monitor_area) begin
				if(route_enabled && i_monitor_valid) rgb_selected=expanded565;
			end else if(C_LEFT_AVM_ENABLE) begin
				if(avm_area && i_avm_valid) rgb_selected=expanded565;
			end else rgb_selected=i_rgb;
			// 图二车辆位图只覆盖 AVM 投影中心，透明像素保留底图。
			if(car_pixel && !display_algo_enabled) rgb_selected=car_rgb;
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			tag_o<=0; rgb_o<=0; hud_state_o<=0;
			// 默认路由与执行规格一致；像素消费必须等当前帧描述符及FIFO准备好。
			// HDMI时序不受此门控影响；未prepared时只输出黑色，不读取旧FIFO。
			active_route_o<={16'd0,2'd3,2'd2,16'd0}; active_valid_o<=0; route_enabled<=0; display_algo_enabled<=0; commit_sent<=0;
		end else begin
			tag_o<=i_tag; rgb_o<=rgb_selected; hud_state_o<=i_hud_state;
			if(display_algo_enabled) tag_o[112]<=i_tag[112];
			else if(monitor_area) tag_o[112]<=route_enabled && i_monitor_valid;
			else if(C_LEFT_AVM_ENABLE) tag_o[112]<=avm_area && i_avm_valid;
			active_valid_o<=0;
			if(output_boundary) begin commit_sent<=0; route_enabled<=0; end
			// 每帧仅提交一次；prepared反馈清除存在一拍延迟，不得重复翻转CDC toggle。
			// 帧边界只锁存已准备好的整体描述符；没有新描述符时保留当前路由。
			// 不能在边界先清零，否则单周期 prepared_valid 可能被错过而整屏黑屏。
			if(C_EXTERNAL_COMMIT && i_route_applied) begin
				active_route_o<=i_active_route;
				active_valid_o<=1;
				route_enabled<=1;
				display_algo_enabled<=i_active_route[15];
			end else if(!C_EXTERNAL_COMMIT && !commit_sent && !output_boundary && i_prepared_valid &&
					((i_tag[157] && y<40) || monitor_area)) begin
				commit_sent<=1;
				active_route_o<=i_prepared_route;
				active_valid_o<=1;
				route_enabled<=1;
				display_algo_enabled<=i_prepared_route[15];
			end
		end
	end
endmodule
