//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 根据最终像素算法配置产生 HUD 道路灯状态，去雾开启时接管示廓/雾灯，双闪保持同相。
// Module Name: hdmi_hud_road_lights
// File Name: source/video/hdmi_hud_road_lights.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 根据最终像素算法配置产生 HUD 道路灯状态，去雾开启时接管示廓/雾灯，双闪保持同相。
// 默认 20 帧亮、20 帧灭，组合结果由 overlay 在真实消隐锁存沿采样，不增加像素流水延迟。
//
// Dependencies: 同域帧节拍、去雾配置与 hdmi_hud_overlay 锁存。
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
// Design Name: HDMI Dehaze Road HUD
// Module Name: hdmi_hud_road_lights
// Project Name: Ethernet_ctl_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 最终像素配置驱动道路灯光；去雾开启时示廓/雾灯常亮，双闪同相。
// Dependencies: 同域hdmi_hud_overlay在VS下降后一拍消隐锁存本模块组合输出。
// Revision: 0.01 - File Created
// Additional Comments: 60fps默认亮20帧/灭20帧；参数范围1..32768。
//                      输出不串入像素流水，不增加视频延迟；无新增IP。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hdmi_hud_road_lights
#(
	parameter integer C_HAZARD_HALF_FRAMES = 20
)
(
	input i_clk,
	input i_rst_n,
	input i_vs,
	input i_dehaze_enable,
	output o_left,
	output o_right,
	output o_hazard,
	output o_front_fog,
	output o_position
);
	// 下一次HUD锁存使用的相位索引；首个开启帧由previous_enable直接选亮。
	reg reg_vs_previous;
	reg reg_frame_latch_pending;
	reg reg_enable_previous;
	reg [15:0] cnt_phase_frame;
	wire frame_latch;
	wire hazard_on;

	assign frame_latch = reg_frame_latch_pending;
	assign hazard_on = i_dehaze_enable &&
		(!reg_enable_previous || (cnt_phase_frame < C_HAZARD_HALF_FRAMES));
	assign o_left = hazard_on;
	assign o_right = hazard_on;
	assign o_hazard = hazard_on;
	assign o_front_fog = i_dehaze_enable;
	assign o_position = i_dehaze_enable;

	// HUD与本模块在同一沿采样；计数器保存下一帧的索引，避免首相位多一帧。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_vs_previous <= 1'b0;
			reg_frame_latch_pending <= 1'b0;
		end else begin
			reg_vs_previous <= i_vs;
			reg_frame_latch_pending <= reg_vs_previous && !i_vs;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_enable_previous <= 1'b0;
		end else if(frame_latch)begin
			reg_enable_previous <= i_dehaze_enable;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_phase_frame <= 16'd0;
		end else if(frame_latch)begin
			if(!i_dehaze_enable)begin
				cnt_phase_frame <= 16'd0;
			end else if(!reg_enable_previous)begin
				cnt_phase_frame <= 16'd1;
			end else if(cnt_phase_frame == 2 * C_HAZARD_HALF_FRAMES - 1)begin
				cnt_phase_frame <= 16'd0;
			end else begin
				cnt_phase_frame <= cnt_phase_frame + 16'd1;
			end
		end
	end
endmodule
