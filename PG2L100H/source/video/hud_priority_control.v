//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-12
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在真实 HUD 消隐锁存时原子提交 RK 目标状态，并逐字段仲裁用户方向、去雾灯光接管及 PCIe 请求。
// Module Name: hud_priority_control
// File Name: source/video/hud_priority_control.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在真实 HUD 消隐锁存时原子提交 RK 目标状态，并逐字段仲裁用户方向、去雾灯光接管及 PCIe 请求。
// 用户方向沿用行为窗口保持，去雾保留五灯接管；确认及 override 标志与采样同沿注册，不新增像素延迟。
//
// Dependencies: overlay 真实锁存前置使能 i_latch；已完成 CDC 的控制目标与行为状态。
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
// Create Date: 2026-10-12
// Design Name: RK3566 PCIe HUD priority
// Module Name: hud_priority_control
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 消隐锁存时原子提交RK目标；用户方向优先，去雾保留五灯接管。
//              o_state是本次锁存前组合值，确认与覆盖标志在同沿寄存。
// Dependencies: i_latch为overlay真实采样沿的前置使能；命令已完成跨域校验。
// Revision: 0.01 - File Created
//           0.02 - 2026-10-21 用户方向沿用行为窗口稳亮，保留去雾override回读。
// Additional Comments: 20亮/20灭帧，无像素流水；复位由控制域同步释放。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hud_priority_control
(
	input i_clk,
	input i_rstn,
	input i_latch,
	input i_command_valid,
	input [31:0] i_command,
	input i_dehaze_enable,
	input [4:0] i_dehaze_lights, // {示廓,雾灯,双闪,右转,左转}，含完整灭相
	input [1:0] i_night_level, // 0=白天，1=夜间近光，2=极暗远光
	input i_user_left,
	input i_user_right,
	output [8:0] o_state,
	output o_applied_valid,
	output [31:0] o_applied_request,
	output o_override
);
	//-------------配置参数区域-------------//
	localparam [8:0] DEFAULT_STATE = 9'h101; // 未接管时仅HUD与ADAS可见
	localparam [5:0] LAST_PHASE = 6'd39;    // 40帧周期的最后一帧

	//---------------计数信号---------------//
	reg [5:0] cnt_left_frame;             // 下一次锁存的左转相位索引
	reg [5:0] cnt_right_frame;            // 下一次锁存的右转相位索引
	reg [5:0] cnt_hazard_frame;           // 下一次锁存的双闪相位索引

	//--------------寄存器信号--------------//
	reg [31:0] reg_pending_request;       // 单一在途命令，直到帧边界保持
	reg reg_pending_valid;               // 同沿刚到命令留待下一帧
	reg [31:0] reg_active_request;        // 最近提交的完整RK请求
	reg [1:0] reg_left_mode;              // {当前可见使能,闪烁}，双闪占用时为0
	reg [1:0] reg_right_mode;             // {当前可见使能,闪烁}
	reg [1:0] reg_hazard_mode;            // {双闪使能,闪烁}

	//---------------其他信号---------------//
	wire [31:0] selected_request;         // 本次overlay采样目标
	wire [8:0] selected_state;            // 未接管采用原默认值
	wire [1:0] left_mode;
	wire [1:0] right_mode;
	wire [1:0] hazard_mode;
	wire flag_left_on;
	wire flag_right_on;
	wire flag_hazard_on;

	//---------------输出信号---------------//
	reg [8:0] state_o;
	reg applied_valid_o;                 // 提交记录有效，持续到控制复位
	reg [31:0] applied_request_o;
	reg override_o;                      // 与overlay锁存同沿采样的去雾标志

	//-------------其他信号连线-------------//
	assign selected_request = (i_latch && reg_pending_valid) ? reg_pending_request : reg_active_request;
	assign selected_state = selected_request[12] ? selected_request[8:0] : DEFAULT_STATE;
	assign hazard_mode = {selected_request[12] && selected_state[3], selected_request[11]};
	assign left_mode = {selected_request[12] && selected_state[1] && !selected_state[3], selected_request[9] && !selected_state[3]};
	assign right_mode = {selected_request[12] && selected_state[2] && !selected_state[3], selected_request[10] && !selected_state[3]};
	assign flag_left_on = left_mode[0] ? ((left_mode != reg_left_mode) || override_o || (cnt_left_frame < 6'd20)) : 1'b1;
	assign flag_right_on = right_mode[0] ? ((right_mode != reg_right_mode) || override_o || (cnt_right_frame < 6'd20)) : 1'b1;
	assign flag_hazard_on = hazard_mode[0] ? ((hazard_mode != reg_hazard_mode) || override_o || (cnt_hazard_frame < 6'd20)) : 1'b1;

	//-------------输出信号连线-------------//
	assign o_state = state_o;
	assign o_applied_valid = applied_valid_o;
	assign o_applied_request = applied_request_o;
	assign o_override = override_o;

	//-----------输出信号处理区域-----------//
	// 去雾选择器完整接管亮/灭相；近光、远光和ADAS始终来自低优先级目标。
	always@(*)begin
		state_o = selected_state;
		if(hazard_mode[1])begin
			state_o[3:1] = {3{flag_hazard_on}};
		end else begin
			state_o[1] = left_mode[1] && flag_left_on;
			state_o[2] = right_mode[1] && flag_right_on;
			state_o[3] = 1'b0;
		end
		if(i_dehaze_enable)begin
			state_o[0] = 1'b1;
			state_o[3:1] = i_dehaze_lights[2:0];
			state_o[7:6] = i_dehaze_lights[4:3];
		end
		// ISP 自适应夜间控制只补充灯光字段，不改写用户方向和行为状态。
		// 位置灯与近光在夜间开启，极暗等级再开启远光；退出夜间后回到
		// selected_state，避免自动状态残留到下一帧。灯光字段在这里统一
		// 仲裁，确保夜间状态不依赖去雾按钮或去雾流的内容有效位。
		if(i_night_level != 2'd0)begin
			state_o[0] = 1'b1;
			state_o[4] = 1'b1;
			state_o[7] = 1'b1;
			if(i_night_level == 2'd2) state_o[5] = 1'b1;
		end
		// 同一份行为快照在overlay帧边界锁存；用户意图不改变允许/禁止判定。
		if(i_user_left || i_user_right)begin
			state_o[0] = 1'b1;
			state_o[3:1] = {1'b0, i_user_right && !i_user_left, i_user_left};
		end
	end

	// 提交记录与overlay采样同一沿；序号/无关字段变化不参与闪烁模式比较。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			applied_valid_o <= 1'b0;
			applied_request_o <= 32'd0;
		end else if(i_latch && reg_pending_valid)begin
			applied_valid_o <= 1'b1;
			applied_request_o <= reg_pending_request;
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			override_o <= 1'b0;
		end else if(i_latch)begin
			override_o <= i_dehaze_enable;
		end
	end

	//-----------主要任务处理区域-----------//
	// pending先提交再接收；同沿有效新命令保存到下一次锁存，不覆盖旧提交。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_pending_request <= 32'd0;
			reg_pending_valid <= 1'b0;
		end else begin
			if(i_latch && reg_pending_valid)begin
				reg_pending_valid <= 1'b0;
			end
			if(i_command_valid && (!reg_pending_valid || i_latch))begin
				reg_pending_request <= i_command;
				reg_pending_valid <= 1'b1;
			end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_active_request <= 32'd0;
		end else if(i_latch && reg_pending_valid)begin
			reg_active_request <= reg_pending_request;
		end
	end

	// 模式仅在帧边界更新，临时输入变化不影响已锁存相位。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_left_mode <= 2'd0;
			reg_right_mode <= 2'd0;
			reg_hazard_mode <= 2'd0;
		end else if(i_latch)begin
			reg_left_mode <= left_mode;
			reg_right_mode <= right_mode;
			reg_hazard_mode <= hazard_mode;
		end
	end

	// 保存下一帧相位：首帧已输出亮，随后索引设1，确保准确20亮/20灭。
	// 去雾占用时不推进RK相位；双闪占用左右时其计数清零，退出从亮相恢复。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_left_frame <= 6'd0;
		end else if(i_latch)begin
			if(i_dehaze_enable || (left_mode != 2'b11))begin
				cnt_left_frame <= 6'd0;
			end else if((left_mode != reg_left_mode) || override_o)begin
				cnt_left_frame <= 6'd1;
			end else if(cnt_left_frame == LAST_PHASE)begin
				cnt_left_frame <= 6'd0;
			end else begin
				cnt_left_frame <= cnt_left_frame + 6'd1;
			end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_right_frame <= 6'd0;
		end else if(i_latch)begin
			if(i_dehaze_enable || (right_mode != 2'b11))begin
				cnt_right_frame <= 6'd0;
			end else if((right_mode != reg_right_mode) || override_o)begin
				cnt_right_frame <= 6'd1;
			end else if(cnt_right_frame == LAST_PHASE)begin
				cnt_right_frame <= 6'd0;
			end else begin
				cnt_right_frame <= cnt_right_frame + 6'd1;
			end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_hazard_frame <= 6'd0;
		end else if(i_latch)begin
			if(i_dehaze_enable || (hazard_mode != 2'b11))begin
				cnt_hazard_frame <= 6'd0;
			end else if((hazard_mode != reg_hazard_mode) || override_o)begin
				cnt_hazard_frame <= 6'd1;
			end else if(cnt_hazard_frame == LAST_PHASE)begin
				cnt_hazard_frame <= 6'd0;
			end else begin
				cnt_hazard_frame <= cnt_hazard_frame + 6'd1;
			end
		end
	end
endmodule
