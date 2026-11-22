//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-12
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 连接 BAR1 HUD 寄存器、双向 CDC、扩展场景/行为快照及 HSST 车道信息，生成像素域 HUD 有效状态。
// Module Name: hud_control_subsystem
// File Name: source/video/hud_control_subsystem.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 连接 BAR1 HUD 寄存器、双向 CDC、扩展场景/行为快照及 HSST 车道信息，生成像素域 HUD 有效状态。
// 逐字段仲裁用户、去雾和 PCIe 控制，使用真实显示锁存确认及 latest-pending 快照；控制会话断链不复位像素流水。
//
// Dependencies: pcie_hud_regs、hud_control_cdc、hud_ext_control_cdc、hud_priority_control、fiber_lane_hud_cdc、overlay 锁存回报。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 2026-11-12 / RK左右车道逐侧优先，未识别侧回退既有光口显示和判定。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-12
// Design Name: RK3566 PCIe HUD subsystem
// Module Name: hud_control_subsystem
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: BAR1 HUD事务、双向CDC和用户/去雾/PCIe逐字段仲裁，独立于视频DMA。
// Dependencies: pcie_hud_regs (v2, v1 CSR-compatible), hud_control_cdc,
// hud_ext_control_cdc, hud_priority_control, fiber_lane_hud_cdc, v2 overlay锁存。
// Revision: 0.01 - File Created
// Revision: 0.02 - 2026-10-21 176位场景/行为快照，单一latest-pending及真实帧确认。
// Revision: 0.03 - 2026-10-21 复用像素域行为窗口驱动用户转向，不新增CDC或计时器。
// Revision: 0.04 - 2026-10-23 独立HSST车道CDC，统一像素域线型及变道结果。
// Additional Comments: 控制断链复位不传递到像素流水；没有新增可配置IP。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hud_control_subsystem
(
	input i_pcie_clk,
	input i_pcie_rstn,
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_lane_sys_clk,
	input i_lane_sys_rst_n,
	input [31:0] i_fiber_lane_payload,
	input i_fiber_lane_valid,
	input i_fiber_lane_stale,
	input i_link_up,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window,
	output o_scene_commit_valid,
	output [111:0] o_scene_commit_payload,
	output o_control_pcie_rstn,
	input [175:0] i_display_snapshot,
	input i_latch,
	input i_latched,
	input [8:0] i_latched_state,
	input i_dehaze_enable,
	input [4:0] i_dehaze_lights,
	input [1:0] i_night_level,
	output [8:0] o_state,
	output [31:0] o_ext_state,
	output [31:0] o_time0,
	output [31:0] o_time1,
	output o_ext_valid,
	output [31:0] o_behavior_result,
	output [31:0] o_behavior_effective,
	output [31:0] o_nav_state,
	output [19:0] o_nav_bcd,
	output [15:0] o_nav_seq,
	output [15:0] o_nav_token,
	output o_nav_valid,
	input [31:0] i_nav_latched_state,
	input [15:0] i_nav_latched_seq,
	input [15:0] i_nav_latched_token,
	input i_nav_latched_valid
);
	localparam [175:0] C_DEFAULT_DISPLAY = {16'd0, 32'h00235716,
		32'h10102026, 32'h00000a40, 32'd0, 32'd0};
	wire control_resetn;
	reg [2:0] reg_pcie_reset_sync;
	reg [2:0] reg_pixel_reset_sync;
	wire control_pcie_rstn;
	wire control_pixel_rstn;
	wire flag_ready;
	wire flag_send;
	wire [31:0] request;
	wire flag_command_valid;
	wire [31:0] command;
	wire flag_applied_valid;
	wire [31:0] applied_request;
	wire flag_override;
	wire flag_snapshot_valid;
	wire [123:0] snapshot;
	wire [123:0] pixel_snapshot;
	wire nav_ready;
	wire nav_send;
	wire [83:0] nav_payload;
	wire nav_command_valid;
	wire [83:0] nav_command;
	reg [83:0] reg_nav_payload;
	reg nav_valid_o;
	wire ext_ready;
	wire ext_send;
	wire ext_command_valid;
	wire [175:0] ext_command;
	reg [175:0] reg_display_seen;
	reg [175:0] reg_display_payload;
	reg reg_display_pending;
	reg [31:0] ext_state_o;
	reg [31:0] time0_o;
	reg [31:0] time1_o;
	reg ext_valid_o;
	reg [31:0] behavior_result_o;
	reg [31:0] behavior_effective_o;
	reg [15:0] reg_ext_pixel_seq;
	reg [15:0] reg_ext_latched_seq;
	wire flag_user_left;
	wire flag_user_right;
	wire [4:0] fiber_lane_state;
	wire flag_rk_lane_fresh;
	wire flag_rk_left_valid;
	wire flag_rk_right_valid;
	wire flag_fiber_left_valid;
	wire flag_fiber_right_valid;
	wire flag_left_lane_fresh;
	wire flag_right_lane_fresh;
	wire [1:0] selected_left_lane;
	wire [1:0] selected_right_lane;
	wire [31:0] selected_ext_state;
	reg [31:0] selected_behavior_result;
	reg [31:0] selected_behavior_effective;

	// 链路/任一域复位协调断言；只释放控制域，不重置HDMI/去雾视频。
	assign control_resetn = i_pcie_rstn && i_pixel_rstn && i_link_up;
	assign control_pcie_rstn = reg_pcie_reset_sync[2];
	assign control_pixel_rstn = reg_pixel_reset_sync[2];
	assign o_control_pcie_rstn = control_pcie_rstn;
	assign pixel_snapshot = {i_nav_latched_valid, i_nav_latched_token, i_nav_latched_seq, i_nav_latched_state, reg_ext_latched_seq, flag_applied_valid, applied_request, flag_override, i_latched_state};
	assign o_nav_state = reg_nav_payload[31:0];
	assign o_nav_bcd = reg_nav_payload[51:32];
	assign o_nav_seq = reg_nav_payload[67:52];
	assign o_nav_token = reg_nav_payload[83:68];
	assign o_nav_valid = nav_valid_o;
	assign o_ext_state = selected_ext_state;
	assign o_time0 = time0_o;
	assign o_time1 = time1_o;
	assign o_ext_valid = ext_valid_o;
	assign o_behavior_result = selected_behavior_result;
	assign o_behavior_effective = selected_behavior_effective;
	// RK bit0表示实际新鲜度；静态初值和掉链会话不构成有效识别。
	assign flag_rk_lane_fresh = control_pixel_rstn && ext_state_o[0];
	assign flag_rk_left_valid = flag_rk_lane_fresh &&
		(ext_state_o[7:6] == 2'd1 || ext_state_o[7:6] == 2'd2);
	assign flag_rk_right_valid = flag_rk_lane_fresh &&
		(ext_state_o[9:8] == 2'd1 || ext_state_o[9:8] == 2'd2);
	assign flag_fiber_left_valid = fiber_lane_state[4] &&
		(fiber_lane_state[1:0] == 2'd1 || fiber_lane_state[1:0] == 2'd2);
	assign flag_fiber_right_valid = fiber_lane_state[4] &&
		(fiber_lane_state[3:2] == 2'd1 || fiber_lane_state[3:2] == 2'd2);
	// RK controls each side independently; an unrecognized side falls back
	// to the existing fiber lane source for that side.
	assign selected_left_lane = flag_rk_left_valid ? ext_state_o[7:6] :
		(flag_fiber_left_valid ? fiber_lane_state[1:0] : 2'd0);
	assign selected_right_lane = flag_rk_right_valid ? ext_state_o[9:8] :
		(flag_fiber_right_valid ? fiber_lane_state[3:2] : 2'd0);
	assign flag_left_lane_fresh = flag_rk_left_valid || flag_fiber_left_valid;
	assign flag_right_lane_fresh = flag_rk_right_valid || flag_fiber_right_valid;
	assign selected_ext_state = {ext_state_o[31:10], selected_right_lane,
		selected_left_lane, ext_state_o[5:0]};
	// 允许和禁止都保留用户方向；高字节序号在窗口结束后仍可非零。
	assign flag_user_left = behavior_result_o[14] && (behavior_result_o[2] || behavior_result_o[6]);
	assign flag_user_right = behavior_result_o[14] && (behavior_result_o[3] || behavior_result_o[7]);
	assign ext_send = control_pcie_rstn && reg_display_pending && ext_ready;

	// 线型与左右动作同源组合选择，overlay在同一VS消隐沿原子锁存。
	// 其余动作、序号及1秒窗口由既有快照保持，不增加像素流水或计时器。
	always@(*)begin
		selected_behavior_result = behavior_result_o;
		selected_behavior_effective = behavior_effective_o;
		if(flag_user_left)begin
			selected_behavior_result[15:0] = 16'h4000;
			selected_behavior_result[15] = !flag_left_lane_fresh;
			if(selected_left_lane == 2'd2)begin
				selected_behavior_result[2] = 1'b1;
			end else begin
				selected_behavior_result[6] = 1'b1;
				selected_behavior_result[13:8] = selected_left_lane == 2'd1 ? 6'd2 : 6'd6;
			end
			selected_behavior_effective = selected_behavior_result;
		end else if(flag_user_right)begin
			selected_behavior_result[15:0] = 16'h4000;
			selected_behavior_result[15] = !flag_right_lane_fresh;
			if(selected_right_lane == 2'd2)begin
				selected_behavior_result[3] = 1'b1;
			end else begin
				selected_behavior_result[7] = 1'b1;
				selected_behavior_result[13:8] = selected_right_lane == 2'd1 ? 6'd3 : 6'd6;
			end
			selected_behavior_effective = selected_behavior_result;
		end
	end

	// 直接从独立系统快照进入像素域；PCIe掉链或停钟不停止车道更新。
	fiber_lane_hud_cdc fiber_lane_hud_cdc_Inst_lane(
		.i_sys_clk(i_lane_sys_clk),
		.i_sys_rst_n(i_lane_sys_rst_n),
		.i_payload(i_fiber_lane_payload),
		.i_valid(i_fiber_lane_valid),
		.i_stale(i_fiber_lane_stale),
		.i_pixel_clk(i_pixel_clk),
		.i_pixel_rst_n(i_pixel_rstn),
		.o_lane_state(fiber_lane_state)
	);
	always@(posedge i_pcie_clk or negedge control_resetn)begin
		if(!control_resetn)begin
			reg_pcie_reset_sync <= 3'd0;
		end else begin
			reg_pcie_reset_sync <= {reg_pcie_reset_sync[1:0], 1'b1};
		end
	end
	always@(posedge i_pixel_clk or negedge control_resetn)begin
		if(!control_resetn)begin
			reg_pixel_reset_sync <= 3'd0;
		end else begin
			reg_pixel_reset_sync <= {reg_pixel_reset_sync[1:0], 1'b1};
		end
	end

	pcie_hud_regs pcie_hud_regs_Inst_regs(
		.i_clk(i_pcie_clk),
		.i_rstn(control_pcie_rstn),
		.i_ready(flag_ready),
		.i_wr_en(i_wr_en),
		.i_wr_addr(i_wr_addr),
		.i_wr_data(i_wr_data),
		.i_wr_strb(i_wr_strb),
		.i_rd_addr(i_rd_addr),
		.o_rd_window(o_rd_window),
		.o_send(flag_send),
		.o_request(request),
		.i_ext_ready(ext_ready),
		.i_ext_pending(reg_display_pending),
		.o_ext_commit_valid(o_scene_commit_valid),
		.o_ext_commit_payload(o_scene_commit_payload),
		.i_snapshot_valid(flag_snapshot_valid),
		.i_snapshot(snapshot),
		.i_nav_ready(nav_ready),
		.o_nav_send(nav_send),
		.o_nav_payload(nav_payload)
	);
	// 在途载荷由CDC保持；新快照只覆盖latest-pending，发送同沿更新优先保留。
	always@(posedge i_pcie_clk or negedge control_pcie_rstn)begin
		if(!control_pcie_rstn)begin
			reg_display_seen <= C_DEFAULT_DISPLAY;
			reg_display_payload <= C_DEFAULT_DISPLAY;
			reg_display_pending <= 1'b0;
		end else begin
			if(ext_send)begin
				reg_display_pending <= 1'b0;
			end
			if(i_display_snapshot != reg_display_seen)begin
				reg_display_seen <= i_display_snapshot;
				reg_display_payload <= i_display_snapshot;
				reg_display_pending <= 1'b1;
			end
		end
	end
	hud_ext_control_cdc #(.C_PAYLOAD_WIDTH(176)) hud_ext_control_cdc_Inst_ext(
		.i_pcie_clk(i_pcie_clk), .i_pcie_rstn(control_pcie_rstn),
		.i_pixel_clk(i_pixel_clk), .i_pixel_rstn(control_pixel_rstn),
		.i_send(ext_send), .i_payload(reg_display_payload), .o_ready(ext_ready),
		.o_command_valid(ext_command_valid), .o_command(ext_command));
	always@(posedge i_pixel_clk or negedge control_pixel_rstn)begin
		if(!control_pixel_rstn)begin
			// 日期/居中布局保持初值，bit0清零防止默认车道被当成新鲜RK识别。
			// 上电扩展HUD可见；车道由独立来源选择，缺少来源时显示未知。
			// PCIe 的 HUD_EXT_REQUEST/TIME/COMMIT 仍可在后续帧原子覆盖这些值。
			ext_state_o <= 32'h00000a40;
			time0_o <= 32'h10102026;
			time1_o <= 32'h00235716;
			ext_valid_o <= 1'b1;
			behavior_result_o <= 32'd0;
			behavior_effective_o <= 32'd0;
			reg_ext_pixel_seq <= 16'd0;
		end else if(ext_command_valid)begin
			behavior_result_o <= ext_command[31:0];
			behavior_effective_o <= ext_command[63:32];
			ext_state_o <= ext_command[95:64];
			time0_o <= ext_command[127:96];
			time1_o <= ext_command[159:128];
			ext_valid_o <= 1'b1;
			reg_ext_pixel_seq <= ext_command[175:160];
		end
	end
	// Track the same pre-edge extension sampled by the overlay at i_latch.
	always@(posedge i_pixel_clk or negedge control_pixel_rstn)begin
		if(!control_pixel_rstn)begin
			reg_ext_latched_seq <= 16'd0;
		end else if(i_latch)begin
			reg_ext_latched_seq <= reg_ext_pixel_seq;
		end
	end
	hud_control_cdc #(.C_SNAPSHOT_WIDTH(124)) hud_control_cdc_Inst_cdc(
		.i_pcie_clk(i_pcie_clk),
		.i_pcie_rstn(control_pcie_rstn),
		.i_pixel_clk(i_pixel_clk),
		.i_pixel_rstn(control_pixel_rstn),
		.i_send(flag_send),
		.i_request(request),
		.o_ready(flag_ready),
		.o_command_valid(flag_command_valid),
		.o_command(command),
		.i_snapshot_valid(i_latched),
		.i_snapshot(pixel_snapshot),
		.o_snapshot_valid(flag_snapshot_valid),
		.o_snapshot(snapshot)
	);
	// 独立导航邮箱；ACK只使用overlay真正锁存的返回口。
	hud_ext_control_cdc #(.C_PAYLOAD_WIDTH(84)) hud_ext_control_cdc_Inst_nav(
		.i_pcie_clk(i_pcie_clk),
		.i_pcie_rstn(control_pcie_rstn),
		.i_pixel_clk(i_pixel_clk),
		.i_pixel_rstn(control_pixel_rstn),
		.i_send(nav_send),
		.i_payload(nav_payload),
		.o_ready(nav_ready),
		.o_command_valid(nav_command_valid),
		.o_command(nav_command)
	);
	always@(posedge i_pixel_clk or negedge control_pixel_rstn)begin
		if(!control_pixel_rstn)begin
			reg_nav_payload <= 84'd0;
			nav_valid_o <= 1'b0;
		end else if(nav_command_valid)begin
			reg_nav_payload <= nav_command;
			nav_valid_o <= 1'b1;
		end
	end
	hud_priority_control hud_priority_control_Inst_priority(
		.i_clk(i_pixel_clk),
		.i_rstn(control_pixel_rstn),
		.i_latch(i_latch),
		.i_command_valid(flag_command_valid),
		.i_command(command),
		.i_dehaze_enable(i_dehaze_enable),
		.i_dehaze_lights(i_dehaze_lights),
		.i_night_level(i_night_level),
		.i_user_left(flag_user_left),
		.i_user_right(flag_user_right),
		.o_state(o_state),
		.o_applied_valid(flag_applied_valid),
		.o_applied_request(applied_request),
		.o_override(flag_override)
	);
endmodule
