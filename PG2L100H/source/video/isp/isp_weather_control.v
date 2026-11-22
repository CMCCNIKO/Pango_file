//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-11-16
// Design Name: 四视频源独立夜间与疑似雾控制
// Module Name: isp_weather_control
// Project Name: PG2L100H_PRJ / v028_isp_dehaze_continuity
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2
// Description:
//   消费同像素时钟域的每区完整直方图摘要，每拍可接收一个区的摘要。
//   夜间与雾分别连续三个新鲜、帧号连续的摘要确认，帧边界原子发布。
//   complete 由上游保证单帧/epoch一致；跨帧epoch变化不破坏确认序列。
//   四个空边界过期；重复/旧帧不续期。有效纯黑与缺源摘要分别处理。
//   夜间0/1/2=日/夜/极暗，3=无效；雾0/1/2/3=无/轻/中/重，先看valid。
//   texture_count为同行相邻样本absDeltaY>=32的强边缘数S；neutral复用WB统计。
//   D1补充有界低照夜间入口，与白天退出互斥，极暗判据保持。
//   雾进入/非零等级切换保持三摘要确认；连续120份无雾才退出，60Hz约2s。
//   缺源/过期/禁用优先清除，保持期间RGB/色调/A估计继续工作。
//   可选逐边界omega/Tmin限速，默认步长0兼容原分级直跳；生产配置2/1。
//   首次请求Tmin直达目标；退出时保留Tmin至omega归零，request随渐退保持。
// Dependencies: 无新增IP、像素缓存、时钟域或除法器。
// Revision 0.01 - File Created
// Revision 0.02 - 增加逐源雾退出时间证据滞回，原C5_D1分类门限保持。
// Revision 0.03 - 增加逐源去雾参数限速，失效仍优先立即撤销。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module isp_weather_control
#(
	parameter C_EXPECTED_SAMPLES = 12800,
	parameter C_EXPECTED_PAIRS = 12720,
	parameter C_FOG_CLEAR_FRAMES = 120,
	parameter C_FOG_OMEGA_STEP = 0,
	parameter C_FOG_TMIN_STEP = 0
)
(
	input i_clk,
	input i_rst_n,
	input i_enable,
	input i_frame_boundary,
	input i_summary_valid,
	input i_summary_complete,
	input [1:0] i_summary_tile,
	input [31:0] i_summary_frame,
	input [15:0] i_summary_epoch,
	input [7:0] i_summary_p10,
	input [7:0] i_summary_p50,
	input [7:0] i_summary_p90,
	input [13:0] i_summary_low48,
	input [13:0] i_summary_fog_count,
	input [13:0] i_summary_texture_count,
	input [13:0] i_summary_neutral_count,
	output [7:0] o_night,
	output [7:0] o_fog,
	output [3:0] o_valid,
	output [31:0] o_omega,
	output [31:0] o_tmin,
	output [1:0] o_night_level,
	output [3:0] o_fog_request
);
	// 参数范围：1<=samples<=16383，1<=pairs<=samples。百分比在展开期计算。
	localparam [13:0] EXPECTED_SAMPLES = C_EXPECTED_SAMPLES;
	localparam [13:0] EXPECTED_PAIRS = C_EXPECTED_PAIRS;
	localparam [13:0] LOW48_DARK_MIN = (C_EXPECTED_SAMPLES * 3 + 3) / 4;
	localparam [13:0] LOW48_DAY_MAX = C_EXPECTED_SAMPLES / 4;
	localparam [13:0] LOW48_RECOVER_MAX = C_EXPECTED_SAMPLES / 2;
	localparam [13:0] NEUTRAL_MIN = (C_EXPECTED_SAMPLES * 40 + 99) / 100;
	localparam [13:0] EDGE_MIN_A = (C_EXPECTED_PAIRS + 999) / 1000;
	localparam [13:0] EDGE_MAX_A = (C_EXPECTED_PAIRS * 10) / 100;
	localparam [13:0] EDGE_MAX_B = (C_EXPECTED_PAIRS * 25) / 100;
	localparam [13:0] FOG_MIN_B = (C_EXPECTED_SAMPLES * 2 + 99) / 100;
	localparam [13:0] FOG_MAX = (C_EXPECTED_SAMPLES * 65) / 100;
	// 1<=clear_frames<=255；120份连续新摘要在当前720p60约2秒。
	localparam [7:0] FOG_CLEAR_FRAMES = C_FOG_CLEAR_FRAMES;
	localparam [7:0] FOG_CLEAR_LAST = C_FOG_CLEAR_FRAMES - 1;
	// 步长范围0..255，0为目标直达；9位差值/加减避免8位回绕。
	localparam [8:0] FOG_OMEGA_STEP = C_FOG_OMEGA_STEP;
	localparam [8:0] FOG_TMIN_STEP = C_FOG_TMIN_STEP;

	// 共享组合分类器；四区状态独立，不重复像素或统计数据通路。
	wire [8:0] summary_span;
	wire flag_well_formed;
	wire flag_very_dark;
	wire flag_dark;
	wire flag_day;
	wire flag_leave_very_dark;
	wire flag_bounded_lowlight;
	wire flag_fog_neutral_branch;
	wire flag_fog_lifted_branch;
	reg [1:0] dec_fog;

	// 打包顺序：tile k 使用[2*k+:2]，字节参数使用[8*k+:8]。
	wire [7:0] night_o;
	wire [7:0] fog_o;
	wire [3:0] valid_o;
	wire [31:0] omega_o;
	wire [31:0] tmin_o;
	wire [3:0] fog_request_o;
	reg [1:0] night_level_o;

	assign summary_span = {1'b0, i_summary_p90} - {1'b0, i_summary_p10};
	assign flag_well_formed = (i_summary_p10 <= i_summary_p50) &&
		(i_summary_p50 <= i_summary_p90) &&
		(i_summary_low48 <= EXPECTED_SAMPLES) &&
		(i_summary_fog_count <= EXPECTED_SAMPLES) &&
		(i_summary_texture_count <= EXPECTED_PAIRS) &&
		(i_summary_neutral_count <= EXPECTED_SAMPLES);
	assign flag_very_dark = (i_summary_p50 <= 8'd36) &&
		(i_summary_low48 >= LOW48_DARK_MIN);
	assign flag_bounded_lowlight = (i_summary_p50 <= 8'd108) &&
		(i_summary_p90 <= 8'd140) && (summary_span >= 9'd24) &&
		(i_summary_p10 <= 8'd100);
	assign flag_dark = (i_summary_p50 <= 8'd76) || flag_bounded_lowlight;
	assign flag_day = (i_summary_p50 >= 8'd108) &&
		(i_summary_low48 <= LOW48_DAY_MAX) && !flag_bounded_lowlight;
	assign flag_leave_very_dark = (i_summary_p50 >= 8'd60) ||
		(i_summary_low48 <= LOW48_RECOVER_MAX);
	assign flag_fog_neutral_branch = (i_summary_neutral_count >= NEUTRAL_MIN) &&
		(i_summary_texture_count >= EDGE_MIN_A) && (i_summary_texture_count <= EDGE_MAX_A);
	assign flag_fog_lifted_branch = (i_summary_p10 >= 8'd100) &&
		(summary_span <= 9'd128) && (i_summary_fog_count >= FOG_MIN_B) &&
		(i_summary_texture_count <= EDGE_MAX_B);

	assign o_night = night_o;
	assign o_fog = fog_o;
	assign o_valid = valid_o;
	assign o_omega = omega_o;
	assign o_tmin = tmin_o;
	assign o_night_level = night_level_o;
	assign o_fog_request = fog_request_o;

	// neutral与稀疏强边缘，或抬升暗部与lifted_chroma，任一满足才进入雾候选。
	always@(*)begin
		dec_fog = 2'd0;
		if((i_summary_p50 >= 8'd80) && (i_summary_p90 >= 8'd112) &&
			(summary_span >= 9'd24) && (summary_span <= 9'd192) &&
			(i_summary_fog_count <= FOG_MAX) &&
			(flag_fog_neutral_branch || flag_fog_lifted_branch))begin
			if((i_summary_p10 >= 8'd172) && (summary_span <= 9'd64))begin
				dec_fog = 2'd3;
			end else if((i_summary_p10 >= 8'd132) && (summary_span <= 9'd96))begin
				dec_fog = 2'd2;
			end else begin
				dec_fog = 2'd1;
			end
		end
	end

	// 聚合已发布有效区的最大夜间等级，任意缺源不参加HUD灯光仲裁。
	always@(*)begin
		night_level_o = 2'd0;
		if(valid_o[0] && (night_o[1:0] > night_level_o))begin
			night_level_o = night_o[1:0];
		end
		if(valid_o[1] && (night_o[3:2] > night_level_o))begin
			night_level_o = night_o[3:2];
		end
		if(valid_o[2] && (night_o[5:4] > night_level_o))begin
			night_level_o = night_o[5:4];
		end
		if(valid_o[3] && (night_o[7:6] > night_level_o))begin
			night_level_o = night_o[7:6];
		end
	end

	genvar gen_tile;
	generate
		for(gen_tile = 0; gen_tile < 4; gen_tile = gen_tile + 1)begin:gen_source
			localparam [1:0] TILE = gen_tile;
			reg flag_seen;
			reg flag_valid;
			reg flag_updated;
			reg [31:0] reg_last_frame;
			reg [1:0] cnt_age;
			reg [1:0] reg_night;
			reg [1:0] reg_night_candidate;
			reg [1:0] cnt_night_confirm;
			reg [1:0] reg_fog;
			reg [1:0] reg_fog_candidate;
			reg [1:0] cnt_fog_confirm;
			reg [7:0] cnt_fog_clear;
			wire [31:0] frame_delta;
			wire flag_accept;
			wire flag_expiring;
			wire flag_continuous;
			wire [1:0] dec_base_night;
			wire [1:0] dec_base_fog;
			reg [1:0] dec_night;
			reg [1:0] night_tile_o;
			reg [1:0] fog_tile_o;
			reg valid_tile_o;
			reg [7:0] omega_tile_o;
			reg [7:0] tmin_tile_o;
			reg fog_request_tile_o;
			reg [7:0] target_omega;
			reg [7:0] target_tmin;
			reg [8:0] dec_omega;
			reg [8:0] dec_tmin;
			wire flag_next_request;

			// 32位模帧序列：FFFFFFFF->0连续，半环及以上差值按旧帧拒绝。
			assign frame_delta = i_summary_frame - reg_last_frame;
			assign flag_accept = i_summary_valid && (i_summary_tile == TILE) &&
				(!flag_seen || ((frame_delta != 32'd0) && !frame_delta[31]));
			assign flag_expiring = i_frame_boundary && !flag_updated && (cnt_age == 2'd3);
			assign flag_continuous = flag_seen && (frame_delta == 32'd1) && !flag_expiring;
			assign dec_base_night = flag_expiring ? 2'd0 : reg_night;
			assign dec_base_fog = flag_expiring ? 2'd0 : reg_fog;
			assign night_o[gen_tile * 2 +: 2] = night_tile_o;
			assign fog_o[gen_tile * 2 +: 2] = fog_tile_o;
			assign valid_o[gen_tile] = valid_tile_o;
			assign omega_o[gen_tile * 8 +: 8] = omega_tile_o;
			assign tmin_o[gen_tile * 8 +: 8] = tmin_tile_o;
			assign fog_request_o[gen_tile] = fog_request_tile_o;
			assign flag_next_request = (reg_fog != 2'd0) || (dec_omega != 9'd0);

			// 分类和H120仅给出目标，实际发布参数在本区帧边界单独限速。
			always@(*)begin
				target_omega = 8'd0;
				target_tmin = 8'd0;
				case(reg_fog)
					2'd1:begin target_omega = 8'd112; target_tmin = 8'd96; end
					2'd2:begin target_omega = 8'd140; target_tmin = 8'd80; end
					2'd3:begin target_omega = 8'd176; target_tmin = 8'd64; end
					default:begin end
				endcase
			end

			// 先比较9位距离再加减，越过目标时钳到目标；步长255也不溢出。
			always@(*)begin
				dec_omega = {1'b0, target_omega};
				if(C_FOG_OMEGA_STEP != 0)begin
					if(target_omega > omega_tile_o)begin
						if(({1'b0, target_omega} - {1'b0, omega_tile_o}) > FOG_OMEGA_STEP)begin
							dec_omega = {1'b0, omega_tile_o} + FOG_OMEGA_STEP;
						end
					end else if(target_omega < omega_tile_o)begin
						if(({1'b0, omega_tile_o} - {1'b0, target_omega}) > FOG_OMEGA_STEP)begin
							dec_omega = {1'b0, omega_tile_o} - FOG_OMEGA_STEP;
						end
					end
				end
			end

			// 入雾先初始化分母下限；渐退阶段保留该下限，omega归零同拍清除。
			always@(*)begin
				dec_tmin = {1'b0, target_tmin};
				if(!flag_next_request)begin
					dec_tmin = 9'd0;
				end else if(reg_fog == 2'd0)begin
					dec_tmin = {1'b0, tmin_tile_o};
				end else if(fog_request_tile_o && (C_FOG_TMIN_STEP != 0))begin
					if(target_tmin > tmin_tile_o)begin
						if(({1'b0, target_tmin} - {1'b0, tmin_tile_o}) > FOG_TMIN_STEP)begin
							dec_tmin = {1'b0, tmin_tile_o} + FOG_TMIN_STEP;
						end
					end else if(target_tmin < tmin_tile_o)begin
						if(({1'b0, tmin_tile_o} - {1'b0, target_tmin}) > FOG_TMIN_STEP)begin
							dec_tmin = {1'b0, tmin_tile_o} - FOG_TMIN_STEP;
						end
					end
				end
			end

			// 夜间滞回与雾等级的确认独立，车灯高光P90不否决夜间。
			always@(*)begin
				dec_night = 2'd0;
				case(dec_base_night)
					2'd0:begin
						if(flag_very_dark)begin
							dec_night = 2'd2;
						end else if(flag_dark)begin
							dec_night = 2'd1;
						end
					end
					2'd1:begin
						if(flag_day)begin
							dec_night = 2'd0;
						end else if(flag_very_dark)begin
							dec_night = 2'd2;
						end else begin
							dec_night = 2'd1;
						end
					end
					2'd2:begin
						if(flag_day)begin
							dec_night = 2'd0;
						end else if(flag_leave_very_dark)begin
							dec_night = 2'd1;
						end else begin
							dec_night = 2'd2;
						end
					end
					default:begin
						dec_night = 2'd0;
					end
				endcase
			end

			// 仅累计连续、完整的新无雾摘要；检出任意雾级重新开始退出计时。
			// 与工作态同拍取旧值，主状态比较N-1可使第N份摘要准确退出。
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					cnt_fog_clear <= 8'd0;
				end else if(!i_enable)begin
					cnt_fog_clear <= 8'd0;
				end else if(flag_accept)begin
					if(!i_summary_complete || !flag_well_formed ||
						(dec_base_fog == 2'd0) || (dec_fog != 2'd0))begin
						cnt_fog_clear <= 8'd0;
					end else if(!flag_continuous)begin
						cnt_fog_clear <= 8'd1;
					end else if(cnt_fog_clear < FOG_CLEAR_FRAMES)begin
						cnt_fog_clear <= cnt_fog_clear + 8'd1;
					end
				end else if(flag_expiring)begin
					cnt_fog_clear <= 8'd0;
				end
			end

			// 工作态接收一拍摘要。边界同拍摘要属于下一次发布。
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					flag_seen <= 1'b0;
					flag_valid <= 1'b0;
					flag_updated <= 1'b0;
					reg_last_frame <= 32'd0;
					cnt_age <= 2'd0;
					reg_night <= 2'd0;
					reg_night_candidate <= 2'd0;
					cnt_night_confirm <= 2'd0;
					reg_fog <= 2'd0;
					reg_fog_candidate <= 2'd0;
					cnt_fog_confirm <= 2'd0;
				end else if(!i_enable)begin
					flag_seen <= 1'b0;
					flag_valid <= 1'b0;
					flag_updated <= 1'b0;
					reg_last_frame <= 32'd0;
					cnt_age <= 2'd0;
					reg_night <= 2'd0;
					reg_night_candidate <= 2'd0;
					cnt_night_confirm <= 2'd0;
					reg_fog <= 2'd0;
					reg_fog_candidate <= 2'd0;
					cnt_fog_confirm <= 2'd0;
				end else if(flag_accept)begin
					flag_seen <= 1'b1;
					flag_updated <= 1'b1;
					reg_last_frame <= i_summary_frame;
					cnt_age <= 2'd0;
					if(!i_summary_complete || !flag_well_formed)begin
						flag_valid <= 1'b0;
						reg_night <= 2'd0;
						reg_night_candidate <= 2'd0;
						cnt_night_confirm <= 2'd0;
						reg_fog <= 2'd0;
						reg_fog_candidate <= 2'd0;
						cnt_fog_confirm <= 2'd0;
					end else begin
						flag_valid <= 1'b1;
						reg_night <= dec_base_night;
						reg_night_candidate <= dec_night;
						cnt_night_confirm <= 2'd0;
						if(dec_night != dec_base_night)begin
							if(flag_continuous && (reg_night_candidate == dec_night) &&
								(cnt_night_confirm >= 2'd2))begin
								reg_night <= dec_night;
							end else if(flag_continuous && (reg_night_candidate == dec_night))begin
								cnt_night_confirm <= cnt_night_confirm + 2'd1;
							end else begin
								cnt_night_confirm <= 2'd1;
							end
						end
						reg_fog <= dec_base_fog;
						reg_fog_candidate <= dec_fog;
						cnt_fog_confirm <= 2'd0;
						if(dec_fog != dec_base_fog)begin
							if(dec_fog == 2'd0)begin
								if((C_FOG_CLEAR_FRAMES == 1) ||
									(flag_continuous && (cnt_fog_clear >= FOG_CLEAR_LAST)))begin
									reg_fog <= 2'd0;
								end
							end else if(flag_continuous && (reg_fog_candidate == dec_fog) &&
								(cnt_fog_confirm >= 2'd2))begin
								reg_fog <= dec_fog;
							end else if(flag_continuous && (reg_fog_candidate == dec_fog))begin
								cnt_fog_confirm <= cnt_fog_confirm + 2'd1;
							end else begin
								cnt_fog_confirm <= 2'd1;
							end
						end
					end
				end else if(i_frame_boundary)begin
					flag_updated <= 1'b0;
					if(flag_updated)begin
						cnt_age <= 2'd0;
					end else if(cnt_age != 2'd3)begin
						cnt_age <= cnt_age + 2'd1;
					end else begin
						flag_valid <= 1'b0;
						reg_night <= 2'd0;
						reg_night_candidate <= 2'd0;
						cnt_night_confirm <= 2'd0;
						reg_fog <= 2'd0;
						reg_fog_candidate <= 2'd0;
						cnt_fog_confirm <= 2'd0;
					end
				end
			end

			// 除复位/禁用外仅边界更新输出，发布前一拍之前已接收的摘要。
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					night_tile_o <= 2'd3;
					fog_tile_o <= 2'd3;
					valid_tile_o <= 1'b0;
					omega_tile_o <= 8'd0;
					tmin_tile_o <= 8'd0;
					fog_request_tile_o <= 1'b0;
				end else if(!i_enable)begin
					night_tile_o <= 2'd3;
					fog_tile_o <= 2'd3;
					valid_tile_o <= 1'b0;
					omega_tile_o <= 8'd0;
					tmin_tile_o <= 8'd0;
					fog_request_tile_o <= 1'b0;
				end else if(i_frame_boundary)begin
					if(flag_valid && !flag_expiring)begin
						night_tile_o <= reg_night;
						fog_tile_o <= reg_fog;
						valid_tile_o <= 1'b1;
						fog_request_tile_o <= flag_next_request;
						omega_tile_o <= dec_omega[7:0];
						tmin_tile_o <= dec_tmin[7:0];
					end else begin
						night_tile_o <= 2'd3;
						fog_tile_o <= 2'd3;
						valid_tile_o <= 1'b0;
						omega_tile_o <= 8'd0;
						tmin_tile_o <= 8'd0;
						fog_request_tile_o <= 1'b0;
					end
				end
			end
		end
	endgenerate
endmodule
