//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在最终 HDMI 顶部 1280×40 区域叠加 HUD 字模、道路与车辆行为图标，包含油门/刹车意图、导航与状态显示。
// Module Name: hdmi_hud_overlay
// File Name: source/video/hdmi_hud_overlay.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在最终 HDMI 顶部 1280×40 区域叠加 HUD 字模、道路与车辆行为图标，包含油门/刹车意图、导航与状态显示。
// VS 下降后一个消隐时钟原子锁存控制快照，字模 ROM 与颜色覆盖合计固定两拍，输出真实锁存观察信号。
//
// Dependencies: 官方 hud_mask_rom_v2；assets/hud/hud_mask_rom_v2.dat；所有控制输入与 i_clk 同域。
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
// Design Name: HDMI Top HUD
// Module Name: hdmi_hud_overlay
// Project Name: Ethernet_ctl_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 (original reference: PDS 2022.2-SP6.4)
// Description: 1280x40 HUD, state latch one blank clock after VS fall, two-clock latency.
// Dependencies: official hud_mask_rom_v2, assets/hud/hud_mask_rom_v2.dat
// Revision: 0.02 - 2026-10-12 增加真实状态锁存观察口，像素流水保持两拍。
// Revision: 0.03 - 2026-10-19 删除旧 FPS 回退，保留 PCIe 控制与离线默认布局。
// Revision: 0.04 - 2026-10-21 动作有效位门控颜色，reason=8刹车警告使用黄色叹号。
// Revision: 0.05 - 2026-10-24 首拍预合成字模两色，缩短ROM后覆盖路径，保持两拍。
// Revision: 0.06 - 2026-11-01 增加油门/刹车意图图标，复用行为帧快照与原两拍流水。
// Additional Comments: Controls must be synchronous to i_clk. No internal blinking.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module hdmi_hud_overlay
(
	input i_clk,
	input i_rst_n,
	input i_vs,
	input i_hs,
	input i_de,
	input [23:0] i_rgb,
	input i_enable,
	input i_left,
	input i_right,
	input i_hazard,
	input i_low_beam,
	input i_high_beam,
	input i_front_fog,
	input i_position,
	input i_adas_enable,
	input [31:0] i_behavior_result,
	input [31:0] i_behavior_effective,
	input [31:0] i_ext_state,
	input [31:0] i_time0,
	input [31:0] i_time1,
	input i_ext_valid,
	input [31:0] i_nav_state,
	input [19:0] i_nav_bcd,
	input [15:0] i_nav_seq,
	input [15:0] i_nav_token,
	input i_nav_valid,
	output o_vs,
	output o_hs,
	output o_de,
	output [23:0] o_rgb,
	output o_control_latch,
	output o_status_latched,
	output [8:0] o_status,
	output [31:0] o_nav_latched_state,
	output [15:0] o_nav_latched_seq,
	output [15:0] o_nav_latched_token,
	output o_nav_latched_valid
);
	reg [31:0] reg_nav_state;
	reg [19:0] reg_nav_bcd;
	reg [15:0] reg_nav_seq;
	reg [15:0] reg_nav_token;
	reg reg_nav_valid;
	reg dec_nav_valid;
	reg [3:0] dec_nav_digit;
	reg [4:0] dec_nav_font;
	reg dec_nav_digit_visible;
	wire [5:0] nav_x;
	wire [5:0] nav_y;
	wire [6:0] nav_text_x;
	wire [2:0] nav_text_col;
	wire [2:0] nav_text_row;
	wire [4:0] nav_m_x;
	reg reg_vs_previous;
	reg reg_status_latch_pending;
	reg [8:0] reg_status;
	reg status_latched_o;
	reg [10:0] cnt_x;
	reg [9:0] cnt_y;
	reg [9:0] dec_rom_addr;
	reg [4:0] dec_mask_bit;
	reg dec_mask_valid;
	reg [23:0] dec_solid;
	reg [23:0] dec_ink;
	reg [4:0] dec_arrow_edge;
	wire [31:0] rom_data;
	reg [26:0] reg_video;
	reg [31:0] reg_mask_onehot;
	reg [23:0] reg_mask_rgb;
	reg [26:0] video_o;
	reg [31:0] reg_ext_state;
	reg [31:0] reg_time0;
	reg [31:0] reg_time1;
	reg reg_ext_valid;
	reg [31:0] reg_behavior_result /* synthesis syn_preserve = 1 */;
	reg [31:0] reg_behavior_effective /* synthesis syn_preserve = 1 */;
	reg reg_behavior_valid /* synthesis syn_preserve = 1 */;
	reg dec_extra_valid;
	reg [23:0] dec_extra_rgb;
	wire [4:0] pedal_y;
	wire [4:0] throttle_x;
	wire [4:0] brake_x;
	wire flag_throttle_active;
	wire flag_brake_active;
	wire flag_throttle_glyph;
	wire flag_brake_glyph;
	integer time_char_idx;
	integer time_col;
	integer time_row;
	reg [3:0] time_nibble;
	reg [4:0] time_font;

	function [4:0] digit_font;
		input [3:0] digit;
		input [2:0] row;
		begin
			case(digit)
				4'd0: case(row) 0:digit_font=5'b11111;1:digit_font=5'b10001;2:digit_font=5'b10011;3:digit_font=5'b10101;4:digit_font=5'b11001;5:digit_font=5'b10001;default:digit_font=5'b11111; endcase
				4'd1: case(row) 0:digit_font=5'b00100;1:digit_font=5'b01100;2:digit_font=5'b00100;3:digit_font=5'b00100;4:digit_font=5'b00100;5:digit_font=5'b00100;default:digit_font=5'b01110; endcase
				4'd2: case(row) 0:digit_font=5'b11111;1:digit_font=5'b00001;2:digit_font=5'b00001;3:digit_font=5'b11111;4:digit_font=5'b10000;5:digit_font=5'b10000;default:digit_font=5'b11111; endcase
				4'd3: case(row) 0:digit_font=5'b11111;1:digit_font=5'b00001;2:digit_font=5'b00001;3:digit_font=5'b01111;4:digit_font=5'b00001;5:digit_font=5'b00001;default:digit_font=5'b11111; endcase
				4'd4: case(row) 0:digit_font=5'b10001;1:digit_font=5'b10001;2:digit_font=5'b10001;3:digit_font=5'b11111;4:digit_font=5'b00001;5:digit_font=5'b00001;default:digit_font=5'b00001; endcase
				4'd5: case(row) 0:digit_font=5'b11111;1:digit_font=5'b10000;2:digit_font=5'b10000;3:digit_font=5'b11111;4:digit_font=5'b00001;5:digit_font=5'b00001;default:digit_font=5'b11111; endcase
				4'd6: case(row) 0:digit_font=5'b01111;1:digit_font=5'b10000;2:digit_font=5'b10000;3:digit_font=5'b11111;4:digit_font=5'b10001;5:digit_font=5'b10001;default:digit_font=5'b11111; endcase
				4'd7: case(row) 0:digit_font=5'b11111;1:digit_font=5'b00001;2:digit_font=5'b00010;3:digit_font=5'b00100;4:digit_font=5'b01000;5:digit_font=5'b01000;default:digit_font=5'b01000; endcase
				4'd8: case(row) 0:digit_font=5'b11111;1:digit_font=5'b10001;2:digit_font=5'b10001;3:digit_font=5'b11111;4:digit_font=5'b10001;5:digit_font=5'b10001;default:digit_font=5'b11111; endcase
				default: case(row) 0:digit_font=5'b11111;1:digit_font=5'b10001;2:digit_font=5'b10001;3:digit_font=5'b11111;4:digit_font=5'b00001;5:digit_font=5'b00001;default:digit_font=5'b11110; endcase
			endcase
		end
	endfunction

	assign o_vs = video_o[26];
	assign o_hs = video_o[25];
	assign o_de = video_o[24];
	assign o_rgb = video_o[23:0];
	assign o_control_latch = reg_status_latch_pending;
	assign o_status_latched = status_latched_o;
	assign o_status = reg_status;
	assign o_nav_latched_state = reg_nav_state;
	assign o_nav_latched_seq = reg_nav_seq;
	assign o_nav_latched_token = reg_nav_token;
	assign o_nav_latched_valid = reg_nav_valid;
	assign nav_x = cnt_x - 11'd928;
	assign nav_y = cnt_y - 10'd8;
	assign nav_text_x = cnt_x - 11'd976;
	assign nav_text_col = nav_text_x[3:1];
	assign nav_text_row = (cnt_y - 10'd12) >> 1;
	assign nav_m_x = (cnt_x - 11'd1058) >> 1;
	assign pedal_y = cnt_y - 10'd9;
	assign throttle_x = cnt_x - 11'd752;
	assign brake_x = cnt_x - 11'd788;
	// 图标表示当前用户意图；允许/禁止仍由左侧原判断格解释。
	assign flag_throttle_active = reg_behavior_valid && (reg_behavior_result[0] || reg_behavior_result[4]);
	assign flag_brake_active = reg_behavior_valid && (reg_behavior_result[1] || reg_behavior_result[5]);
	// 油门踏板及向上加速箭头，只用小范围坐标比较，不新增字模ROM。
	assign flag_throttle_glyph =
		(throttle_x >= 5'd3 && throttle_x <= 5'd10 && (pedal_y == 5'd7 || pedal_y == 5'd21)) ||
		((throttle_x == 5'd2 || throttle_x == 5'd11) && pedal_y >= 5'd8 && pedal_y <= 5'd20) ||
		(throttle_x >= 5'd5 && throttle_x <= 5'd8 && (pedal_y == 5'd10 || pedal_y == 5'd13 || pedal_y == 5'd16 || pedal_y == 5'd19)) ||
		(throttle_x >= 5'd17 && throttle_x <= 5'd18 && pedal_y >= 5'd3 && pedal_y <= 5'd19) ||
		(pedal_y == 5'd4 && throttle_x >= 5'd16 && throttle_x <= 5'd19) ||
		(pedal_y == 5'd5 && throttle_x >= 5'd15 && throttle_x <= 5'd20) ||
		(pedal_y == 5'd6 && throttle_x >= 5'd14 && throttle_x <= 5'd21) ||
		(pedal_y == 5'd7 && throttle_x >= 5'd13 && throttle_x <= 5'd22);
	// 刹车圆环与叹号；空白像素继续使用HUD底色。
	assign flag_brake_glyph =
		(brake_x >= 5'd8 && brake_x <= 5'd15 && (pedal_y == 5'd3 || pedal_y == 5'd4 || pedal_y == 5'd19 || pedal_y == 5'd20)) ||
		(((brake_x >= 5'd5 && brake_x <= 5'd7) || (brake_x >= 5'd16 && brake_x <= 5'd18)) && (pedal_y == 5'd5 || pedal_y == 5'd6 || pedal_y == 5'd17 || pedal_y == 5'd18)) ||
		(((brake_x >= 5'd4 && brake_x <= 5'd5) || (brake_x >= 5'd18 && brake_x <= 5'd19)) && (pedal_y == 5'd7 || pedal_y == 5'd8 || pedal_y == 5'd15 || pedal_y == 5'd16)) ||
		(((brake_x >= 5'd3 && brake_x <= 5'd4) || (brake_x >= 5'd19 && brake_x <= 5'd20)) && pedal_y >= 5'd9 && pedal_y <= 5'd14) ||
		(brake_x >= 5'd11 && brake_x <= 5'd12 && ((pedal_y >= 5'd7 && pedal_y <= 5'd12) || pedal_y == 5'd15 || pedal_y == 5'd16));

	// VS下降后一拍消隐锁控制，接收与VS边界同沿提交、晚一拍传播的配置。
	// 坐标仍在原VS下降沿复位，像素流水不变；像素域之外控制必须先做CDC。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_vs_previous <= 1'b0;
			reg_status_latch_pending <= 1'b0;
			reg_status <= 9'd0;
			status_latched_o <= 1'b0;
			// 与 HTML 初始渲染帧一致。控制域尚未释放时仍显示静态布局，
			// 后续 ext_valid=1 的 PCIe 提交会在 VS 边界覆盖这些默认值。
			reg_ext_state <= 32'h00000a41;
			reg_time0 <= 32'h10102026;
			reg_time1 <= 32'h00235716;
			reg_ext_valid <= 1'b1;
			reg_behavior_result <= 32'd0;
			reg_behavior_effective <= 32'd0;
			reg_behavior_valid <= 1'b0;
			reg_nav_state <= 32'd0;
			reg_nav_bcd <= 20'd0;
			reg_nav_seq <= 16'd0;
			reg_nav_token <= 16'd0;
			reg_nav_valid <= 1'b0;
		end else begin
			reg_vs_previous <= i_vs;
			status_latched_o <= reg_status_latch_pending;
			reg_status_latch_pending <= reg_vs_previous && !i_vs;
			if(reg_status_latch_pending)begin
				reg_nav_state <= i_nav_valid ? i_nav_state : 32'd0;
				reg_nav_bcd <= i_nav_bcd;
				reg_nav_seq <= i_nav_seq;
				reg_nav_token <= i_nav_token;
				reg_nav_valid <= i_nav_valid;
				reg_status <= {i_adas_enable, i_position, i_front_fog, i_high_beam,
					i_low_beam, i_hazard, i_right, i_left, i_enable};
				if(i_ext_valid)begin
					reg_ext_state <= i_ext_state;
					reg_time0 <= i_time0;
					reg_time1 <= i_time1;
					reg_ext_valid <= 1'b1;
				end
				reg_behavior_result <= i_behavior_result;
				reg_behavior_effective <= i_behavior_effective;
				reg_behavior_valid <= i_behavior_result[14];
			end
		end
	end
	// 计数只消耗有效像素，每行固定 1280；消隐期间不改变坐标。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_x <= 11'd0;
			cnt_y <= 10'd0;
		end else if(reg_vs_previous && !i_vs)begin
			cnt_x <= 11'd0;
			cnt_y <= 10'd0;
		end else if(i_de)begin
			if(cnt_x == 11'd1279)begin
				cnt_x <= 11'd0;
				if(cnt_y != 10'd1023)begin
					cnt_y <= cnt_y + 10'd1;
				end
			end else begin
				cnt_x <= cnt_x + 11'd1;
			end
		end
	end

	// 固定箭头斜边查表，仅比较器，不引入运行时除法或乘法器。
	always@(*)begin
		dec_arrow_edge = 5'd31;
		case(cnt_y)
			10'd8:begin dec_arrow_edge = 5'd15; end
			10'd9:begin dec_arrow_edge = 5'd14; end
			10'd10:begin dec_arrow_edge = 5'd12; end
			10'd11:begin dec_arrow_edge = 5'd11; end
			10'd12:begin dec_arrow_edge = 5'd10; end
			10'd13:begin dec_arrow_edge = 5'd8; end
			10'd14:begin dec_arrow_edge = 5'd7; end
			10'd15:begin dec_arrow_edge = 5'd5; end
			10'd16:begin dec_arrow_edge = 5'd4; end
			10'd17:begin dec_arrow_edge = 5'd3; end
			10'd18:begin dec_arrow_edge = 5'd1; end
			10'd19:begin dec_arrow_edge = 5'd0; end
			10'd20:begin dec_arrow_edge = 5'd1; end
			10'd21:begin dec_arrow_edge = 5'd3; end
			10'd22:begin dec_arrow_edge = 5'd4; end
			10'd23:begin dec_arrow_edge = 5'd5; end
			10'd24:begin dec_arrow_edge = 5'd7; end
			10'd25:begin dec_arrow_edge = 5'd8; end
			10'd26:begin dec_arrow_edge = 5'd10; end
			10'd27:begin dec_arrow_edge = 5'd11; end
			10'd28:begin dec_arrow_edge = 5'd12; end
			10'd29:begin dec_arrow_edge = 5'd14; end
			10'd30:begin dec_arrow_edge = 5'd15; end
			10'd31:begin dec_arrow_edge = 5'd16; end
			default:begin dec_arrow_edge = 5'd31; end
		endcase
	end
	// 字模地址译码；每拍只需一行 32 位掩码。
	always@(*)begin
		dec_rom_addr = 9'd511;
		dec_mask_bit = 5'd0;
		dec_mask_valid = 1'b0;
		dec_extra_valid = 1'b0;
		dec_extra_rgb = 24'h000000;
		dec_solid = 24'h0a0e14;
		dec_ink = 24'h252a32;
		if(cnt_y == 10'd39 ||
			(cnt_x == 11'd135 && cnt_y >= 10'd8 && cnt_y < 10'd32) ||
			(cnt_x == 11'd270 && cnt_y >= 10'd8 && cnt_y < 10'd32))begin
			dec_solid = 24'h3a424d;
		end
		if(cnt_x >= 11'd15 && cnt_x < 11'd31 && cnt_y >= 10'd8 && cnt_y < 10'd32 && dec_arrow_edge <= 5'd15 && (cnt_x - 11'd15) >= dec_arrow_edge)begin
			dec_solid = reg_status[1] ? 24'h30d158 : 24'h252a32;
		end
		if(cnt_x >= 11'd43 && cnt_x < 11'd59 && cnt_y >= 10'd8 && cnt_y < 10'd32 && dec_arrow_edge <= 5'd15 && (cnt_x - 11'd43) <= (5'd15 - dec_arrow_edge))begin
			dec_solid = reg_status[2] ? 24'h30d158 : 24'h252a32;
		end
		if(cnt_x >= 11'd79 && cnt_x < 11'd95 && cnt_y >= 10'd8 && cnt_y < 10'd32 && dec_arrow_edge <= 5'd15 && (cnt_x - 11'd79) >= dec_arrow_edge)begin
			dec_solid = reg_status[3] ? 24'hff453a : 24'h252a32;
		end
		if(cnt_x >= 11'd103 && cnt_x < 11'd119 && cnt_y >= 10'd8 && cnt_y < 10'd32 && dec_arrow_edge <= 5'd15 && (cnt_x - 11'd103) <= (5'd15 - dec_arrow_edge))begin
			dec_solid = reg_status[3] ? 24'hff453a : 24'h252a32;
		end
		if(cnt_x >= 11'd145 && cnt_x < 11'd169 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			dec_rom_addr = 9'd224 + cnt_y - 10'd9;
			dec_mask_bit = cnt_x - 11'd145;
			dec_mask_valid = 1'b1;
			dec_ink = reg_status[4] ? 24'h30d158 : 24'h252a32;
		end
		if(cnt_x >= 11'd175 && cnt_x < 11'd199 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			dec_rom_addr = 9'd248 + cnt_y - 10'd9;
			dec_mask_bit = cnt_x - 11'd175;
			dec_mask_valid = 1'b1;
			dec_ink = reg_status[5] ? 24'h0a84ff : 24'h252a32;
		end
		if(cnt_x >= 11'd205 && cnt_x < 11'd229 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			dec_rom_addr = 9'd272 + cnt_y - 10'd9;
			dec_mask_bit = cnt_x - 11'd205;
			dec_mask_valid = 1'b1;
			dec_ink = reg_status[6] ? 24'hff9f0a : 24'h252a32;
		end
		if(cnt_x >= 11'd235 && cnt_x < 11'd259 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			dec_rom_addr = 9'd296 + cnt_y - 10'd9;
			dec_mask_bit = cnt_x - 11'd235;
			dec_mask_valid = 1'b1;
			dec_ink = reg_status[7] ? 24'h30d158 : 24'h252a32;
		end
		if(reg_status[8] && cnt_x >= 11'd541 && cnt_x < 11'd559 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd0 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd541;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd559 && cnt_x < 11'd577 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd20 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd559;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd577 && cnt_x < 11'd595 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd40 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd577;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd595 && cnt_x < 11'd613 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd60 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd595;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd613 && cnt_x < 11'd631 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd80 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd613;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd631 && cnt_x < 11'd649 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd100 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd631;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd649 && cnt_x < 11'd667 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd120 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd649;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd667 && cnt_x < 11'd685 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd140 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd667;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd685 && cnt_x < 11'd703 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd160 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd685;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd703 && cnt_x < 11'd721 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd180 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd703;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		if(reg_status[8] && cnt_x >= 11'd721 && cnt_x < 11'd739 && cnt_y >= 10'd10 && cnt_y < 10'd30)begin
			dec_rom_addr = 9'd200 + cnt_y - 10'd10;
			dec_mask_bit = cnt_x - 11'd721;
			dec_mask_valid = 1'b1;
			dec_ink = 24'h00d4ff;
		end
		// 统一使用 PCIe HUD 布局；日期时间关闭时留空，不回退旧 FPS 水印。
		// HTML感知区：交通灯、行人、静止目标和超速图标。
		if(reg_ext_valid && cnt_x >= 11'd285 && cnt_x < 11'd300 && cnt_y >= 10'd12 && cnt_y < 10'd31)begin
			if((cnt_x == 11'd285 || cnt_x == 11'd299 || cnt_y == 10'd12 || cnt_y == 10'd30))begin
				dec_extra_valid = 1'b1; dec_extra_rgb = 24'h3a424d;
			end else if(((cnt_y == 10'd14 || cnt_y == 10'd20) && cnt_x >= 11'd290 && cnt_x <= 11'd294) ||
				(cnt_y >= 10'd15 && cnt_y <= 10'd19 && cnt_x >= 11'd289 && cnt_x <= 11'd295))begin
				dec_extra_valid = 1'b1;
				dec_extra_rgb = (reg_ext_state[2:1] == 2'd1) ? 24'hff453a : 24'h252a32;
			end else if(((cnt_y == 10'd22 || cnt_y == 10'd28) && cnt_x >= 11'd290 && cnt_x <= 11'd294) ||
				(cnt_y >= 10'd23 && cnt_y <= 10'd27 && cnt_x >= 11'd289 && cnt_x <= 11'd295))begin
				dec_extra_valid = 1'b1;
				dec_extra_rgb = (reg_ext_state[2:1] == 2'd2) ? 24'h30d158 : 24'h252a32;
			end
		end
		if(reg_ext_valid && cnt_x >= 11'd315 && cnt_x < 11'd331 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			// HTML: 3px头、7px身体、四肢和双腿均按整数像素近似。
			if(((cnt_y == 10'd13 || cnt_y == 10'd19) && cnt_x >= 11'd322 && cnt_x <= 11'd324) ||
				(cnt_y >= 10'd14 && cnt_y <= 10'd18 && cnt_x >= 11'd321 && cnt_x <= 11'd325) ||
				(cnt_x >= 11'd321 && cnt_x <= 11'd324 && cnt_y >= 10'd20 && cnt_y <= 10'd26) ||
				(cnt_x >= 11'd319 && cnt_x <= 11'd320 && cnt_y >= 10'd21 && cnt_y <= 10'd24) ||
				(cnt_x >= 11'd325 && cnt_x <= 11'd326 && cnt_y >= 10'd21 && cnt_y <= 10'd24) ||
				(cnt_x >= 11'd320 && cnt_x <= 11'd321 && cnt_y >= 10'd27 && cnt_y <= 10'd31) ||
				(cnt_x >= 11'd324 && cnt_x <= 11'd325 && cnt_y >= 10'd27 && cnt_y <= 10'd31))begin
				dec_extra_valid = 1'b1; dec_extra_rgb = reg_ext_state[3] ? 24'hff9f0a : 24'h252a32;
			end
		end
		if(reg_ext_valid && cnt_x >= 11'd345 && cnt_x < 11'd365 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			// HTML 静止目标始终保留圆环和 X，状态只改变颜色。
			if(((cnt_x == 11'd347 || cnt_x == 11'd363) && cnt_y >= 10'd17 && cnt_y <= 10'd25) ||
				((cnt_y == 10'd13 || cnt_y == 10'd29) && cnt_x >= 11'd351 && cnt_x <= 11'd359) ||
				((cnt_x-11'd345) == (cnt_y-10'd9) || (cnt_x-11'd345) + (cnt_y-10'd9) == 10'd19))begin
				dec_extra_valid = 1'b1; dec_extra_rgb = reg_ext_state[4] ? 24'hff453a : 24'h252a32;
			end
		end
		if(reg_ext_valid && cnt_x >= 11'd375 && cnt_x < 11'd395 && cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			// HTML 超速圆环和 SPD 小字；用比较器字模避免新增 ROM/IP。
			if(((cnt_x == 11'd377 || cnt_x == 11'd393) && cnt_y >= 10'd17 && cnt_y <= 10'd25) ||
				((cnt_y == 10'd13 || cnt_y == 10'd29) && cnt_x >= 11'd381 && cnt_x <= 11'd389) ||
				((cnt_x >= 11'd379 && cnt_x <= 11'd381 && (cnt_y == 10'd18 || cnt_y == 10'd20 || cnt_y == 10'd22)) ||
				 (cnt_x == 11'd379 && (cnt_y == 10'd19 || cnt_y == 10'd21)) ||
				 (cnt_x >= 11'd384 && cnt_x <= 11'd386 && (cnt_y == 10'd18 || cnt_y == 10'd20)) ||
				 (cnt_x == 11'd384 && cnt_y >= 10'd18 && cnt_y <= 10'd22) ||
				 (cnt_x >= 11'd389 && cnt_x <= 11'd391 && (cnt_y == 10'd18 || cnt_y == 10'd22)) ||
				 (cnt_x == 11'd391 && cnt_y >= 10'd18 && cnt_y <= 10'd22)))begin
				dec_extra_valid = 1'b1; dec_extra_rgb = reg_ext_state[5] ? 24'hff9f0a : 24'h252a32;
			end
		end
		// 用户行为结果：四个固定动作格（油门、刹车、左变道、右变道）。
		// 允许为绿色方框，禁止为红色X；没有当前动作时保持深灰轮廓。
		// 动作有效位门控实际颜色，避免窗口到期后旧允许/禁止位留下颜色。
		if(cnt_y >= 10'd9 && cnt_y < 10'd33 &&
			cnt_x >= 11'd420 && cnt_x < 11'd540)begin
			if((cnt_x == 11'd420 || cnt_x == 11'd449 || cnt_x == 11'd450 ||
				cnt_x == 11'd479 || cnt_x == 11'd480 || cnt_x == 11'd509 ||
				cnt_x == 11'd510 || cnt_x == 11'd539 || cnt_y == 10'd9 || cnt_y == 10'd32))begin
				dec_extra_valid = 1'b1;
				dec_extra_rgb = 24'h3a424d;
			end
			if(cnt_x >= 11'd421 && cnt_x < 11'd449)begin
				if(reg_behavior_valid && reg_behavior_result[4])begin
					if((cnt_x-11'd421) == (cnt_y-10'd10) ||
						(cnt_x-11'd421) + (cnt_y-10'd10) == 10'd23)begin
						dec_extra_valid = 1'b1; dec_extra_rgb = 24'hff453a;
					end
				end else if(reg_behavior_valid && reg_behavior_result[0])begin
					dec_extra_valid = 1'b1; dec_extra_rgb = 24'h30d158;
				end
			end
			if(cnt_x >= 11'd451 && cnt_x < 11'd479)begin
				if(reg_behavior_valid && reg_behavior_result[1] && reg_behavior_result[13:8] == 6'd8)begin
					dec_extra_valid = 1'b1;
					dec_extra_rgb = 24'hff9f0a;
					if(cnt_x >= 11'd463 && cnt_x <= 11'd466 &&
						((cnt_y >= 10'd14 && cnt_y <= 10'd23) ||
						(cnt_y >= 10'd26 && cnt_y <= 10'd28)))begin
						dec_extra_rgb = 24'h0a0e14;
					end
				end else if(reg_behavior_valid && reg_behavior_result[5])begin
					if((cnt_x-11'd451) == (cnt_y-10'd10) ||
						(cnt_x-11'd451) + (cnt_y-10'd10) == 10'd23)begin
						dec_extra_valid = 1'b1; dec_extra_rgb = 24'hff453a;
					end
				end else if(reg_behavior_valid && reg_behavior_result[1])begin
					dec_extra_valid = 1'b1; dec_extra_rgb = 24'h30d158;
				end
			end
			if(cnt_x >= 11'd481 && cnt_x < 11'd509)begin
				if(reg_behavior_valid && reg_behavior_result[6])begin
					if((cnt_x-11'd481) == (cnt_y-10'd10) ||
						(cnt_x-11'd481) + (cnt_y-10'd10) == 10'd23)begin
						dec_extra_valid = 1'b1; dec_extra_rgb = 24'hff453a;
					end
				end else if(reg_behavior_valid && reg_behavior_result[2])begin
					dec_extra_valid = 1'b1; dec_extra_rgb = 24'h30d158;
				end
			end
			if(cnt_x >= 11'd511 && cnt_x < 11'd539)begin
				if(reg_behavior_valid && reg_behavior_result[7])begin
					if((cnt_x-11'd511) == (cnt_y-10'd10) ||
						(cnt_x-11'd511) + (cnt_y-10'd10) == 10'd23)begin
						dec_extra_valid = 1'b1; dec_extra_rgb = 24'hff453a;
					end
				end else if(reg_behavior_valid && reg_behavior_result[3])begin
					dec_extra_valid = 1'b1; dec_extra_rgb = 24'h30d158;
				end
			end
		end
		// 与运行文字、车道区留出间隔；复用同帧动作，禁止油门同样点亮意图。
		if(cnt_y >= 10'd9 && cnt_y < 10'd33)begin
			if(cnt_x >= 11'd752 && cnt_x < 11'd776 && flag_throttle_glyph)begin
				dec_extra_valid = 1'b1;
				dec_extra_rgb = flag_throttle_active ? 24'h30d158 : 24'h252a32;
			end
			if(cnt_x >= 11'd788 && cnt_x < 11'd812 && flag_brake_glyph)begin
				dec_extra_valid = 1'b1;
				dec_extra_rgb = flag_brake_active ? 24'hff453a : 24'h252a32;
			end
		end
		// HTML车道检测：左/右三态车道线和中心小车。
		if(reg_ext_valid && cnt_x >= 11'd880 && cnt_x < 11'd905 && cnt_y >= 10'd10 && cnt_y < 10'd31)begin
			if(reg_ext_state[10]) dec_extra_rgb = 24'hff453a;
			else dec_extra_rgb = 24'h00d4ff;
			if((reg_ext_state[7:6] != 2'd0) &&
				(cnt_x == (11'd884 - ((cnt_y - 10'd10) >> 3))) &&
				(reg_ext_state[7:6] == 2'd1 || ((cnt_y - 10'd10) % 10'd6 < 10'd3))) dec_extra_valid = 1'b1;
			if((reg_ext_state[9:8] != 2'd0) &&
				(cnt_x == (11'd900 + ((cnt_y - 10'd10) >> 3))) &&
				(reg_ext_state[9:8] == 2'd1 || ((cnt_y - 10'd10) % 10'd6 < 10'd3))) dec_extra_valid = 1'b1;
			if((cnt_x >= 11'd889 && cnt_x <= 11'd895 && cnt_y >= 10'd18 && cnt_y <= 10'd28) &&
				(cnt_x == 11'd889 || cnt_x == 11'd895 || cnt_y == 10'd18 || cnt_y == 10'd28)) dec_extra_valid = 1'b1;
		end
		// BCD日期时间：YYYY-MM-DD HH:MM:SS，采用小型固定数字字模，不新增IP。
		if(reg_ext_valid && reg_ext_state[11] && cnt_x >= 11'd1120 && cnt_x < 11'd1272 && cnt_y >= 10'd12 && cnt_y < 10'd28)begin
			time_char_idx = (cnt_x - 11'd1120) >> 3;
			time_col = (cnt_x - 11'd1120) & 7;
			time_row = (cnt_y - 10'd12) >> 1;
			time_nibble = 4'd0;
			time_font = 5'd0;
			case(time_char_idx)
				0: time_nibble=reg_time0[15:12]; 1: time_nibble=reg_time0[11:8]; 2: time_nibble=reg_time0[7:4]; 3: time_nibble=reg_time0[3:0];
				5: time_nibble=reg_time0[23:20]; 6: time_nibble=reg_time0[19:16];
				8: time_nibble=reg_time0[31:28]; 9: time_nibble=reg_time0[27:24];
				11: time_nibble=reg_time1[7:4]; 12: time_nibble=reg_time1[3:0];
				14: time_nibble=reg_time1[15:12]; 15: time_nibble=reg_time1[11:8];
				17: time_nibble=reg_time1[23:20]; 18: time_nibble=reg_time1[19:16];
				default: time_nibble=4'd15;
			endcase
			if(time_char_idx == 4 || time_char_idx == 7 || time_char_idx == 13 || time_char_idx == 16)begin
				if(time_col >= 3 && time_col <= 4 && time_row >= 2 && time_row <= 5)begin dec_extra_valid=1'b1; dec_extra_rgb=24'h6b7480; end
			end else if(time_char_idx != 10 && time_col >= 1 && time_col <= 5 && time_row <= 6)begin
				time_font = digit_font(time_nibble,time_row[2:0]);
				if(time_font[5-time_col[2:0]])begin dec_extra_valid=1'b1; dec_extra_rgb=24'h6b7480; end
			end
		end
	end

	// 导航并行译码，像素坐标只比较/移位；复用既有数字字模函数。
	always@(*)begin
		dec_nav_valid = 1'b0;
		dec_nav_digit = 4'd0;
		dec_nav_font = 5'd0;
		dec_nav_digit_visible = 1'b0;
		if(reg_nav_valid && reg_nav_state[0])begin
			if(cnt_x >= 11'd928 && cnt_x < 11'd960 && cnt_y >= 10'd8 && cnt_y < 10'd32)begin
				case(reg_nav_state[2:1])
					2'd0:begin
						dec_nav_valid = (nav_y <= 6'd10 && nav_x >= (6'd15 - nav_y) && nav_x <= (6'd16 + nav_y)) ||
							(nav_x >= 6'd13 && nav_x <= 6'd18 && nav_y >= 6'd10);
					end
					2'd1:begin
						dec_nav_valid = (nav_x <= 6'd10 && nav_y >= (6'd11 - nav_x) && nav_y <= (6'd12 + nav_x)) ||
							(nav_x >= 6'd8 && nav_x <= 6'd28 && nav_y >= 6'd9 && nav_y <= 6'd14) ||
							(nav_x >= 6'd23 && nav_x <= 6'd28 && nav_y >= 6'd12);
					end
					2'd2:begin
						dec_nav_valid = (nav_x >= 6'd21 && nav_y >= (nav_x - 6'd20) && nav_y <= (6'd43 - nav_x)) ||
							(nav_x >= 6'd3 && nav_x <= 6'd23 && nav_y >= 6'd9 && nav_y <= 6'd14) ||
							(nav_x >= 6'd3 && nav_x <= 6'd8 && nav_y >= 6'd12);
					end
					default:begin dec_nav_valid = 1'b0; end
				endcase
			end
			if(cnt_x >= 11'd976 && cnt_x < 11'd1056 && cnt_y >= 10'd12 && cnt_y < 10'd26)begin
				case(nav_text_x[6:4])
					3'd0:begin dec_nav_digit = reg_nav_bcd[19:16]; dec_nav_digit_visible = reg_nav_bcd[19:16] != 4'd0; end
					3'd1:begin dec_nav_digit = reg_nav_bcd[15:12]; dec_nav_digit_visible = reg_nav_bcd[19:12] != 8'd0; end
					3'd2:begin dec_nav_digit = reg_nav_bcd[11:8]; dec_nav_digit_visible = reg_nav_bcd[19:8] != 12'd0; end
					3'd3:begin dec_nav_digit = reg_nav_bcd[7:4]; dec_nav_digit_visible = reg_nav_bcd[19:4] != 16'd0; end
					3'd4:begin dec_nav_digit = reg_nav_bcd[3:0]; dec_nav_digit_visible = 1'b1; end
					default:begin end
				endcase
				dec_nav_font = digit_font(dec_nav_digit, nav_text_row);
				if(dec_nav_digit_visible && nav_text_col < 3'd5 && dec_nav_font[4 - nav_text_col])begin
					dec_nav_valid = 1'b1;
				end
			end
			if(cnt_x >= 11'd1058 && cnt_x < 11'd1068 && cnt_y >= 10'd16 && cnt_y < 10'd26)begin
				if(nav_text_row == 3'd2 || nav_m_x == 5'd0 || nav_m_x == 5'd2 || nav_m_x == 5'd4)begin
					dec_nav_valid = 1'b1;
				end
			end
		end
	end

	hud_mask_rom_v2 hud_mask_rom_Inst_masks(
		.addr(dec_rom_addr),
		.clk(i_clk),
		.rst(!i_rst_n),
		.rd_data(rom_data)
	);
	// 第一拍：与官方同步ROM并行预合成字模位0/1的两种颜色。
	// 覆盖、额外图标与掩码有效条件均不依赖ROM，提前处理以缩短第二拍路径。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_video <= 27'd0;
			reg_mask_onehot <= 32'd1;
			reg_mask_rgb <= 24'd0;
		end else begin
			reg_video[26:24] <= {i_vs, i_hs, i_de};
			reg_mask_onehot <= 32'd1 << dec_mask_bit;
			if(i_de && reg_status[0] && cnt_y < 10'd40)begin
				if(dec_extra_valid || dec_nav_valid)begin
					reg_video[23:0] <= dec_nav_valid ? 24'h00d4ff : dec_extra_rgb;
					reg_mask_rgb <= dec_nav_valid ? 24'h00d4ff : dec_extra_rgb;
				end else begin
					reg_video[23:0] <= dec_solid;
					reg_mask_rgb <= dec_mask_valid ? dec_ink : dec_solid;
				end
			end else begin
				reg_video[23:0] <= i_rgb;
				reg_mask_rgb <= i_rgb;
			end
		end
	end
	// 第二拍：覆盖输出。关闭 HUD 与消隐像素保持同样两拍延迟。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			video_o <= 27'd0;
		end else begin
			video_o[26:24] <= reg_video[26:24];
			video_o[23:0] <= (|(rom_data & reg_mask_onehot)) ? reg_mask_rgb : reg_video[23:0];
		end
	end
endmodule
