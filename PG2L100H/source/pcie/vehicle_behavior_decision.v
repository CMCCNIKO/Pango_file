//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在 PCIe 域融合场景感知、HSST 车道与用户动作，按保持窗口和定时规则生成统一 HUD 状态及 192 位遥测快照。
// Module Name: vehicle_behavior_decision
// File Name: source/pcie/vehicle_behavior_decision.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在 PCIe 域融合场景感知、HSST 车道与用户动作，按保持窗口和定时规则生成统一 HUD 状态及 192 位遥测快照。
// Ethernet 动作与 BAR1 0x6c0..0x6df 原子动作 CSR 共享判定，ETH 持有优先；HSST 车道判定独立于 RK 交通感知新鲜度。
//
// Dependencies: vehicle_behavior_timebase、vehicle_perception_hold；同域场景/动作和 HSST 车道邮箱。
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
// Create Date: 2026-10-18
// Design Name: RK3566 scene and Ethernet user behavior decision
// Module Name: vehicle_behavior_decision
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2
// Description: Committed scene holds, timed actions and coherent HUD snapshot.
//              i_control_rst_n is the PCIe-synchronously released HUD session reset.
// Dependencies: vehicle_behavior_timebase, vehicle_perception_hold.
// Revision: 0.01 - 2026-10-18 Created
//           0.02 - 2026-10-21 Reference milliseconds, holds and brake advisory.
//           0.03 - 2026-10-23 HSST车道优先，车道可用性独立于RK交通感知保鲜。
//           0.04 - 2026-10-28 同拍车道/行为遥测，仅增加只读观察状态。
//           0.05 - 2026-11-01 BAR1原子动作CSR，与Ethernet共享保持和判定；ETH持有优先。
//           0.06 - 2026-11-12 RK场景车道逐侧优先，字段未识别时回退既有光口车道。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module vehicle_behavior_decision
#(
	parameter integer C_REF_CYCLES_PER_MS = 27000,
	parameter [31:0] C_SCENE_HOLD_MS = 32'd501,
	parameter [31:0] C_ACTION_HOLD_MS = 32'd1001
)
(
	input i_clk,
	input i_rst_n,
	input i_ref_clk,
	input i_ref_rst_n,
	input i_control_rst_n,
	input i_link_up,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window_data,
	input i_scene_commit_valid,
	input [111:0] i_scene_commit_payload,
	input [31:0] i_fiber_lane_sequence,
	input [31:0] i_fiber_lane_payload,
	input i_fiber_lane_valid,
	input i_fiber_lane_stale,
	input i_action_valid,
	input [15:0] i_action_sequence,
	input [2:0] i_action_type,
	input i_action_request,
	output [31:0] o_behavior_result,
	output [31:0] o_behavior_effective,
	output [175:0] o_display_snapshot,
	output [191:0] o_telemetry_snapshot
);
	localparam [11:0] C_SCENE_SHADOW = 12'h638;
	localparam [11:0] C_SCENE_COMMIT = 12'h644;
	localparam [11:0] C_SCENE_STATUS = 12'h658;
	localparam [11:0] C_RESULT = 12'h65c;
	localparam [11:0] C_EFFECTIVE = 12'h660;
	localparam [11:0] C_CAPS = 12'h664;
	localparam [11:0] C_ACTION_MAGIC = 12'h6c0;
	localparam [11:0] C_ACTION_VERSION = 12'h6c4;
	localparam [11:0] C_ACTION_REQUEST = 12'h6c8;
	localparam [11:0] C_ACTION_STATUS = 12'h6cc;
	localparam [11:0] C_ACTION_ACCEPTED = 12'h6d0;
	localparam [11:0] C_ACTION_REJECTED = 12'h6d4;
	localparam [11:0] C_ACTION_REJECT_COUNT = 12'h6d8;
	localparam [11:0] C_ACTION_ERROR = 12'h6dc;
	localparam [7:0] C_ACTION_ERR_ACCESS = 8'd1;
	localparam [7:0] C_ACTION_ERR_ADDRESS = 8'd2;
	localparam [7:0] C_ACTION_ERR_SEQUENCE = 8'd3;
	localparam [7:0] C_ACTION_ERR_RESERVED = 8'd4;
	localparam [7:0] C_ACTION_ERR_TYPE = 8'd5;
	localparam [7:0] C_ACTION_ERR_ETH_BUSY = 8'd6;
	localparam [5:0] R_RED_LIGHT = 6'd1;
	localparam [5:0] R_LEFT_SOLID = 6'd2;
	localparam [5:0] R_RIGHT_SOLID = 6'd3;
	localparam [5:0] R_SENSOR_LINK = 6'd6;
	localparam [5:0] R_BRAKE_ADVISORY = 6'd8;
	wire control_rst_n;
	wire time_ready;
	wire [31:0] timestamp_ms;
	wire [31:0] held_state;
	wire [31:0] scene_time0;
	wire [31:0] scene_time1;
	wire [15:0] scene_sequence;
	wire scene_ready;
	wire scene_fresh;
	wire scene_stale;
	reg [4:0] reg_fiber_lane;
	reg [65:0] reg_fiber_snapshot;
	reg [31:0] telemetry_lane_status_comb;
	reg [31:0] telemetry_action_status_comb;
	reg [31:0] display_scene_comb;
	wire [1:0] left_lane;
	wire [1:0] right_lane;
	wire flag_rk_left_valid;
	wire flag_rk_right_valid;
	wire flag_fiber_left_valid;
	wire flag_fiber_right_valid;
	wire flag_left_lane_stale;
	wire flag_right_lane_stale;
	wire write_full;
	wire action_accept;
	wire action_accept_eth;
	wire action_accept_pcie;
	wire flag_action_write_window;
	wire flag_action_eth_busy;
	reg [7:0] action_error_comb;
	reg [31:0] action_status_comb;
	reg [31:0] reg_action_accepted_request;
	reg [31:0] reg_action_rejected_request;
	reg [31:0] cnt_action_reject;
	reg [7:0] reg_action_error;
	reg [31:0] scene_shadow;
	reg scene_last_error;
	reg [7:0] scene_error_code;
	reg reg_action_active;
	reg reg_action_source_eth;
	reg reg_action_age_ready;
	reg reg_action_anchor_pending;
	reg [2:0] reg_action_type;
	reg [15:0] reg_action_sequence;
	reg [31:0] reg_action_last_ms;
	reg [31:0] reg_action_elapsed_ms;
	reg [31:0] cnt_action_accept;
	reg [31:0] behavior_decision_comb;
	reg [31:0] behavior_result_o /* synthesis syn_preserve = 1 */;
	reg [31:0] behavior_effective_o /* synthesis syn_preserve = 1 */;
	reg [175:0] display_snapshot_o /* synthesis syn_preserve = 1 */;
	reg [127:0] rd_window_o;
	reg [31:0] rd_word;
	reg [11:0] rd_address;
	integer rd_lane;

	assign write_full = i_wr_en && i_wr_strb == 4'hf && i_wr_addr[1:0] == 2'b00;
	assign action_accept_eth = i_action_valid && i_action_request &&
		i_action_sequence != 16'd0 && i_action_type >= 3'd1 && i_action_type <= 3'd4;
	assign flag_action_write_window = i_wr_en && i_wr_addr[11:5] == 7'h36;
	// 只使用既有保持寄存器，不增加毫秒减法/比较至CSR接受路径。
	// 到期沿仍按沿前的ETH ACTIVE拒绝；本沿清owner后，下一沿即可接受PCIe。
	assign flag_action_eth_busy = action_accept_eth || (reg_action_active && reg_action_source_eth);
	assign action_accept_pcie = flag_action_write_window && action_error_comb == 8'd0 && control_rst_n;
	assign action_accept = action_accept_eth || action_accept_pcie;
	// HUD already synchronizes this session reset; do not delay accepted startup events.
	assign control_rst_n = i_ref_rst_n && i_rst_n && i_control_rst_n && i_link_up;
	assign scene_stale = !scene_ready || !scene_fresh || !i_link_up;
	assign flag_rk_left_valid = !scene_stale &&
		(held_state[7:6] == 2'd1 || held_state[7:6] == 2'd2);
	assign flag_rk_right_valid = !scene_stale &&
		(held_state[9:8] == 2'd1 || held_state[9:8] == 2'd2);
	assign flag_fiber_left_valid = reg_fiber_lane[4] &&
		(reg_fiber_lane[1:0] == 2'd1 || reg_fiber_lane[1:0] == 2'd2);
	assign flag_fiber_right_valid = reg_fiber_lane[4] &&
		(reg_fiber_lane[3:2] == 2'd1 || reg_fiber_lane[3:2] == 2'd2);
	// RK controls each side independently. A zero/expired RK field releases
	// only that side, allowing the existing fiber lane decision to continue.
	assign left_lane = flag_rk_left_valid ? held_state[7:6] :
		(flag_fiber_left_valid ? reg_fiber_lane[1:0] : 2'd0);
	assign right_lane = flag_rk_right_valid ? held_state[9:8] :
		(flag_fiber_right_valid ? reg_fiber_lane[3:2] : 2'd0);
	assign flag_left_lane_stale = !(flag_rk_left_valid || flag_fiber_left_valid);
	assign flag_right_lane_stale = !(flag_rk_right_valid || flag_fiber_right_valid);
	assign o_behavior_result = behavior_result_o;
	assign o_behavior_effective = behavior_effective_o;
	assign o_display_snapshot = display_snapshot_o;
	assign o_rd_window_data = rd_window_o;
	// 与原输出级使用同一拍的状态和组合判定，禁止拼接滞后一拍的behavior_result_o。
	assign o_telemetry_snapshot = {cnt_action_accept, behavior_decision_comb,
		telemetry_action_status_comb, telemetry_lane_status_comb, reg_fiber_snapshot[63:0]};

	// 一个DWORD即一次完整动作；所有检查完成后才允许改变共用动作状态。
	// 错误优先级固定，错误ETH输入不能占有动作或阻止合法PCIe请求。
	always@(*)begin
		action_error_comb = 8'd0;
		if(!write_full)begin
			action_error_comb = C_ACTION_ERR_ACCESS;
		end else if(i_wr_addr != C_ACTION_REQUEST)begin
			action_error_comb = C_ACTION_ERR_ADDRESS;
		end else if(i_wr_data[31:16] == 16'd0)begin
			action_error_comb = C_ACTION_ERR_SEQUENCE;
		end else if(i_wr_data[15:3] != 13'd0)begin
			action_error_comb = C_ACTION_ERR_RESERVED;
		end else if(i_wr_data[2:0] < 3'd1 || i_wr_data[2:0] > 3'd4)begin
			action_error_comb = C_ACTION_ERR_TYPE;
		end else if(flag_action_eth_busy)begin
			action_error_comb = C_ACTION_ERR_ETH_BUSY;
		end
	end

	// 此状态仅确认decision域已接受；不声称HDMI帧已应用。
	always@(*)begin
		action_status_comb = 32'd0;
		action_status_comb[0] = control_rst_n && !flag_action_eth_busy;
		action_status_comb[1] = reg_action_active;
		action_status_comb[2] = reg_action_active && reg_action_source_eth;
		action_status_comb[3] = reg_action_active && !reg_action_source_eth;
		action_status_comb[4] = time_ready;
		action_status_comb[7:5] = reg_action_type;
		action_status_comb[31:16] = reg_action_sequence;
	end

	// 只记录PCIe动作窗口的写入结果；ETH替换不冒充PCIe命令接受。
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			reg_action_accepted_request <= 32'd0;
			reg_action_rejected_request <= 32'd0;
			cnt_action_reject <= 32'd0;
			reg_action_error <= 8'd0;
		end else if(flag_action_write_window)begin
			reg_action_error <= action_error_comb;
			if(action_accept_pcie)begin
				reg_action_accepted_request <= i_wr_data;
			end else begin
				reg_action_rejected_request <= i_wr_data;
				cnt_action_reject[7:0] <= cnt_action_reject[7:0] + 8'd1;
				cnt_action_reject[15:8] <= cnt_action_reject[15:8] + {7'd0, (&cnt_action_reject[7:0])};
				cnt_action_reject[23:16] <= cnt_action_reject[23:16] + {7'd0, (&cnt_action_reject[15:0])};
				cnt_action_reject[31:24] <= cnt_action_reject[31:24] + {7'd0, (&cnt_action_reject[23:0])};
			end
		end
	end

	// 原始字段与HSST拥有权/有效分类同拍采样，避免新原始包搭配旧分类。
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			reg_fiber_snapshot <= 66'd0;
		end else begin
			reg_fiber_snapshot <= {i_fiber_lane_stale, i_fiber_lane_valid,
				i_fiber_lane_payload, i_fiber_lane_sequence};
		end
	end

	// 来源由实际变道判定拥有权给出；新鲜HSST的未知/零条同样拥有双侧。
	always@(*)begin
		telemetry_lane_status_comb = 32'd0;
		telemetry_lane_status_comb[1:0] = reg_fiber_snapshot[65:64];
		telemetry_lane_status_comb[2] = flag_fiber_left_valid || flag_fiber_right_valid;
		telemetry_lane_status_comb[3] = !scene_stale;
		telemetry_lane_status_comb[5:4] = left_lane;
		telemetry_lane_status_comb[7:6] = right_lane;
		// 保留既有遥测来源编码：1=RK场景，2=普通光口；不新增编码。
		if(flag_fiber_left_valid || flag_fiber_right_valid)begin
			telemetry_lane_status_comb[9:8] = 2'd2;
		end else if(!scene_stale)begin
			telemetry_lane_status_comb[9:8] = 2'd1;
		end
		telemetry_action_status_comb = 32'd0;
		telemetry_action_status_comb[15:0] = reg_action_sequence;
		telemetry_action_status_comb[18:16] = reg_action_type;
		telemetry_action_status_comb[19] = reg_action_active;
		telemetry_action_status_comb[20] = time_ready;
	end

	// 已有sys->PCIe mailbox输出同域采样。仅5bit分类，不延长任何宽配置提交链。
	// 左右车道逐侧分类；无效/未知/明确零条不回用RK旧值。
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			reg_fiber_lane <= 5'd0;
		end else begin
			reg_fiber_lane <= 5'd0;
			if(i_fiber_lane_valid && !i_fiber_lane_stale)begin
				reg_fiber_lane[4] <= 1'b1;
				if(i_fiber_lane_payload[22] &&
					(i_fiber_lane_payload[17:16] == 2'd1 || i_fiber_lane_payload[17:16] == 2'd2) &&
					!(i_fiber_lane_payload[20] && i_fiber_lane_payload[7:0] == 8'd0))begin
					reg_fiber_lane[1:0] <= i_fiber_lane_payload[17:16];
				end
				if(i_fiber_lane_payload[23] &&
					(i_fiber_lane_payload[19:18] == 2'd1 || i_fiber_lane_payload[19:18] == 2'd2) &&
					!(i_fiber_lane_payload[21] && i_fiber_lane_payload[15:8] == 8'd0))begin
					reg_fiber_lane[3:2] <= i_fiber_lane_payload[19:18];
				end
			end
		end
	end

	// 像素域另有独立HSST邮箱，基础快照必须保留RK车道而非二次回退到旧HSST。
	// state bit0按协议表达RK当前保鲜；首次提交前的示例车道不作为识别数据。
	always@(*)begin
		display_scene_comb = held_state;
		display_scene_comb[0] = !scene_stale;
		display_scene_comb[7:6] = left_lane;
		display_scene_comb[9:8] = right_lane;
	end

	// Only the accepted shared HUD commit updates perception; writes are diagnostics.
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			scene_shadow <= 32'd0;
			scene_last_error <= 1'b0;
			scene_error_code <= 8'd0;
		end else begin
			if(write_full && i_wr_addr == C_SCENE_SHADOW)begin
				scene_shadow <= i_wr_data;
			end
			if(write_full && i_wr_addr == C_SCENE_COMMIT && i_wr_data[15:0] == 16'd0)begin
				scene_last_error <= 1'b1;
				scene_error_code <= 8'd1;
			end else if(i_scene_commit_valid)begin
				scene_last_error <= 1'b0;
				scene_error_code <= 8'd0;
			end
		end
	end

	// 两源共用唯一当前动作、计时和算法；ETH持有期间不缓存/恢复旧PCIe请求。
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			reg_action_active <= 1'b0;
			reg_action_source_eth <= 1'b0;
			reg_action_age_ready <= 1'b0;
			reg_action_anchor_pending <= 1'b0;
			reg_action_type <= 3'd0;
			reg_action_sequence <= 16'd0;
			reg_action_last_ms <= 32'd0;
			reg_action_elapsed_ms <= 32'd0;
		end else if(action_accept)begin
			reg_action_active <= 1'b1;
			reg_action_source_eth <= action_accept_eth;
			reg_action_age_ready <= 1'b0;
			reg_action_anchor_pending <= !time_ready;
			reg_action_type <= action_accept_eth ? i_action_type : i_wr_data[2:0];
			reg_action_sequence <= action_accept_eth ? i_action_sequence : i_wr_data[31:16];
			reg_action_last_ms <= timestamp_ms;
		end else if(time_ready)begin
			// Startup events wait for a real timestamp; established windows never reanchor.
			if(reg_action_anchor_pending)begin
				reg_action_last_ms <= timestamp_ms;
				reg_action_anchor_pending <= 1'b0;
				reg_action_age_ready <= 1'b0;
			end else begin
				reg_action_elapsed_ms <= timestamp_ms - reg_action_last_ms;
				reg_action_age_ready <= 1'b1;
				if(reg_action_age_ready && reg_action_elapsed_ms >= C_ACTION_HOLD_MS)begin
					reg_action_active <= 1'b0;
					reg_action_source_eth <= 1'b0;
				end
			end
		end
	end

	// 同序号重触发也计数；分段并行进位，不在新增路径串接32bit加法器。
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			cnt_action_accept <= 32'd0;
		end else if(action_accept)begin
			cnt_action_accept[7:0] <= cnt_action_accept[7:0] + 8'd1;
			cnt_action_accept[15:8] <= cnt_action_accept[15:8] + {7'd0, (&cnt_action_accept[7:0])};
			cnt_action_accept[23:16] <= cnt_action_accept[23:16] + {7'd0, (&cnt_action_accept[15:0])};
			cnt_action_accept[31:24] <= cnt_action_accept[31:24] + {7'd0, (&cnt_action_accept[23:0])};
		end
	end

	// The held action follows live held perception, not a frozen initial decision.
	always@(*)begin
		behavior_decision_comb = 32'd0;
		behavior_decision_comb[23:16] = reg_action_sequence[7:0];
		behavior_decision_comb[31:24] = scene_sequence[7:0];
		if(reg_action_active)begin
			behavior_decision_comb[14] = 1'b1;
			if(reg_action_type == 3'd2)begin
				behavior_decision_comb[1] = 1'b1;
				if(!scene_stale && held_state[2:1] == 2'd2 && held_state[5:3] == 3'd0)begin
					behavior_decision_comb[13:8] = R_BRAKE_ADVISORY;
				end
			end else if(reg_action_type == 3'd3)begin
				if(left_lane == 2'd2)begin
					behavior_decision_comb[2] = 1'b1;
				end else begin
					behavior_decision_comb[6] = 1'b1;
					behavior_decision_comb[13:8] = left_lane == 2'd1 ? R_LEFT_SOLID : R_SENSOR_LINK;
					behavior_decision_comb[15] = flag_left_lane_stale;
				end
			end else if(reg_action_type == 3'd4)begin
				if(right_lane == 2'd2)begin
					behavior_decision_comb[3] = 1'b1;
				end else begin
					behavior_decision_comb[7] = 1'b1;
					behavior_decision_comb[13:8] = right_lane == 2'd1 ? R_RIGHT_SOLID : R_SENSOR_LINK;
					behavior_decision_comb[15] = flag_right_lane_stale;
				end
			end else if(scene_stale)begin
				behavior_decision_comb[4] = reg_action_type == 3'd1;
				behavior_decision_comb[6] = reg_action_type == 3'd3;
				behavior_decision_comb[7] = reg_action_type == 3'd4;
				behavior_decision_comb[13:8] = R_SENSOR_LINK;
				behavior_decision_comb[15] = 1'b1;
			end else if(reg_action_type == 3'd1)begin
				if(held_state[2:1] == 2'd2)begin
					behavior_decision_comb[0] = 1'b1;
				end else begin
					behavior_decision_comb[4] = 1'b1;
					behavior_decision_comb[13:8] = held_state[2:1] == 2'd1 ? R_RED_LIGHT : R_SENSOR_LINK;
				end
			end
		end
	end
	always@(posedge i_clk or negedge control_rst_n)begin
		if(!control_rst_n)begin
			behavior_result_o <= 32'd0;
			behavior_effective_o <= 32'd0;
			display_snapshot_o <= {16'd0, 32'h00235716, 32'h10102026, 32'h00000a40, 64'd0};
		end else begin
			behavior_result_o <= behavior_decision_comb;
			behavior_effective_o <= behavior_decision_comb;
			display_snapshot_o <= {scene_sequence, scene_time1, scene_time0, display_scene_comb,
				behavior_decision_comb, behavior_decision_comb};
		end
	end

	always@(*)begin
		rd_window_o = 128'd0;
		rd_word = 32'd0;
		rd_address = 12'd0;
		for(rd_lane = 0; rd_lane < 4; rd_lane = rd_lane + 1)begin
			rd_address = {i_rd_addr[11:4], rd_lane[1:0], 2'b00};
			case(rd_address)
				C_SCENE_SHADOW:begin rd_word = scene_shadow; end
				C_SCENE_COMMIT:begin rd_word = 32'd0; end
				C_SCENE_STATUS:begin
					rd_word = 32'd0;
					rd_word[31:16] = scene_sequence;
					rd_word[15:8] = scene_error_code;
					rd_word[5] = !i_link_up;
					rd_word[6] = reg_fiber_lane[4];
					rd_word[4] = scene_last_error;
					rd_word[3] = scene_stale;
					rd_word[2] = scene_ready;
					rd_word[1] = scene_ready;
				end
				C_RESULT:begin rd_word = behavior_result_o; end
				C_EFFECTIVE:begin rd_word = behavior_effective_o; end
				C_CAPS:begin rd_word = 32'h0002_003f; end
				C_ACTION_MAGIC:begin rd_word = 32'h3154_4341; end
				C_ACTION_VERSION:begin rd_word = 32'h0001_0000; end
				C_ACTION_REQUEST:begin rd_word = 32'd0; end
				C_ACTION_STATUS:begin rd_word = action_status_comb; end
				C_ACTION_ACCEPTED:begin rd_word = reg_action_accepted_request; end
				C_ACTION_REJECTED:begin rd_word = reg_action_rejected_request; end
				C_ACTION_REJECT_COUNT:begin rd_word = cnt_action_reject; end
				C_ACTION_ERROR:begin rd_word = {24'd0, reg_action_error}; end
				default:begin rd_word = 32'd0; end
			endcase
			rd_window_o[rd_lane * 32 +: 32] = rd_word;
		end
	end

	vehicle_behavior_timebase
	#(
		.C_REF_CYCLES_PER_MS(C_REF_CYCLES_PER_MS)
	) u_vehicle_behavior_timebase
	(
		.i_ref_clk(i_ref_clk),
		.i_ref_rst_n(i_ref_rst_n),
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_control_rst_n(i_control_rst_n),
		.i_link_up(i_link_up),
		.o_dst_rst_n(),
		.o_ready(time_ready),
		.o_timestamp_ms(timestamp_ms)
	);
	vehicle_perception_hold
	#(
		.C_HOLD_MS(C_SCENE_HOLD_MS)
	) u_vehicle_perception_hold
	(
		.i_clk(i_clk),
		.i_rst_n(control_rst_n),
		.i_time_ready(time_ready),
		.i_timestamp_ms(timestamp_ms),
		.i_scene_commit_valid(i_scene_commit_valid),
		.i_scene_commit_payload(i_scene_commit_payload),
		.o_held_state(held_state),
		.o_time0(scene_time0),
		.o_time1(scene_time1),
		.o_sequence(scene_sequence),
		.o_scene_ready(scene_ready),
		.o_scene_fresh(scene_fresh)
	);
endmodule
