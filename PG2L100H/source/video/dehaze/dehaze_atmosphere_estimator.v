//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-16（v028 天气连续跟踪与混帧保护）
// Design Name: 对四个 tile 分别收集四个空间候选，在消隐内完成置信度排序、场景跟踪及 IIR 平滑，生成上一帧大气光参数。
// Module Name: dehaze_atmosphere_estimator
// File Name: source/video/dehaze/dehaze_atmosphere_estimator.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对四个 tile 分别收集四个空间候选，在消隐内完成置信度排序、场景跟踪及 IIR 平滑，生成上一帧大气光参数。
// 所有 tile 参数在首个 DE 之前一起发布；消隐不足则延后到下一帧边界，像素参数选择保持组合且不停流。
// 天气模式仅对已标记的连续自动ISP更新保留A；完整帧独立记录采样epoch，帧内变化拒绝候选。
// 有效天气A每个可见帧最多变化2，首次完整预热直接初始化；手动模式保留原IIR与场景跳变。
//
// Dependencies: 无 RAM 或算术 IP；帧边界不得与 i_de 重叠。
//
// Revision:
// 2026-11-16 / v028 - 新增采样epoch64位、天气模式1位及pending发布保护1位；无新增RAM/IP。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Design Name: Four-tile previous-frame atmosphere estimator
// Module Name: dehaze_atmosphere_estimator
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Four spatial candidates per tile; confidence sorting and IIR
//              run in blanking. All tiles publish together before the first DE.
//              Short blanking defers publication to the next frame boundary.
//              Pixel parameter selection remains combinational, with no stalls.
// Dependencies: No RAM or arithmetic IP; boundary must not overlap i_de.
// Revision: 0.01 - Original frame estimator.
//           0.02 - 2026-10-21, spatial confidence and temporal scene tracking.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module dehaze_atmosphere_estimator
#(
	parameter integer C_ISP_ENABLE = 0,
	parameter integer C_WEATHER_ENABLE = 0,
	parameter integer C_TILE_PIXELS = 204800,
	parameter [7:0] C_DARK_DELTA = 8'd16,
	parameter [7:0] C_SCENE_THRESHOLD = 8'd32
)
(
	input i_clk,
	input i_rstn,
	input i_de,
	input i_content_valid,
	input [1:0] i_tile,
	input [9:0] i_local_x,
	input [8:0] i_local_y,
	input [7:0] i_dark,
	input [23:0] i_rgb,
	input [31:0] i_frame_id,
	input i_frame_boundary,
	input [63:0] i_config,
	input [3:0] i_source_alive,
	input i_source_alive_valid,
	input [63:0] i_isp_epoch,
	input [3:0] i_isp_active,
	input i_weather_tracking,
	input i_isp_continuous_update,
	output [7:0] o_active_a,
	output o_a_valid,
	output [31:0] o_stat_frame,
	output [31:0] o_all_a,
	output [3:0] o_all_a_valid,
	output [127:0] o_all_stat_frame
);
	localparam [3:0] ST_IDLE = 4'd0;
	localparam [3:0] ST_LOAD = 4'd1;
	localparam [3:0] ST_MAX = 4'd2;
	localparam [3:0] ST_FILTER = 4'd3;
	localparam [3:0] ST_SORT0 = 4'd4;
	localparam [3:0] ST_SORT1 = 4'd5;
	localparam [3:0] ST_SORT2 = 4'd6;
	localparam [3:0] ST_SELECT = 4'd7;
	localparam [3:0] ST_DIFFERENCE = 4'd8;
	localparam [3:0] ST_SMOOTH = 4'd9;
	localparam [3:0] ST_NEXT = 4'd10;
	localparam [3:0] ST_DONE = 4'd11;
	reg [3:0] state_current;
	reg [3:0] state_next;
	reg [63:0] reg_config;
	reg [3:0] reg_alive;
	reg flag_alive_known;
	reg [63:0] reg_isp_epoch;
	reg reg_weather_tracking;
	reg [3:0] flag_fixed_compatible;
	reg [7:0] reg_candidate_d [0:15];
	reg [7:0] reg_candidate_c [0:15];
	reg [15:0] flag_candidate;
	reg [31:0] reg_sample_frame [0:3];
	reg [15:0] reg_sample_epoch [0:3];
	reg [3:0] flag_frame_mixed;
	reg [17:0] cnt_frame_samples [0:3];
	reg [7:0] reg_frozen_d [0:15];
	reg [7:0] reg_frozen_c [0:15];
	reg [15:0] flag_frozen_candidate;
	reg [31:0] reg_frozen_frame [0:3];
	reg [3:0] flag_job_eligible;
	reg [7:0] reg_active_a [0:3];
	reg [31:0] reg_stat_frame [0:3];
	reg [3:0] flag_a_valid;
	reg [3:0] flag_jump_armed;
	reg [3:0] flag_jump_up;
	reg [7:0] reg_result_a [0:3];
	reg [31:0] reg_result_frame [0:3];
	reg [3:0] flag_result_valid;
	reg [3:0] flag_result_jump;
	reg [3:0] flag_result_up;
	reg flag_pending;
	reg flag_visible_started;
	reg flag_boundary_published;
	reg [1:0] reg_work_tile;
	reg [7:0] reg_work_d [0:3];
	reg [7:0] reg_work_c [0:3];
	reg [3:0] flag_work_valid;
	reg [7:0] reg_max_pair0;
	reg [7:0] reg_max_pair1;
	reg [7:0] reg_dark_threshold;
	reg [2:0] cnt_selected;
	reg [7:0] reg_selected_a;
	reg [7:0] reg_difference;
	reg flag_difference_up;
	wire [63:0] isp_epoch;
	wire [3:0] isp_active;
	wire weather_tracking;
	wire [7:0] max_rg;
	wire [7:0] max_rgb;
	wire [1:0] region_index;
	wire [3:0] candidate_index;
	wire flag_sample;
	wire flag_auto;
	wire flag_selected;
	wire [3:0] invalidate_mask;
	wire [3:0] source_invalidate_mask;
	wire flag_publish_blank;
	wire [7:0] work_maximum;
	wire [3:0] work_keep;
	wire [8:0] pair_sum0;
	wire [8:0] pair_sum1;
	wire [7:0] selected_clamped;
	wire [8:0] rounded_difference;
	wire [7:0] smooth_step;
	wire [7:0] weather_step;
	wire [7:0] frozen_d0;
	wire [7:0] frozen_d1;
	wire [7:0] frozen_d2;
	wire [7:0] frozen_d3;
	integer cnt_sample;
	integer cnt_tile;
	integer cnt_frozen;
	genvar gen_tile;
	genvar gen_region;
	assign isp_epoch = C_ISP_ENABLE != 0 ? i_isp_epoch : 64'd0;
	assign isp_active = C_ISP_ENABLE != 0 ? i_isp_active : 4'd0;
	assign weather_tracking = C_WEATHER_ENABLE != 0 && i_weather_tracking;
	assign max_rg = i_rgb[23:16] >= i_rgb[15:8] ? i_rgb[23:16] : i_rgb[15:8];
	assign max_rgb = max_rg >= i_rgb[7:0] ? max_rg : i_rgb[7:0];
	assign region_index = {i_local_y >= 9'd160, i_local_x >= 10'd320};
	assign candidate_index = {i_tile, region_index};
	assign flag_auto = reg_config[16];
	assign flag_selected = reg_config[56] && reg_config[48 + i_tile];
	assign flag_sample = i_de && i_content_valid && i_source_alive_valid && flag_alive_known &&
		i_source_alive[i_tile] && reg_alive[i_tile] && flag_selected;
	assign o_active_a = flag_auto ? reg_active_a[i_tile] : reg_config[31:24];
	assign o_a_valid = flag_auto ? flag_sample && flag_a_valid[i_tile] :
		i_de && i_content_valid && flag_fixed_compatible[i_tile];
	assign o_stat_frame = flag_auto ? reg_stat_frame[i_tile] : 32'd0;
	// 旧pending已在本边界发布时，新job保留到下边界，防止一个可见帧累计两次天气步进。
	assign flag_publish_blank = state_current == ST_DONE && !flag_visible_started && !i_de && !i_frame_boundary &&
		(!weather_tracking || !flag_boundary_published);
	assign work_maximum = reg_max_pair0 >= reg_max_pair1 ? reg_max_pair0 : reg_max_pair1;
	assign work_keep[0] = flag_work_valid[0] && reg_work_d[0] >= reg_dark_threshold;
	assign work_keep[1] = flag_work_valid[1] && reg_work_d[1] >= reg_dark_threshold;
	assign work_keep[2] = flag_work_valid[2] && reg_work_d[2] >= reg_dark_threshold;
	assign work_keep[3] = flag_work_valid[3] && reg_work_d[3] >= reg_dark_threshold;
	assign pair_sum0 = {1'b0, reg_work_c[0]} + {1'b0, reg_work_c[1]};
	assign pair_sum1 = {1'b0, reg_work_c[1]} + {1'b0, reg_work_c[2]};
	assign selected_clamped = reg_selected_a == 8'd0 ? 8'd1 : (reg_selected_a > 8'd250 ? 8'd250 : reg_selected_a);
	assign rounded_difference = {1'b0, reg_difference} + 9'd2;
	assign smooth_step = {1'b0, rounded_difference[8:2]};
	assign weather_step = reg_difference > 8'd2 ? 8'd2 : reg_difference;
	assign frozen_d0 = flag_frozen_candidate[{reg_work_tile, 2'b00}] ? reg_frozen_d[{reg_work_tile, 2'b00}] : 8'd0;
	assign frozen_d1 = flag_frozen_candidate[{reg_work_tile, 2'b01}] ? reg_frozen_d[{reg_work_tile, 2'b01}] : 8'd0;
	assign frozen_d2 = flag_frozen_candidate[{reg_work_tile, 2'b10}] ? reg_frozen_d[{reg_work_tile, 2'b10}] : 8'd0;
	assign frozen_d3 = flag_frozen_candidate[{reg_work_tile, 2'b11}] ? reg_frozen_d[{reg_work_tile, 2'b11}] : 8'd0;
	generate
		for(gen_tile = 0; gen_tile < 4; gen_tile = gen_tile + 1)begin:gen_parameter_outputs
			assign o_all_a[gen_tile * 8 +: 8] = reg_active_a[gen_tile];
			assign o_all_a_valid[gen_tile] = flag_a_valid[gen_tile];
			assign o_all_stat_frame[gen_tile * 32 +: 32] = reg_stat_frame[gen_tile];
			assign invalidate_mask[gen_tile] = !reg_alive[gen_tile] ||
				((reg_isp_epoch[gen_tile * 16 +: 16] != isp_epoch[gen_tile * 16 +: 16]) &&
				 !(weather_tracking && i_isp_continuous_update)) ||
				(reg_weather_tracking != weather_tracking) ||
				reg_config[16] != i_config[16] || (!reg_config[56] && i_config[56]) ||
				(!reg_config[48 + gen_tile] && i_config[48 + gen_tile]);
			assign source_invalidate_mask[gen_tile] = i_source_alive_valid && !flag_alive_known &&
				(!i_source_alive[gen_tile] || !reg_alive[gen_tile]);
		end
		// Constant-index comparisons avoid a 16-candidate feedback mux per pixel.
		for(gen_region = 0; gen_region < 16; gen_region = gen_region + 1)begin:gen_spatial_candidate
			always@(posedge i_clk or negedge i_rstn)begin
				if(!i_rstn)begin
					flag_candidate[gen_region] <= 1'b0;
					reg_candidate_d[gen_region] <= 8'd0;
					reg_candidate_c[gen_region] <= 8'd0;
				end else if(i_frame_boundary)begin
					flag_candidate[gen_region] <= 1'b0;
				end else if(flag_sample && candidate_index == gen_region)begin
					if(!flag_candidate[gen_region] || i_dark > reg_candidate_d[gen_region] ||
						(i_dark == reg_candidate_d[gen_region] && max_rgb > reg_candidate_c[gen_region]))begin
						flag_candidate[gen_region] <= 1'b1;
						reg_candidate_d[gen_region] <= i_dark;
						reg_candidate_c[gen_region] <= max_rgb;
					end
				end
			end
		end
	endgenerate

	// Configuration is sampled after the dark-channel boundary pipeline drains.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_config <= 64'h000fe619dc010080;
			reg_alive <= 4'd0;
			flag_alive_known <= 1'b0;
			reg_isp_epoch <= 64'd0;
			reg_weather_tracking <= 1'b0;
		end else if(i_frame_boundary)begin
			reg_config <= i_config;
			reg_isp_epoch <= isp_epoch;
			reg_weather_tracking <= weather_tracking;
			flag_alive_known <= 1'b0;
		end else if(!i_source_alive_valid)begin
			flag_alive_known <= 1'b0;
		end else if(!flag_alive_known)begin
			reg_alive <= i_source_alive;
			flag_alive_known <= 1'b1;
		end
	end

	// A mixed or incomplete frame cannot finish warmup.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			flag_frame_mixed <= 4'd0;
			for(cnt_sample = 0; cnt_sample < 4; cnt_sample = cnt_sample + 1)begin
				reg_sample_frame[cnt_sample] <= 32'd0;
				reg_sample_epoch[cnt_sample] <= 16'd0;
				cnt_frame_samples[cnt_sample] <= 18'd0;
			end
		end else if(i_frame_boundary)begin
			flag_frame_mixed <= 4'd0;
			for(cnt_sample = 0; cnt_sample < 4; cnt_sample = cnt_sample + 1)begin
				cnt_frame_samples[cnt_sample] <= 18'd0;
			end
		end else if(flag_sample)begin
			if(cnt_frame_samples[i_tile] == 18'd0)begin
				reg_sample_frame[i_tile] <= i_frame_id;
				reg_sample_epoch[i_tile] <= isp_epoch[i_tile * 16 +: 16];
			end else if(reg_sample_frame[i_tile] != i_frame_id ||
				reg_sample_epoch[i_tile] != isp_epoch[i_tile * 16 +: 16])begin
				flag_frame_mixed[i_tile] <= 1'b1;
			end
			if(cnt_frame_samples[i_tile] < C_TILE_PIXELS)begin
				cnt_frame_samples[i_tile] <= cnt_frame_samples[i_tile] + 18'd1;
			end
		end
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			state_current <= ST_IDLE;
		end else if(i_frame_boundary)begin
			state_current <= ST_LOAD;
		end else begin
			state_current <= state_next;
		end
	end
	always@(*)begin
		state_next = ST_IDLE;
		case(state_current)
			ST_IDLE:begin state_next = ST_IDLE; end
			ST_LOAD:begin state_next = ST_MAX; end
			ST_MAX:begin state_next = ST_FILTER; end
			ST_FILTER:begin state_next = ST_SORT0; end
			ST_SORT0:begin state_next = ST_SORT1; end
			ST_SORT1:begin state_next = ST_SORT2; end
			ST_SORT2:begin state_next = ST_SELECT; end
			ST_SELECT:begin state_next = ST_DIFFERENCE; end
			ST_DIFFERENCE:begin state_next = ST_SMOOTH; end
			ST_SMOOTH:begin state_next = ST_NEXT; end
			ST_NEXT:begin state_next = reg_work_tile == 2'd3 ? ST_DONE : ST_LOAD; end
			ST_DONE:begin state_next = ST_IDLE; end
			default:begin state_next = ST_IDLE; end
		endcase
	end

	// A 41-cycle worker uses four-value compare/swap stages in the blank interval.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			flag_frozen_candidate <= 16'd0;
			flag_job_eligible <= 4'd0;
			flag_result_valid <= 4'd0;
			flag_result_jump <= 4'd0;
			flag_result_up <= 4'd0;
			flag_pending <= 1'b0;
			flag_visible_started <= 1'b0;
			flag_boundary_published <= 1'b0;
			reg_work_tile <= 2'd0;
			flag_work_valid <= 4'd0;
			reg_max_pair0 <= 8'd0;
			reg_max_pair1 <= 8'd0;
			reg_dark_threshold <= 8'd0;
			cnt_selected <= 3'd0;
			reg_selected_a <= 8'd1;
			reg_difference <= 8'd0;
			flag_difference_up <= 1'b0;
			for(cnt_frozen = 0; cnt_frozen < 16; cnt_frozen = cnt_frozen + 1)begin
				reg_frozen_d[cnt_frozen] <= 8'd0;
				reg_frozen_c[cnt_frozen] <= 8'd0;
			end
			for(cnt_frozen = 0; cnt_frozen < 4; cnt_frozen = cnt_frozen + 1)begin
				reg_frozen_frame[cnt_frozen] <= 32'd0;
				reg_result_a[cnt_frozen] <= 8'd220;
				reg_result_frame[cnt_frozen] <= 32'd0;
				reg_work_d[cnt_frozen] <= 8'd0;
				reg_work_c[cnt_frozen] <= 8'd0;
			end
		end else if(i_frame_boundary)begin
			flag_frozen_candidate <= flag_candidate;
			reg_work_tile <= 2'd0;
			flag_visible_started <= 1'b0;
			flag_boundary_published <= weather_tracking && flag_pending &&
				(|(flag_result_valid & ~invalidate_mask));
			flag_pending <= 1'b0;
			for(cnt_frozen = 0; cnt_frozen < 16; cnt_frozen = cnt_frozen + 1)begin
				reg_frozen_d[cnt_frozen] <= reg_candidate_d[cnt_frozen];
				reg_frozen_c[cnt_frozen] <= reg_candidate_c[cnt_frozen];
			end
			for(cnt_frozen = 0; cnt_frozen < 4; cnt_frozen = cnt_frozen + 1)begin
				reg_frozen_frame[cnt_frozen] <= reg_sample_frame[cnt_frozen];
				flag_job_eligible[cnt_frozen] <= !invalidate_mask[cnt_frozen] && !flag_frame_mixed[cnt_frozen] &&
					(|flag_candidate[cnt_frozen * 4 +: 4]) &&
					(C_ISP_ENABLE == 0 || cnt_frame_samples[cnt_frozen] == C_TILE_PIXELS);
				if(!flag_pending)begin
					reg_result_a[cnt_frozen] <= reg_active_a[cnt_frozen];
					reg_result_frame[cnt_frozen] <= reg_stat_frame[cnt_frozen];
				end
				flag_result_valid[cnt_frozen] <= !invalidate_mask[cnt_frozen] &&
					(flag_pending ? flag_result_valid[cnt_frozen] : flag_a_valid[cnt_frozen]);
				flag_result_jump[cnt_frozen] <= 1'b0;
				flag_result_up[cnt_frozen] <= flag_pending ? flag_result_up[cnt_frozen] : flag_jump_up[cnt_frozen];
			end
		end else begin
			if(i_de)begin flag_visible_started <= 1'b1; end
			case(state_current)
				ST_LOAD:begin
					flag_work_valid <= flag_frozen_candidate[{reg_work_tile, 2'b00} +: 4];
					for(cnt_frozen = 0; cnt_frozen < 4; cnt_frozen = cnt_frozen + 1)begin
						reg_work_d[cnt_frozen] <= reg_frozen_d[{reg_work_tile, 2'b00} + cnt_frozen];
						reg_work_c[cnt_frozen] <= reg_frozen_c[{reg_work_tile, 2'b00} + cnt_frozen];
					end
					reg_max_pair0 <= frozen_d0 >= frozen_d1 ? frozen_d0 : frozen_d1;
					reg_max_pair1 <= frozen_d2 >= frozen_d3 ? frozen_d2 : frozen_d3;
				end
				ST_MAX:begin
					reg_dark_threshold <= work_maximum > C_DARK_DELTA ? work_maximum - C_DARK_DELTA : 8'd0;
				end
				ST_FILTER:begin
					cnt_selected <= {2'd0, work_keep[0]} + {2'd0, work_keep[1]} + {2'd0, work_keep[2]} + {2'd0, work_keep[3]};
					for(cnt_frozen = 0; cnt_frozen < 4; cnt_frozen = cnt_frozen + 1)begin
						reg_work_c[cnt_frozen] <= work_keep[cnt_frozen] ? reg_work_c[cnt_frozen] : 8'd255;
					end
				end
				ST_SORT0:begin
					if(reg_work_c[0] > reg_work_c[1])begin
						reg_work_c[0] <= reg_work_c[1]; reg_work_c[1] <= reg_work_c[0];
					end
					if(reg_work_c[2] > reg_work_c[3])begin
						reg_work_c[2] <= reg_work_c[3]; reg_work_c[3] <= reg_work_c[2];
					end
				end
				ST_SORT1:begin
					if(reg_work_c[0] > reg_work_c[2])begin
						reg_work_c[0] <= reg_work_c[2]; reg_work_c[2] <= reg_work_c[0];
					end
					if(reg_work_c[1] > reg_work_c[3])begin
						reg_work_c[1] <= reg_work_c[3]; reg_work_c[3] <= reg_work_c[1];
					end
				end
				ST_SORT2:begin
					if(reg_work_c[1] > reg_work_c[2])begin
						reg_work_c[1] <= reg_work_c[2]; reg_work_c[2] <= reg_work_c[1];
					end
				end
				ST_SELECT:begin
					case(cnt_selected)
						3'd2:begin reg_selected_a <= pair_sum0[8:1]; end
						3'd3:begin reg_selected_a <= reg_work_c[1]; end
						3'd4:begin reg_selected_a <= pair_sum1[8:1]; end
						default:begin reg_selected_a <= reg_work_c[0]; end
					endcase
				end
				ST_DIFFERENCE:begin
					reg_selected_a <= selected_clamped;
					flag_difference_up <= selected_clamped >= reg_active_a[reg_work_tile];
					reg_difference <= selected_clamped >= reg_active_a[reg_work_tile] ?
						selected_clamped - reg_active_a[reg_work_tile] : reg_active_a[reg_work_tile] - selected_clamped;
				end
				ST_SMOOTH:begin
					if(flag_job_eligible[reg_work_tile])begin
						if(!flag_a_valid[reg_work_tile])begin
							reg_result_a[reg_work_tile] <= reg_selected_a;
							flag_result_jump[reg_work_tile] <= 1'b0;
						end else if(weather_tracking)begin
							reg_result_a[reg_work_tile] <= flag_difference_up ?
								reg_active_a[reg_work_tile] + weather_step : reg_active_a[reg_work_tile] - weather_step;
							flag_result_jump[reg_work_tile] <= 1'b0;
						end else if(reg_difference >= C_SCENE_THRESHOLD && flag_jump_armed[reg_work_tile] &&
							flag_jump_up[reg_work_tile] == flag_difference_up)begin
							reg_result_a[reg_work_tile] <= reg_selected_a;
							flag_result_jump[reg_work_tile] <= 1'b0;
						end else begin
							reg_result_a[reg_work_tile] <= flag_difference_up ?
								reg_active_a[reg_work_tile] + smooth_step : reg_active_a[reg_work_tile] - smooth_step;
							flag_result_jump[reg_work_tile] <= reg_difference >= C_SCENE_THRESHOLD;
						end
						flag_result_up[reg_work_tile] <= flag_difference_up;
						reg_result_frame[reg_work_tile] <= reg_frozen_frame[reg_work_tile];
						flag_result_valid[reg_work_tile] <= 1'b1;
					end
				end
				ST_NEXT:begin reg_work_tile <= reg_work_tile + 2'd1; end
				ST_DONE:begin flag_pending <= !flag_publish_blank; end
				default:begin end
			endcase
			for(cnt_frozen = 0; cnt_frozen < 4; cnt_frozen = cnt_frozen + 1)begin
				if(source_invalidate_mask[cnt_frozen])begin
					flag_result_valid[cnt_frozen] <= 1'b0;
					flag_result_jump[cnt_frozen] <= 1'b0;
					flag_job_eligible[cnt_frozen] <= 1'b0;
				end
			end
		end
	end

	// Configuration invalidation takes priority over any pending old-frame result.
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			flag_a_valid <= 4'd0;
			flag_fixed_compatible <= 4'hf;
			flag_jump_armed <= 4'd0;
			flag_jump_up <= 4'd0;
			for(cnt_tile = 0; cnt_tile < 4; cnt_tile = cnt_tile + 1)begin
				reg_active_a[cnt_tile] <= 8'd220;
				reg_stat_frame[cnt_tile] <= 32'd0;
			end
		end else begin
			for(cnt_tile = 0; cnt_tile < 4; cnt_tile = cnt_tile + 1)begin
				if(i_frame_boundary)begin
					if(!isp_active[cnt_tile] || reg_config[31:24] != i_config[31:24] || (reg_config[16] && !i_config[16]))begin
						flag_fixed_compatible[cnt_tile] <= 1'b1;
					end else if(reg_isp_epoch[cnt_tile * 16 +: 16] != isp_epoch[cnt_tile * 16 +: 16])begin
						flag_fixed_compatible[cnt_tile] <= 1'b0;
					end
					if(invalidate_mask[cnt_tile])begin
						flag_a_valid[cnt_tile] <= 1'b0;
						flag_jump_armed[cnt_tile] <= 1'b0;
					end else if(flag_pending)begin
						reg_active_a[cnt_tile] <= reg_result_a[cnt_tile];
						reg_stat_frame[cnt_tile] <= reg_result_frame[cnt_tile];
						flag_a_valid[cnt_tile] <= flag_result_valid[cnt_tile];
						flag_jump_armed[cnt_tile] <= flag_result_jump[cnt_tile];
						flag_jump_up[cnt_tile] <= flag_result_up[cnt_tile];
					end
				end else if(flag_publish_blank)begin
					reg_active_a[cnt_tile] <= reg_result_a[cnt_tile];
					reg_stat_frame[cnt_tile] <= reg_result_frame[cnt_tile];
					flag_a_valid[cnt_tile] <= flag_result_valid[cnt_tile] && !source_invalidate_mask[cnt_tile];
					flag_jump_armed[cnt_tile] <= flag_result_jump[cnt_tile] && !source_invalidate_mask[cnt_tile];
					flag_jump_up[cnt_tile] <= flag_result_up[cnt_tile];
				end else if(source_invalidate_mask[cnt_tile])begin
					flag_a_valid[cnt_tile] <= 1'b0;
					flag_jump_armed[cnt_tile] <= 1'b0;
				end
			end
		end
	end
endmodule
