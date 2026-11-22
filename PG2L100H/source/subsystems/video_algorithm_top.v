//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-16
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 视频算法子系统，在四宫格流上携带同步内容标签，接入算法 v2 配置、逐 tile ISP 变换和共享去雾。
// Module Name: video_algorithm_top
// File Name: source/subsystems/video_algorithm_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 视频算法子系统，在四宫格流上携带同步内容标签，接入算法 v2 配置、逐 tile ISP 变换和共享去雾。
// 共享去雾结果同时供 HDMI 与 PCIe；仅 HDMI 后级继续串联高斯、灰度中值和 Sobel，配置在像素帧边界更新并保留 epoch 对应关系。
//
// Dependencies: algorithm_control_v2、isp_control、video_hdmi_tag_adapter、isp_pipeline、algo_dehaze_pango、video_algorithm_tail。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-16
// Design Name: AVM PCIe hierarchy refactor
// Module Name: video_algorithm_top
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Structural extraction; original clocks, handshakes and latency retained.
// Dependencies: Original project modules instantiated below.
// Revision: 0.01 - Extracted from top_design.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module video_algorithm_top(
	input i_rgmii_clk,
	input i_isp_ext_valid,
	input i_isp_ext_query,
	input [519:0] i_isp_ext_config,
	output o_isp_ext_ready,
	output o_isp_ext_done,
	output [7:0] o_isp_ext_status,
	output [519:0] o_isp_ext_snapshot,
	output [15:0] o_isp_ext_epoch,
	output [1:0] o_isp_ext_source,
	input i_algo_ext_valid,
	input i_algo_ext_query,
	input [63:0] i_algo_ext_config,
	output o_algo_ext_ready,
	output o_algo_ext_done,
	output [7:0] o_algo_ext_status,
	output [63:0] o_algo_ext_snapshot,
	output [15:0] o_algo_ext_epoch,
	output [1:0] o_algo_ext_source,
	output o_algo_common_reset_n,
	output o_algo_net_rst_n,
	output o_algo_pixel_rst_n,
	output o_algo_ack_valid,
	output [143:0] o_algo_ack_payload,
	output [15:0] o_algo_ack_length,
	input i_algo_ack_ready,
	output o_isp_ack_valid,
	output [143:0] o_isp_ack_payload,
	output [15:0] o_isp_ack_length,
	input i_isp_ack_ready,
	input i_isp_start,
	input i_isp_valid,
	input [7:0] i_isp_data,
	input i_isp_end,
	input [15:0] i_isp_length,
	input i_isp_error,
	input i_hdmi_pixel_clk,
	input i_hdmi_pll_lock,
	input i_quad_base_vs,
	input i_quad_base_hs,
	input i_quad_base_de,
	input [23:0] i_quad_base_rgb,
	input i_quad_content_valid,
	input [11:0] i_quad_display_x,
	input [11:0] i_quad_display_y,
	input [1:0] i_quad_display_tile,
	input [9:0] i_quad_tile_x,
	input [8:0] i_quad_tile_y,
	input [3:0] i_quad_source_present,
	input i_quad_source_present_valid,
	output [23:0] o_dehaze_algo_rgb,
	output [159:0] o_dehaze_algo_tag,
	output [159:0] o_algo_output_tag,
	output o_hdmi_algo_vs,
	output [23:0] o_hdmi_algo_rgb,
	output [1:0] o_night_level,
	output o_dehaze_hud_enable,
	input i_monitor_algo_start,
	input i_monitor_algo_valid,
	input i_monitor_algo_end,
	input i_monitor_algo_error,
	input [7:0] i_monitor_algo_data,
	input [15:0] i_monitor_algo_length,
	input i_button_rst_n
);
	wire rgmii_clk;
	wire algo_common_reset_n;
	reg [1:0] algo_net_reset_sync /* synthesis syn_preserve = 1 */;
	reg [1:0] algo_pixel_reset_sync /* synthesis syn_preserve = 1 */;
	wire algo_net_rst_n;
	wire algo_pixel_rst_n;
	wire [63:0] algo_active_config;
	wire [15:0] algo_config_epoch;
	wire algo_mode_busy;
	wire algo_mode_pending;
	wire [1:0] algo_active_mode;
	wire [10:0] algo_active_threshold;
	wire algo_ack_valid;
	wire [143:0] algo_ack_payload;
	wire [15:0] algo_ack_length;
	wire algo_ack_ready;
	wire [31:0] algo_status_dropped;
	reg algo_base_vs_d1;
	wire algo_frame_apply;
	wire hdmi_pixel_clk;
	wire hdmi_pll_lock;
	wire quad_base_vs;
	wire quad_base_hs;
	wire quad_base_de;
	wire [23:0] quad_base_rgb;
	wire quad_content_valid;
	wire [11:0] quad_display_x;
	wire [11:0] quad_display_y;
	wire [1:0] quad_display_tile;
	wire [9:0] quad_tile_x;
	wire [8:0] quad_tile_y;
	wire [3:0] quad_source_present;
	wire quad_source_present_valid;
	wire [23:0] algo_tagged_rgb;
	wire [159:0] algo_input_tag;
	wire [23:0] dehaze_algo_rgb;
	wire [159:0] dehaze_algo_tag;
	wire [159:0] algo_output_tag;
	wire algo_tagged_boundary;
	wire [3:0] algo_tagged_alive;
	wire algo_tagged_alive_valid;
	wire hdmi_algo_vs;
	wire hdmi_algo_hs;
	wire hdmi_algo_de;
	wire [23:0] hdmi_algo_rgb;
	wire [1:0] hdmi_algo_active;
	wire hdmi_algo_frame_boundary;
	wire monitor_algo_start;
	wire monitor_algo_valid;
	wire monitor_algo_end;
	wire monitor_algo_error;
	wire [7:0] monitor_algo_data;
	wire [15:0] monitor_algo_length;
	wire [519:0] isp_active_config;
	wire [63:0] isp_effective_epoch;
	wire [15:0] isp_config_epoch;
	wire [23:0] isp_rgb;
	wire [159:0] isp_tag;
	wire isp_boundary;
	wire [63:0] isp_algo_config;
	wire [519:0] isp_output_config;
	wire [63:0] isp_output_epoch;
	wire [3:0] isp_alive;
	wire isp_alive_valid;
	wire [3:0] isp_transform_active;
	wire isp_adaptive_enable;
	wire isp_adaptive_pixel_enable;
	reg reg_weather_enable_pixel;
	wire [1:0] night_level;
	wire summary_valid;
	wire summary_complete;
	wire [1:0] summary_tile;
	wire [31:0] summary_frame;
	wire [15:0] summary_epoch;
	wire [7:0] summary_p10;
	wire [7:0] summary_p50;
	wire [7:0] summary_p90;
	wire [13:0] summary_low48;
	wire [13:0] summary_fog_count;
	wire [13:0] summary_texture_count;
	wire [13:0] summary_neutral_count;
	wire [7:0] weather_night;
	wire [7:0] weather_fog;
	wire [3:0] weather_valid;
	wire [3:0] weather_request;
	wire [31:0] weather_omega;
	wire [31:0] weather_tmin;
	wire [159:0] weather_actual;

	assign rgmii_clk = i_rgmii_clk;
	assign o_algo_common_reset_n = algo_common_reset_n;
	assign o_algo_net_rst_n = algo_net_rst_n;
	assign o_algo_pixel_rst_n = algo_pixel_rst_n;
	assign o_algo_ack_valid = algo_ack_valid;
	assign o_algo_ack_payload = algo_ack_payload;
	assign o_algo_ack_length = algo_ack_length;
	assign algo_ack_ready = i_algo_ack_ready;
	assign hdmi_pixel_clk = i_hdmi_pixel_clk;
	assign hdmi_pll_lock = i_hdmi_pll_lock;
	assign quad_base_vs = i_quad_base_vs;
	assign quad_base_hs = i_quad_base_hs;
	assign quad_base_de = i_quad_base_de;
	assign quad_base_rgb = i_quad_base_rgb;
	assign quad_content_valid = i_quad_content_valid;
	assign quad_display_x = i_quad_display_x;
	assign quad_display_y = i_quad_display_y;
	assign quad_display_tile = i_quad_display_tile;
	assign quad_tile_x = i_quad_tile_x;
	assign quad_tile_y = i_quad_tile_y;
	assign quad_source_present = i_quad_source_present;
	assign quad_source_present_valid = i_quad_source_present_valid;
	assign o_dehaze_algo_rgb = dehaze_algo_rgb;
	assign o_dehaze_algo_tag = dehaze_algo_tag;
	assign o_algo_output_tag = algo_output_tag;
	assign o_hdmi_algo_vs = hdmi_algo_vs;
	assign o_hdmi_algo_rgb = hdmi_algo_rgb;
	assign o_night_level = night_level;
	// 共用HUD按实际执行像素聚合，缺源与A预热不能由总开关伪报已去雾。
	assign o_dehaze_hud_enable = weather_actual[10] || weather_actual[26] ||
		weather_actual[42] || weather_actual[58];
	assign monitor_algo_start = i_monitor_algo_start;
	assign monitor_algo_valid = i_monitor_algo_valid;
	assign monitor_algo_end = i_monitor_algo_end;
	assign monitor_algo_error = i_monitor_algo_error;
	assign monitor_algo_data = i_monitor_algo_data;
	assign monitor_algo_length = i_monitor_algo_length;
	// 复用CDC已有两级同步，再在像素边界提交模式，避免可见行中途切换tag。
	always@(posedge hdmi_pixel_clk or negedge algo_pixel_rst_n)begin
		if(!algo_pixel_rst_n)begin reg_weather_enable_pixel <= 1'b0;
		end else if(isp_boundary)begin reg_weather_enable_pixel <= isp_adaptive_pixel_enable; end
	end

	// PLL失锁也同时复位mailbox两端；各域独立同步释放。
	assign algo_common_reset_n = i_button_rst_n && hdmi_pll_lock;
	assign algo_net_rst_n = algo_net_reset_sync[1];
	assign algo_pixel_rst_n = algo_pixel_reset_sync[1];
	always @(posedge rgmii_clk or negedge algo_common_reset_n) begin
		if (!algo_common_reset_n) begin
			algo_net_reset_sync <= 2'b00;
		end else begin
			algo_net_reset_sync <= {algo_net_reset_sync[0], 1'b1};
		end
	end
	always @(posedge hdmi_pixel_clk or negedge algo_common_reset_n) begin
		if (!algo_common_reset_n) begin
			algo_pixel_reset_sync <= 2'b00;
		end else begin
			algo_pixel_reset_sync <= {algo_pixel_reset_sync[0], 1'b1};
		end
	end
	// 使用quad_base VS下降沿的垂直消隐窗口，不复用DDR读帧事件。
	always @(posedge hdmi_pixel_clk or negedge algo_pixel_rst_n) begin
		if (!algo_pixel_rst_n) begin
			algo_base_vs_d1 <= 1'b0;
		end else begin
			algo_base_vs_d1 <= quad_base_vs;
		end
	end
	assign algo_frame_apply = algo_base_vs_d1 && !quad_base_vs;

	// v2配置进入共享ISP/去雾链；其结果供HDMI后级和PCIe采集使用。
	algorithm_control_v2 algorithm_control_v2_Inst_control(
		.i_ext_valid(i_algo_ext_valid),
		.i_ext_query(i_algo_ext_query),
		.i_ext_config(i_algo_ext_config),
		.o_ext_ready(o_algo_ext_ready),
		.o_ext_done(o_algo_ext_done),
		.o_ext_status(o_algo_ext_status),
		.o_ext_snapshot(o_algo_ext_snapshot),
		.o_ext_epoch(o_algo_ext_epoch),
		.o_ext_source(o_algo_ext_source),
		.i_control_clk(rgmii_clk),
		.i_control_rstn(algo_net_rst_n),
		.i_pixel_clk(hdmi_pixel_clk),
		.i_pixel_rstn(algo_pixel_rst_n),
		.i_start(monitor_algo_start),
		.i_valid(monitor_algo_valid),
		.i_data(monitor_algo_data),
		.i_end(monitor_algo_end),
		.i_length(monitor_algo_length),
		.i_error(monitor_algo_error),
		.i_frame_boundary(algo_frame_apply),
		.o_active_config(algo_active_config),
		.o_config_epoch(algo_config_epoch),
		.o_busy(algo_mode_busy),
		.o_pending(algo_mode_pending),
		.o_ack_valid(algo_ack_valid),
		.o_ack_payload(algo_ack_payload),
		.o_ack_length(algo_ack_length),
		.i_ack_ready(algo_ack_ready),
		.o_dropped_status_count(algo_status_dropped)
	);
	// 旧bridge临时兼容解码；真串联集成直接使用64位配置及epoch。
	assign algo_active_mode = (algo_active_config[59] || algo_active_config[19]) ? 2'd3 :
		((algo_active_config[58] || algo_active_config[18]) ? 2'd1 :
		((algo_active_config[57] || algo_active_config[17]) ? 2'd2 : 2'd0));
	assign algo_active_threshold = algo_active_config[10:0];
	// v028：更新原因与effective epoch同拍通过ISP，只有连续自动更新允许A平滑跟踪。
	wire isp_continuous_update;
	wire isp_output_continuous_update;
	isp_control isp_control_Inst_control(
		.i_ext_valid(i_isp_ext_valid),
		.i_ext_query(i_isp_ext_query),
		.i_ext_config(i_isp_ext_config),
		.o_ext_ready(o_isp_ext_ready),
		.o_ext_done(o_isp_ext_done),
		.o_ext_status(o_isp_ext_status),
		.o_ext_snapshot(o_isp_ext_snapshot),
		.o_ext_epoch(o_isp_ext_epoch),
		.o_ext_source(o_isp_ext_source),
		.i_control_clk(rgmii_clk),.i_control_rstn(algo_net_rst_n),
		.i_pixel_clk(hdmi_pixel_clk),.i_pixel_rstn(algo_pixel_rst_n),
		.i_start(i_isp_start),.i_valid(i_isp_valid),.i_data(i_isp_data),
		.i_end(i_isp_end),.i_length(i_isp_length),.i_error(i_isp_error),
		// ISP前scene决定场景，ISP后反馈仅限制新增裁剪；同frame_id配对。
		// 独立ISP配置epoch用于输入摘要一致性，不读取tag内算法epoch。
		.i_adaptive_rgb(isp_rgb),.i_adaptive_tag(isp_tag),
		.i_scene_rgb(algo_tagged_rgb),.i_scene_tag(algo_input_tag),
		.i_scene_frame_boundary(algo_tagged_boundary),.i_scene_epoch(isp_config_epoch),
		.i_frame_boundary(isp_boundary),.o_active_config(isp_active_config),
		.o_config_epoch(isp_config_epoch),.o_effective_epoch(isp_effective_epoch),
		.o_isp_continuous_update(isp_continuous_update),
		.o_change_mask(),.o_busy(),.o_pending(),
		.o_ack_valid(o_isp_ack_valid),.o_ack_payload(o_isp_ack_payload),
		.o_ack_length(o_isp_ack_length),.i_ack_ready(i_isp_ack_ready),
		.o_dropped_status_count(),.o_adaptive_enable(isp_adaptive_enable),
		.o_adaptive_pixel_enable(isp_adaptive_pixel_enable),
		.i_weather_actual(weather_actual),
		.o_summary_valid(summary_valid),.o_summary_complete(summary_complete),
		.o_summary_tile(summary_tile),.o_summary_frame(summary_frame),
		.o_summary_epoch(summary_epoch),.o_summary_p10(summary_p10),
		.o_summary_p50(summary_p50),.o_summary_p90(summary_p90),
		.o_summary_low48(summary_low48),.o_summary_fog_count(summary_fog_count),
		.o_summary_texture_count(summary_texture_count),
		.o_summary_neutral_count(summary_neutral_count));


	// 四宫格在此携带完整标签，PCIe取后面的共享去雾结果，不含HDMI后级与HUD。
	video_hdmi_tag_adapter u_hdmi_tag_adapter(
		.i_clk(hdmi_pixel_clk),.i_rst_n(algo_pixel_rst_n),
		.i_vs(quad_base_vs),.i_hs(quad_base_hs),.i_de(quad_base_de),.i_rgb(quad_base_rgb),
		.i_x(quad_display_x[10:0]),.i_y(quad_display_y[9:0]),.i_tile(quad_display_tile),
		.i_tile_x(quad_tile_x),.i_tile_y(quad_tile_y),.i_content_valid(quad_content_valid),
		.i_frame_boundary(algo_frame_apply),.i_config(algo_active_config),.i_epoch(algo_config_epoch),
		.i_source_alive(quad_source_present),.i_source_alive_valid(quad_source_present_valid),
		.o_rgb(algo_tagged_rgb),.o_tag(algo_input_tag),.o_frame_boundary(algo_tagged_boundary),
		.o_source_alive(algo_tagged_alive),.o_source_alive_valid(algo_tagged_alive_valid));
	isp_pipeline #(.C_COMPACT_CONFIG(1)) isp_pipeline_Inst_quality(
		.i_clk(hdmi_pixel_clk),.i_rstn(algo_pixel_rst_n),
		.i_rgb(algo_tagged_rgb),.i_tag(algo_input_tag),
		.i_frame_boundary(algo_tagged_boundary),.i_algo_config(algo_active_config),
		.i_isp_config(isp_active_config),.i_effective_epoch(isp_effective_epoch),
		.i_isp_continuous_update(isp_continuous_update),
		.i_source_alive(algo_tagged_alive),.i_source_alive_valid(algo_tagged_alive_valid),
		.o_rgb(isp_rgb),.o_tag(isp_tag),.o_frame_boundary(isp_boundary),
		.o_algo_config(isp_algo_config),.o_isp_config(isp_output_config),
		.o_effective_epoch(isp_output_epoch),.o_source_alive(isp_alive),
		.o_isp_continuous_update(isp_output_continuous_update),
		.o_source_alive_valid(isp_alive_valid));
	// ISP前原始统计逐视频源判定，不能把四区亮度混合成一个夜间等级。
	isp_weather_control #(.C_FOG_OMEGA_STEP(2),.C_FOG_TMIN_STEP(1)) u_isp_weather_control(
		.i_clk(hdmi_pixel_clk),.i_rst_n(algo_pixel_rst_n),
		.i_enable(reg_weather_enable_pixel),.i_frame_boundary(isp_boundary),
		.i_summary_valid(summary_valid),.i_summary_complete(summary_complete),
		.i_summary_tile(summary_tile),.i_summary_frame(summary_frame),
		.i_summary_epoch(summary_epoch),.i_summary_p10(summary_p10),
		.i_summary_p50(summary_p50),.i_summary_p90(summary_p90),
		.i_summary_low48(summary_low48),.i_summary_fog_count(summary_fog_count),
		.i_summary_texture_count(summary_texture_count),
		.i_summary_neutral_count(summary_neutral_count),
		.o_night(weather_night),.o_fog(weather_fog),.o_valid(weather_valid),
		.o_omega(weather_omega),.o_tmin(weather_tmin),
		.o_night_level(night_level),.o_fog_request(weather_request));
	genvar isp_tile;
	generate
		for(isp_tile = 0; isp_tile < 4; isp_tile = isp_tile + 1)begin:gen_isp_transform
			assign isp_transform_active[isp_tile] = isp_output_config[512] &&
				isp_output_config[513 + isp_tile] &&
				(|isp_output_config[isp_tile * 128 + 120 +: 6]);
		end
	endgenerate
	// 算法层 A：去雾结果是 HDMI/PCIe 共用视频源。
	algo_dehaze_pango #(.C_ISP_ENABLE(1),.C_WEATHER_ENABLE(1)) u_shared_dehaze(
		.i_clk(hdmi_pixel_clk),.i_rst_n(algo_pixel_rst_n),
		.i_rgb(isp_rgb),.i_tag(isp_tag),
		.i_frame_boundary(isp_boundary),.i_config(isp_algo_config),
		.i_source_alive(isp_alive),.i_source_alive_valid(isp_alive_valid),
		.i_isp_epoch(isp_output_epoch),.i_isp_active(isp_transform_active),
		.i_isp_continuous_update(isp_output_continuous_update),
		.i_weather_enable(reg_weather_enable_pixel),.i_weather_night(weather_night),
		.i_weather_fog(weather_fog),.i_weather_valid(weather_valid),
		.i_weather_request(weather_request),.i_weather_omega(weather_omega),
		.i_weather_tmin(weather_tmin),.o_weather_actual(weather_actual),
		.o_rgb(dehaze_algo_rgb),.o_tag(dehaze_algo_tag),.o_all_a(),.o_all_a_valid());
	// 算法层 B：仅 HDMI 使用的高斯、灰度中值和 Sobel 串联。
	video_algorithm_tail u_hdmi_algorithm_tail(
		.i_clk(hdmi_pixel_clk),.i_rst_n(algo_pixel_rst_n),
		.i_rgb(dehaze_algo_rgb),.i_tag(dehaze_algo_tag),
		.o_rgb(hdmi_algo_rgb),.o_tag(algo_output_tag));
	assign hdmi_algo_vs=algo_output_tag[159];
	assign hdmi_algo_hs=algo_output_tag[158];
	assign hdmi_algo_de=algo_output_tag[157];
	assign hdmi_algo_active=(algo_output_tag[59] || algo_output_tag[19]) ? 2'd3 :
		((algo_output_tag[58] || algo_output_tag[18]) ? 2'd1 :
		((algo_output_tag[57] || algo_output_tag[17]) ? 2'd2 : 2'd0));
	assign hdmi_algo_frame_boundary=isp_boundary;


endmodule
