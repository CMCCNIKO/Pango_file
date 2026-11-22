//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 串联 tile 因果暗通道、上一帧大气光估计和 RGB 恢复，按配置执行完整去雾处理。
// Module Name: algo_dehaze_pango
// File Name: source/video/dehaze/algo_dehaze_pango.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 串联 tile 因果暗通道、上一帧大气光估计和 RGB 恢复，按配置执行完整去雾处理。
// 暗通道 2 拍与像素恢复 66 拍合计固定 68 拍，II=1；tile 局部坐标用于空间大气光估计并保持 tag 对齐。
// v026复用共享去雾，按视频源覆盖天气参数；在真实恢复输入处采集实际处理状态，下一帧边界冻结。
//
// Dependencies: dehaze_atmosphere_estimator、dehaze_dark_channel、dehaze_pixel_restore
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-16 / v026 - 四源天气参数覆盖、同延迟旁路及实际像素天气快照。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Module: algo_dehaze_pango
// Project: PG2L100H HDMI dehaze; PDS 2022.2-SP6.4
// Device: PG2L100H-6FBG676
// Description: 因果暗通道2拍+参数组合0拍+RGB恢复66拍，总68拍II1。
// Dependencies: 本次真实官方RAM/ROM/APM、atmosphere estimator。
// Revision: 1.0 初始串联。
//           1.1 2026-10-21, local coordinates for robust spatial atmosphere A.
//////////////////////////////////////////////////////////////////////////////
module algo_dehaze_pango
#(
	parameter integer C_ISP_ENABLE = 0,
	parameter integer C_WEATHER_ENABLE = 0
)
(
	input wire i_clk,
	input wire i_rst_n,
	input wire [23:0] i_rgb,
	input wire [159:0] i_tag,
	input wire i_frame_boundary,
	input wire [63:0] i_config,
	input wire [3:0] i_source_alive,
	input wire i_source_alive_valid,
	input wire [63:0] i_isp_epoch,
	input wire [3:0] i_isp_active,
	input wire i_isp_continuous_update,
	input wire i_weather_enable,
	input wire [7:0] i_weather_night,
	input wire [7:0] i_weather_fog,
	input wire [3:0] i_weather_valid,
	input wire [3:0] i_weather_request,
	input wire [31:0] i_weather_omega,
	input wire [31:0] i_weather_tmin,
	output wire [23:0] o_rgb,
	output wire [159:0] o_tag,
	output wire [31:0] o_all_a,
	output wire [3:0] o_all_a_valid,
	output wire [159:0] o_weather_actual
);
	wire [23:0] dark_rgb;
	wire [159:0] dark_tag;
	wire [7:0] dark;
	wire [7:0] atmosphere;
	wire a_valid;
	wire [159:0] effective_dark_tag;
	wire [63:0] effective_config;
	reg [127:0] reg_weather_work;
	reg [3:0] reg_weather_seen;
	reg [31:0] reg_weather_frame;
	reg [159:0] weather_actual_o;
	wire [1:0] weather_tile;
	wire weather_requested;
	wire weather_applied;
	integer weather_index;
	reg [1:0] boundary_pipe;
	reg [3:0] alive_pipe [0:1];
	reg [1:0] alive_valid_pipe;
	reg [63:0] config_pipe [0:1];
	reg [63:0] isp_epoch_pipe [0:1];
	reg [3:0] isp_active_pipe [0:1];
	reg [1:0] isp_continuous_pipe;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			boundary_pipe<=0; alive_pipe[0]<=0; alive_pipe[1]<=0; alive_valid_pipe<=0;
			config_pipe[0]<=64'h000fe619dc010080;config_pipe[1]<=64'h000fe619dc010080;
			isp_epoch_pipe[0]<=0;isp_epoch_pipe[1]<=0;
			isp_active_pipe[0]<=0;isp_active_pipe[1]<=0;
			isp_continuous_pipe<=0;
		end
		else begin
			boundary_pipe<={boundary_pipe[0],i_frame_boundary};
			alive_pipe[0]<=i_source_alive; alive_pipe[1]<=alive_pipe[0];
			alive_valid_pipe<={alive_valid_pipe[0],i_source_alive_valid};
			config_pipe[0]<=i_config;config_pipe[1]<=config_pipe[0];
			isp_epoch_pipe[0]<=i_isp_epoch;isp_epoch_pipe[1]<=isp_epoch_pipe[0];
			isp_active_pipe[0]<=i_isp_active;isp_active_pipe[1]<=isp_active_pipe[0];
			isp_continuous_pipe<={isp_continuous_pipe[0],i_isp_continuous_update};
		end
	end
	dehaze_dark_channel u_dark(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_rgb(i_rgb),.i_tag(i_tag),
		.o_rgb(dark_rgb),.o_tag(dark_tag),.o_dark(dark));
	// 天气寄存器在ISP边界发布，2拍后的暗通道边界使用同一组新参数。
	// 不将新天气mask先送入2拍config_pipe，避免A估计器晚一帧看到使能。
	isp_weather_dehaze_config u_weather_config(
		.i_enable(C_WEATHER_ENABLE != 0 && i_weather_enable),
		.i_valid(i_weather_valid),.i_fog_request(i_weather_request),
		.i_omega(i_weather_omega),.i_tmin(i_weather_tmin),
		.i_tag(dark_tag),.i_config(config_pipe[1]),
		.o_tag(effective_dark_tag),.o_config(effective_config));
	dehaze_atmosphere_estimator #(.C_ISP_ENABLE(C_ISP_ENABLE),.C_WEATHER_ENABLE(C_WEATHER_ENABLE)) u_atmosphere(
		.i_clk(i_clk),.i_rstn(i_rst_n),.i_de(dark_tag[157]),.i_content_valid(dark_tag[112]),
		.i_tile(dark_tag[133:132]),.i_dark(dark),.i_rgb(dark_rgb),.i_frame_id(dark_tag[111:80]),
		.i_local_x(dark_tag[131:122]),.i_local_y(dark_tag[121:113]),
		.i_frame_boundary(boundary_pipe[1]),.i_config(effective_config),.i_source_alive(alive_pipe[1]),
		.i_source_alive_valid(alive_valid_pipe[1]),
		.i_isp_epoch(isp_epoch_pipe[1]),.i_isp_active(isp_active_pipe[1]),
		.i_weather_tracking(C_WEATHER_ENABLE != 0 && i_weather_enable),
		.i_isp_continuous_update(isp_continuous_pipe[1]),
		.o_active_a(atmosphere),.o_a_valid(a_valid),.o_stat_frame(),
		.o_all_a(o_all_a),.o_all_a_valid(o_all_a_valid),.o_all_stat_frame());
	dehaze_pixel_restore u_restore(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_rgb(dark_rgb),.i_tag(effective_dark_tag),
		.i_dark(dark),.i_atmosphere(atmosphere),.i_a_valid(a_valid),.o_rgb(o_rgb),.o_tag(o_tag));
	assign weather_tile = effective_dark_tag[133:132];
	assign weather_requested = effective_dark_tag[56] && effective_dark_tag[48 + weather_tile];
	assign weather_applied = weather_requested && a_valid;
	assign o_weather_actual = weather_actual_o;
	// 记录真正送入恢复流水的像素参数，而不是下一帧候选值。
	// Aready在有效像素处采集；空白期o_a_valid带DE为0不能当作帧级ready。
	// 工作帧在下一暗通道边界冻结，使用独立frame_id标示回读的处理帧。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_weather_work <= 128'd0;
			reg_weather_seen <= 4'd0;
			reg_weather_frame <= 32'd0;
			weather_actual_o <= {32'd0,64'd0,{4{16'hf000}}};
		end else if(boundary_pipe[1])begin
			weather_actual_o[159:128] <= reg_weather_frame;
			for(weather_index = 0; weather_index < 4; weather_index = weather_index + 1)begin
				weather_actual_o[weather_index * 16 +: 16] <= reg_weather_seen[weather_index] ?
					reg_weather_work[weather_index * 16 +: 16] : 16'hf000;
				weather_actual_o[64 + weather_index * 16 +: 16] <= reg_weather_seen[weather_index] ?
					reg_weather_work[64 + weather_index * 16 +: 16] : 16'd0;
			end
			reg_weather_seen <= 4'd0;
		end else if(effective_dark_tag[157])begin
			reg_weather_frame <= effective_dark_tag[111:80];
			if(effective_dark_tag[112])begin
				reg_weather_seen[weather_tile] <= 1'b1;
				reg_weather_work[weather_tile * 16 +: 16] <= {
					(C_WEATHER_ENABLE != 0 && i_weather_enable ? i_weather_night[weather_tile * 2 +: 2] : 2'd3),
					(C_WEATHER_ENABLE != 0 && i_weather_enable ? i_weather_fog[weather_tile * 2 +: 2] : 2'd3),
					(C_WEATHER_ENABLE != 0 && i_weather_enable && i_weather_valid[weather_tile]),
					weather_applied,weather_requested,a_valid,effective_dark_tag[47:40]};
				reg_weather_work[64 + weather_tile * 16 +: 16] <= {effective_dark_tag[39:32],atmosphere};
			end
		end
	end
endmodule
