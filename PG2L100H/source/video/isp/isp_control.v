//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 接收 Ethernet 分流后的 IS v1 绝对 ISP 设置，通过配置 CDC 执行帧原子提交并生成 18 字节状态应答。
// Module Name: isp_control
// File Name: source/video/isp/isp_control.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 接收 Ethernet 分流后的 IS v1 绝对 ISP 设置，通过配置 CDC 执行帧原子提交并生成 18 字节状态应答。
// 在字段接收时预译码命令和应答页，保留完整包校验、合法提交及原应答拍数。
//
// Dependencies: isp_config_cdc；完整分流 UDP 字节流及 ACK 发送接口。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-16 / v025 - 增加scene接口与冻结实时只读页9..18，保留原配置ACK语义。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: Ethernet ISP control
// Module Name: isp_control
// Project Name: PG2L100H_AVM_ISP
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: IS v1 absolute settings, frame-atomic CDC and 18-byte status.
//              No video/IP dependencies; coordinated reset is required.
// Dependencies: isp_config_cdc; byte stream is a complete dispatched UDP packet.
// Revision: 0.03 - 2026-10-23，提前寄存定长包判定，保持提交及应答拍数。
//           0.04 - 2026-10-24，载荷预装与合法提交分离，切断配置校验至宽使能。
//           0.05 - 2026-10-29，随字段写入预译码命令/有效应答页，保持协议拍数。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module isp_control
(
	input i_control_clk,
	input i_control_rstn,
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	input i_frame_boundary,
	input [23:0] i_adaptive_rgb,
	input [159:0] i_adaptive_tag,
	input [23:0] i_scene_rgb,
	input [159:0] i_scene_tag,
	input i_scene_frame_boundary,
	input [15:0] i_scene_epoch,
	input [159:0] i_weather_actual,
	output o_summary_valid,
	output o_summary_complete,
	output [1:0] o_summary_tile,
	output [31:0] o_summary_frame,
	output [15:0] o_summary_epoch,
	output [7:0] o_summary_p10,
	output [7:0] o_summary_p50,
	output [7:0] o_summary_p90,
	output [13:0] o_summary_low48,
	output [13:0] o_summary_fog_count,
	output [13:0] o_summary_texture_count,
	output [13:0] o_summary_neutral_count,
	input i_ext_valid,
	input i_ext_query,
	input [519:0] i_ext_config,
	output o_ext_ready,
	output o_ext_done,
	output [7:0] o_ext_status,
	output [519:0] o_ext_snapshot,
	output [15:0] o_ext_epoch,
	output [1:0] o_ext_source,
	output [519:0] o_active_config,
	output [15:0] o_config_epoch,
	output [63:0] o_effective_epoch,
	output o_isp_continuous_update,
	output [3:0] o_change_mask,
	output o_busy,
	output o_pending,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	input i_ack_ready,
	output [31:0] o_dropped_status_count,
	output o_adaptive_enable,
	output o_adaptive_pixel_enable
);
	localparam [127:0] DEFAULT_TILE = {8'd0,10'd256,10'd256,10'd256,
		8'd0,8'd0,8'd0,2'd1,3'd1,8'd12,8'd32,3'd1,8'd4,8'd16,3'd0,3'd0,20'd0};
	reg [7:0] reg_bytes [0:23];
	reg flag_opcode_global;
	reg flag_opcode_tile;
	reg flag_opcode_query;
	reg flag_opcode_adaptive;
	reg [21:0] reg_query_page;
	wire flag_write_opcode;
	wire flag_write_page;
	reg [4:0] cnt_bytes;
	wire [4:0] cnt_bytes_next;
	reg flag_count8;
	reg flag_count24;
	reg flag_declared8;
	reg flag_declared24;
	reg flag_receiving;
	reg flag_rx_error;
	reg flag_evaluate;
	reg flag_end_error;
	reg update_o;
	reg [519:0] reg_submit;
	reg [15:0] reg_sequence;
	reg [7:0] reg_opcode;
	reg [143:0] reg_queue [0:7];
	reg [2:0] reg_write;
	reg [2:0] reg_read;
	reg [3:0] cnt_queue;
	reg [31:0] dropped_status_count_o;
	reg adaptive_enable_o;
	wire flag_cdc_busy;
	wire flag_applied;
	wire [519:0] active_snapshot;
	wire [15:0] snapshot_epoch;
	wire [519:0] live_config;
	wire [15:0] live_epoch;
	wire [63:0] live_status;
	wire [15:0] live_id;
	wire live_enable;
	wire live_valid;
	wire [159:0] live_weather_actual;
	reg [519:0] reg_telemetry_config;
	reg [63:0] reg_telemetry_status;
	reg [63:0] reg_telemetry_header;
	reg [159:0] reg_telemetry_weather_actual;
	// 第9页原子冻结整个遥测读窗口；第10..21页读取同一快照。
	// 收到页字段即预装只读快照，校验仅控制ACK，不驱动宽总线CE。
	wire flag_freeze_telemetry = flag_write_page && flag_opcode_query && i_data == 8'd9;
	wire [127:0] dec_tile;
	wire [15:0] dec_sequence;
	wire [7:0] dec_opcode;
	wire [7:0] dec_page;
	wire flag_pop;
	wire flag_space;
	wire flag_event_space;
	wire flag_evaluate_now;
	wire flag_push;
	wire flag_submit;
	reg [7:0] dec_error;
	reg [519:0] dec_config;
	reg [63:0] dec_page_data;
	reg [143:0] dec_ack;
	reg [127:0] dec_tile_accepted;
	integer tile_index;
	reg [519:0] reg_authoritative;
	reg [519:0] reg_eth_pending_config;
	reg flag_eth_pending;
	reg [1:0] reg_inflight_source;
	reg [1:0] reg_applied_source;
	reg ext_done_o;
	reg [7:0] ext_status_o;
	reg [519:0] ext_snapshot_o;
	reg [15:0] ext_epoch_o;
	reg [1:0] ext_source_o;
	reg flag_ext_config_valid;
	integer ext_tile_index;
	wire flag_eth_applied;
	wire flag_ext_accept;
	wire flag_launch_eth;

	assign o_busy = flag_cdc_busy || update_o || flag_eth_pending || reg_inflight_source != 2'd0;
	assign o_ext_ready = !ext_done_o && reg_inflight_source == 2'd0 && !flag_cdc_busy && !update_o &&
		!flag_eth_pending && !flag_receiving && !flag_evaluate && !i_start && !i_end;
	assign flag_ext_accept = i_ext_valid && o_ext_ready;
	assign flag_eth_applied = flag_applied && reg_inflight_source == 2'd1;
	assign flag_launch_eth = reg_inflight_source == 2'd0 && !flag_cdc_busy &&
		!update_o && flag_eth_pending;
	assign o_ext_done = ext_done_o;
	assign o_ext_status = ext_status_o;
	assign o_ext_snapshot = ext_snapshot_o;
	assign o_ext_epoch = ext_epoch_o;
	assign o_ext_source = ext_source_o;
	assign o_ack_valid = cnt_queue != 0;
	assign o_ack_payload = reg_queue[reg_read];
	assign o_ack_length = 16'd18;
	assign o_dropped_status_count = dropped_status_count_o;
	assign o_adaptive_enable = adaptive_enable_o;
	assign dec_opcode = reg_bytes[3];
	assign dec_sequence = cnt_bytes >= 5'd6 ? {reg_bytes[4],reg_bytes[5]} : 16'd0;
	// 协议仅接受8/24字节；超长包饱和在31，仍返回原长度错误与序号。
	assign cnt_bytes_next = i_start ? (i_valid ? 5'd1 : 5'd0) :
		(flag_receiving && i_valid && cnt_bytes != 5'd31 ? cnt_bytes + 5'd1 : cnt_bytes);
	assign dec_page = flag_opcode_query ? reg_bytes[6] : 8'd0;
	assign flag_write_opcode = i_valid && !i_start && flag_receiving && cnt_bytes == 5'd3;
	assign flag_write_page = i_valid && !i_start && flag_receiving && cnt_bytes == 5'd6;
	assign dec_tile = {reg_bytes[8],reg_bytes[9],reg_bytes[10],reg_bytes[11],
		reg_bytes[12],reg_bytes[13],reg_bytes[14],reg_bytes[15],
		reg_bytes[16],reg_bytes[17],reg_bytes[18],reg_bytes[19],
		reg_bytes[20],reg_bytes[21],reg_bytes[22],reg_bytes[23]};
	assign flag_pop = o_ack_valid && i_ack_ready;
	assign flag_space = cnt_queue < 8 || flag_pop;
	assign flag_event_space = ((reg_inflight_source == 2'd1 || flag_eth_pending) ? cnt_queue < 7 : cnt_queue < 8) || flag_pop;
	assign flag_evaluate_now = flag_evaluate && !flag_eth_applied;
	assign flag_push = (flag_eth_applied && flag_space) || (flag_evaluate_now && flag_event_space);
	// Reserve accepted + applied slots before starting a transaction.
	assign flag_submit = flag_evaluate_now && dec_error == 0 && !flag_opcode_query && !flag_opcode_adaptive;

	// 译码是已收字段的同步镜像，不能在start/end/evaluate时清空。
	// 更新任一字段时都用另一字段的当前值，短包/覆盖/延后评价保持旧语义。
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			flag_opcode_global <= 1'b0;
			flag_opcode_tile <= 1'b0;
			flag_opcode_query <= 1'b0;
			flag_opcode_adaptive <= 1'b0;
		end else if(flag_write_opcode)begin
			flag_opcode_global <= i_data == 8'd1;
			flag_opcode_tile <= i_data == 8'd2;
			flag_opcode_query <= i_data == 8'd3;
			flag_opcode_adaptive <= i_data == 8'd4;
		end
	end
	genvar page_slot;
	generate
		for(page_slot = 0; page_slot < 22; page_slot = page_slot + 1)begin:gen_query_page_ff
			always@(posedge i_control_clk or negedge i_control_rstn)begin
				if(!i_control_rstn)begin
					reg_query_page[page_slot] <= page_slot == 0;
				end else if(flag_write_opcode)begin
					if(page_slot == 0)begin
						reg_query_page[page_slot] <= i_data != 8'd3 || reg_bytes[6] == 8'd0 || reg_bytes[6] > 8'd21;
					end else begin
						reg_query_page[page_slot] <= i_data == 8'd3 && reg_bytes[6] == page_slot;
					end
				end else if(flag_write_page)begin
					if(page_slot == 0)begin
						reg_query_page[page_slot] <= !flag_opcode_query || i_data == 8'd0 || i_data > 8'd21;
					end else begin
						reg_query_page[page_slot] <= flag_opcode_query && i_data == page_slot;
					end
				end
			end
		end
	endgenerate

	genvar byte_slot;
	generate
		for(byte_slot = 0; byte_slot < 24; byte_slot = byte_slot + 1)begin:gen_packet_ff
			always@(posedge i_control_clk or negedge i_control_rstn)begin
				if(!i_control_rstn)begin
					reg_bytes[byte_slot] <= 8'd0;
				end else if(i_valid && ((i_start && byte_slot == 0) ||
					(!i_start && flag_receiving && cnt_bytes == byte_slot)))begin
					reg_bytes[byte_slot] <= i_data;
				end
			end
		end
	endgenerate
	// Evaluate after the final byte, including streams with end and valid together.
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			cnt_bytes <= 0;
			flag_count8 <= 1'b0;
			flag_count24 <= 1'b0;
			flag_declared8 <= 1'b0;
			flag_declared24 <= 1'b0;
			flag_receiving <= 0;
			flag_rx_error <= 0;
			flag_evaluate <= 0;
			flag_end_error <= 0;
		end else begin
			cnt_bytes <= cnt_bytes_next;
			// 与更新后的计数逐拍一致，兼容end+valid及延后evaluate期间的新start。
			// 长度比较仅驱动少量标志FF，不再直接扇出到520bit配置提交使能。
			flag_count8 <= cnt_bytes_next == 5'd8;
			flag_count24 <= cnt_bytes_next == 5'd24;
			if(flag_evaluate_now)begin
				flag_evaluate <= 0;
			end
			if(i_start)begin
				flag_receiving <= 1;
				flag_rx_error <= i_error;
			end else if(flag_receiving)begin
				if(i_error)begin
					flag_rx_error <= 1;
				end
			end
			if(i_end)begin
				flag_receiving <= 0;
				flag_evaluate <= 1;
				flag_declared8 <= i_length == 16'd8;
				flag_declared24 <= i_length == 16'd24;
				flag_end_error <= i_error || flag_rx_error || !flag_receiving;
			end
		end
	end

	always@(*)begin
		dec_error = 0;
		if(flag_end_error)begin
			dec_error = 6;
		end else if(flag_opcode_tile ? !(flag_count24 && flag_declared24) :
			!(flag_count8 && flag_declared8))begin
			dec_error = 1;
		end else if({reg_bytes[0],reg_bytes[1]} != 16'h4953)begin
			dec_error = 2;
		end else if(reg_bytes[2] != 1)begin
			dec_error = 3;
		end else if(!(flag_opcode_global || flag_opcode_tile || flag_opcode_query || flag_opcode_adaptive))begin
			dec_error = 4;
		end else if(flag_opcode_adaptive && (reg_bytes[6] > 8'd1 || reg_bytes[7] != 8'd0))begin
			dec_error = 5;
		end else if(adaptive_enable_o && (flag_opcode_global || flag_opcode_tile))begin
			dec_error = 8;
		end else if(flag_opcode_global && (reg_bytes[6] > 1 || reg_bytes[7] > 15))begin
			dec_error = 5;
		end else if(flag_opcode_query && (reg_bytes[6] > 21 || reg_bytes[7] != 0))begin
			dec_error = 5;
		end else if(flag_opcode_tile && (reg_bytes[6] == 0 || reg_bytes[6] > 15 ||
			reg_bytes[7] != 0 || dec_tile[127:126] != 0 || dec_tile[65:64] > 1 ||
			dec_tile[63:61] > 4 || dec_tile[44:42] > 4 || dec_tile[25:23] > 3 ||
			dec_tile[22:20] > 4 || dec_tile[19:0] != 0))begin
			dec_error = 5;
		end else if(!flag_opcode_query && (reg_inflight_source == 2'd1 || flag_eth_pending || cnt_queue > 5))begin
			dec_error = 7;
		end
	end
	always@(*)begin
		// These reserved bits are rejected above and are protocol-defined zero.
		// Keep them explicit in the accepted image so synthesis can propagate
		// constants through the tile fanout and remove redundant data logic.
		dec_tile_accepted = dec_tile;
		dec_tile_accepted[127:126] = 2'd0;
		dec_tile_accepted[65] = 1'b0;
		dec_tile_accepted[25] = 1'b0;
		dec_tile_accepted[19:0] = 20'd0;
		// Merge partial UDP updates into the shared accepted image, including
		// an external update already frozen but still awaiting a frame boundary.
		dec_config = reg_authoritative;
		dec_config[519:517] = 3'd0;
		if(flag_opcode_global)begin
			dec_config[519:512] = {3'd0,reg_bytes[7][3:0],reg_bytes[6][0]};
		end else if(flag_opcode_tile)begin
			for(tile_index = 0; tile_index < 4; tile_index = tile_index + 1)begin
				if(reg_bytes[6][tile_index])begin
					dec_config[tile_index * 128 +: 128] = dec_tile_accepted;
				end
			end
		end
	end
	always@(*)begin
		// 互斥选择已在字段接收时寄存；应答周期只做并行数据选择。
		dec_page_data = ({64{reg_query_page[0]}} & {snapshot_epoch,active_snapshot[127:120],
			active_snapshot[255:248],active_snapshot[383:376],active_snapshot[511:504],16'd0}) |
			({64{reg_query_page[1]}} & active_snapshot[127:64]) |
			({64{reg_query_page[2]}} & active_snapshot[63:0]) |
			({64{reg_query_page[3]}} & active_snapshot[255:192]) |
			({64{reg_query_page[4]}} & active_snapshot[191:128]) |
			({64{reg_query_page[5]}} & active_snapshot[383:320]) |
			({64{reg_query_page[6]}} & active_snapshot[319:256]) |
			({64{reg_query_page[7]}} & active_snapshot[511:448]) |
			({64{reg_query_page[8]}} & active_snapshot[447:384]) |
			({64{reg_query_page[9]}} & reg_telemetry_header) |
			({64{reg_query_page[10]}} & reg_telemetry_status) |
			({64{reg_query_page[11]}} & reg_telemetry_config[127:64]) |
			({64{reg_query_page[12]}} & reg_telemetry_config[63:0]) |
			({64{reg_query_page[13]}} & reg_telemetry_config[255:192]) |
			({64{reg_query_page[14]}} & reg_telemetry_config[191:128]) |
			({64{reg_query_page[15]}} & reg_telemetry_config[383:320]) |
			({64{reg_query_page[16]}} & reg_telemetry_config[319:256]) |
			({64{reg_query_page[17]}} & reg_telemetry_config[511:448]) |
			({64{reg_query_page[18]}} & reg_telemetry_config[447:384]) |
			({64{reg_query_page[19]}} & reg_telemetry_weather_actual[63:0]) |
			({64{reg_query_page[20]}} & reg_telemetry_weather_actual[127:64]) |
			({64{reg_query_page[21]}} & {32'd0,reg_telemetry_weather_actual[159:128]});
		dec_ack = {16'h4953,8'd1,(dec_opcode | 8'h80),dec_sequence,
			8'd3,dec_error,active_snapshot[519:512],dec_page,dec_page_data};
		if(flag_opcode_query && (|reg_query_page[21:9]))begin
			dec_ack[79:72] = reg_telemetry_config[519:512];
		end
		if(dec_error == 0)begin
			dec_ack[95:88] = flag_opcode_query ? 8'd4 : 8'd1;
		end
		if(flag_eth_applied)begin
			dec_ack = {16'h4953,8'd1,(reg_opcode | 8'h80),reg_sequence,
				8'd2,8'd0,active_snapshot[519:512],8'd0,snapshot_epoch,
				active_snapshot[127:120],active_snapshot[255:248],
				active_snapshot[383:376],active_snapshot[511:504],16'd0};
		end
	end
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_telemetry_config <= {8'h1e,{4{DEFAULT_TILE}}};
			reg_telemetry_status <= 64'd0;
			reg_telemetry_header <= 64'h2501000000000004;
			reg_telemetry_weather_actual <= 160'd0;
		end else if(flag_freeze_telemetry)begin
			reg_telemetry_config <= live_config;
			reg_telemetry_status <= live_status;
			reg_telemetry_header <= {16'h2501,live_epoch,live_id,13'd0,1'b1,live_valid,live_enable};
			reg_telemetry_weather_actual <= live_weather_actual;
		end
	end
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			update_o <= 0;
			reg_submit <= {8'h1e,{4{DEFAULT_TILE}}};
			reg_sequence <= 0;
			reg_opcode <= 0;
			reg_authoritative <= {8'h1e,{4{DEFAULT_TILE}}};
			reg_eth_pending_config <= {8'h1e,{4{DEFAULT_TILE}}};
			flag_eth_pending <= 1'b0;
			reg_inflight_source <= 2'd0;
			adaptive_enable_o <= 1'b0;
		end else begin
			update_o <= 1'b0;
			if(flag_evaluate_now && flag_opcode_adaptive && dec_error == 0)
				adaptive_enable_o <= reg_bytes[6][0];
			if(flag_submit)begin
				reg_eth_pending_config <= dec_config;
				reg_authoritative <= dec_config;
				flag_eth_pending <= 1'b1;
				reg_sequence <= dec_sequence;
				reg_opcode <= dec_opcode;
			end else if(update_o && reg_inflight_source == 2'd2)begin
				// 外部握手时没有接收/评价中的UDP，下一拍才可能出现新start/end。
				// 因而先复制已合法提交的载荷，最早再后一拍的UDP评价仍读取新底图。
				reg_authoritative <= reg_submit;
			end
			if(flag_applied)begin
				reg_inflight_source <= 2'd0;
			end
			if(flag_launch_eth)begin
				reg_submit <= reg_eth_pending_config;
				flag_eth_pending <= 1'b0;
				reg_inflight_source <= 2'd1;
				update_o <= 1'b1;
			end else if(flag_ext_accept && !i_ext_query)begin
				// ready已保证旧像素事务排空。非法载荷只预装，不产生update或epoch；
				// 合法性仅驱动事务控制，避免串入520bit载荷使能及共享底图使能。
				reg_submit <= i_ext_config;
				if(flag_ext_config_valid)begin
					reg_inflight_source <= 2'd2;
					update_o <= 1'b1;
				end
			end
		end
	end
	// Check the same field limits as the IS UDP tile command. Validation uses
	// only the source-domain request; accepted payload is held until CDC ACK.
	always@(*)begin
		flag_ext_config_valid = i_ext_config[519:517] == 3'd0;
		for(ext_tile_index = 0; ext_tile_index < 4; ext_tile_index = ext_tile_index + 1)begin
			if(i_ext_config[ext_tile_index * 128 + 126 +: 2] != 2'd0 ||
				i_ext_config[ext_tile_index * 128 + 64 +: 2] > 2'd1 ||
				i_ext_config[ext_tile_index * 128 + 61 +: 3] > 3'd4 ||
				i_ext_config[ext_tile_index * 128 + 42 +: 3] > 3'd4 ||
				i_ext_config[ext_tile_index * 128 + 23 +: 3] > 3'd3 ||
				i_ext_config[ext_tile_index * 128 + 20 +: 3] > 3'd4 ||
				i_ext_config[ext_tile_index * 128 +: 20] != 20'd0)begin
				flag_ext_config_valid = 1'b0;
			end
		end
	end
	// External completion is independent of all UDP ACK storage and credit.
	// Query returns the latest complete applied snapshot, never a partial image.
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_applied_source <= 2'd0;
			ext_done_o <= 1'b0;
			ext_status_o <= 8'd0;
			ext_snapshot_o <= {8'h1e,{4{DEFAULT_TILE}}};
			ext_epoch_o <= 16'd0;
			ext_source_o <= 2'd0;
		end else begin
			ext_done_o <= 1'b0;
			if(flag_applied)begin
				reg_applied_source <= reg_inflight_source;
				if(reg_inflight_source == 2'd2)begin
					ext_done_o <= 1'b1;
					ext_status_o <= 8'd0;
				end
			end
			if(flag_ext_accept && (i_ext_query || !flag_ext_config_valid))begin
				ext_done_o <= 1'b1;
				ext_status_o <= i_ext_query ? 8'd0 : 8'd2;
			end
			// 宽响应在每次受理时预装，避免字段合法性驱动 538 位 CE。
			// 合法 SET 的 DONE=0 时可预装旧快照；最终 applied 时覆盖新快照。
			// PCIe mailbox 仅在 DONE 捕获，响应拍数与 status 保持原契约。
			if(flag_ext_accept || (flag_applied && reg_inflight_source == 2'd2))begin
				ext_snapshot_o <= active_snapshot;
				ext_epoch_o <= snapshot_epoch;
				ext_source_o <= flag_ext_accept ? reg_applied_source : 2'd2;
			end
		end
	end
	genvar ack_slot;
	generate
		for(ack_slot = 0; ack_slot < 8; ack_slot = ack_slot + 1)begin:gen_ack_ff
			always@(posedge i_control_clk or negedge i_control_rstn)begin
				if(!i_control_rstn)begin
					reg_queue[ack_slot] <= 0;
				end else if(flag_push && reg_write == ack_slot)begin
					reg_queue[ack_slot] <= dec_ack;
				end
			end
		end
	endgenerate
	always@(posedge i_control_clk or negedge i_control_rstn)begin
		if(!i_control_rstn)begin
			reg_write <= 0;
			reg_read <= 0;
			cnt_queue <= 0;
			dropped_status_count_o <= 0;
		end else begin
			if(flag_push)begin reg_write <= reg_write + 3'd1; end
			if(flag_pop)begin reg_read <= reg_read + 3'd1; end
			case({flag_push,flag_pop})
				2'b10:begin cnt_queue <= cnt_queue + 4'd1; end
				2'b01:begin cnt_queue <= cnt_queue - 4'd1; end
				default:begin end
			endcase
			if((flag_eth_applied && !flag_space) || (flag_evaluate_now && !flag_event_space))begin
				dropped_status_count_o <= dropped_status_count_o + 32'd1;
			end
		end
	end
	isp_config_cdc isp_config_cdc_Inst_config(
		.i_src_clk(i_control_clk),
		.i_src_rst_n(i_control_rstn),
		.i_dst_clk(i_pixel_clk),
		.i_dst_rst_n(i_pixel_rstn),
		.i_update(update_o),
		.i_config(reg_submit),
		.i_frame_boundary(i_frame_boundary),
		.i_adaptive_enable(adaptive_enable_o),
		.o_adaptive_pixel_enable(o_adaptive_pixel_enable),
		.i_adaptive_rgb(i_adaptive_rgb),
		.i_adaptive_tag(i_adaptive_tag),
		.i_scene_rgb(i_scene_rgb),
		.i_scene_tag(i_scene_tag),
		.i_scene_frame_boundary(i_scene_frame_boundary),
		.i_scene_epoch(i_scene_epoch),
		.i_weather_actual(i_weather_actual),
		.o_summary_valid(o_summary_valid),
		.o_summary_complete(o_summary_complete),
		.o_summary_tile(o_summary_tile),
		.o_summary_frame(o_summary_frame),
		.o_summary_epoch(o_summary_epoch),
		.o_summary_p10(o_summary_p10),
		.o_summary_p50(o_summary_p50),
		.o_summary_p90(o_summary_p90),
		.o_summary_low48(o_summary_low48),
		.o_summary_fog_count(o_summary_fog_count),
		.o_summary_texture_count(o_summary_texture_count),
		.o_summary_neutral_count(o_summary_neutral_count),
		.o_busy(flag_cdc_busy),
		.o_applied(flag_applied),
		.o_active_config(o_active_config),
		.o_config_epoch(o_config_epoch),
		.o_effective_epoch(o_effective_epoch),
		.o_isp_continuous_update(o_isp_continuous_update),
		.o_change_mask(o_change_mask),
		.o_src_active_snapshot(active_snapshot),
		.o_src_config_epoch(snapshot_epoch),
		.o_src_live_config(live_config),
		.o_src_live_epoch(live_epoch),
		.o_src_live_status(live_status),
		.o_src_live_id(live_id),
		.o_src_live_enable(live_enable),
		.o_src_live_valid(live_valid),
		.o_src_live_weather_actual(live_weather_actual),
		.o_pending(o_pending)
	);
endmodule
