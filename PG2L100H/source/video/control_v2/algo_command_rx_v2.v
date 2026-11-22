//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 解析 HDMI 算法 v1/v2 控制字节流并校验包格式、CRC、忙状态及队列信用，仅在合法完整包被接受后更新 shadow。
// Module Name: algo_command_rx_v2
// File Name: source/video/control_v2/algo_command_rx_v2.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 解析 HDMI 算法 v1/v2 控制字节流并校验包格式、CRC、忙状态及队列信用，仅在合法完整包被接受后更新 shadow。
// 将 stage、tile、W、t_min、A、flags 和 16 位 threshold 合成为 64 位配置，局部命令基于共享配置合并。
//
// Dependencies: UDP 分流、共享配置基线、下游 CDC 及状态队列信用。
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
// Design Name: HDMI algorithm control v1/v2
// Module Name: algo_command_rx_v2
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 包起始可伴首字节，包结束必须在最后字节之后一拍。
//              配置64位依次 stage,tile,W,t_min,A,flags,threshold(16)。
//              shadow仅在整包合法且空闲时改变；CRC错误优先于busy。
// Dependencies: UDP分流、协调复位、下游CDC及队列信用
// Revision: 0.02 - 2026-10-23，局部命令合并到共享配置基线
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module algo_command_rx_v2
(
	input i_clk,
	input i_rstn,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	input i_busy,
	input [63:0] i_base_config,
	output o_receiving,
	output o_accepted,
	output o_rejected,
	output [7:0] o_error,
	output [7:0] o_version,
	output [7:0] o_opcode,
	output [15:0] o_sequence,
	output [63:0] o_shadow
);
	localparam [63:0] DEFAULT_CONFIG = 64'h000fe619dc010080;
	reg [7:0] reg_bytes [0:15];
	reg [15:0] cnt_bytes;
	reg [15:0] reg_crc;
	reg [15:0] dec_crc;
	reg flag_receiving;
	reg flag_error;
	reg accepted_o;
	reg rejected_o;
	reg [7:0] error_o;
	reg [7:0] version_o;
	reg [7:0] opcode_o;
	reg [15:0] sequence_o;
	reg [63:0] shadow_o;
	reg [63:0] dec_config;
	reg [7:0] dec_error;
	reg [15:0] dec_sequence;
	reg [15:0] dec_threshold;
	reg [15:0] dec_length;
	integer cnt_bit;
	assign o_accepted = accepted_o;
	assign o_rejected = rejected_o;
	assign o_error = error_o;
	assign o_version = version_o;
	assign o_opcode = opcode_o;
	assign o_sequence = sequence_o;
	assign o_shadow = shadow_o;
	assign o_receiving = flag_receiving;
	// 所有包字节包括末尾CRC进入计算；CCITT-FALSE合法报文余数为零。
	always@(*)begin
		dec_crc = (i_start ? 16'hffff : reg_crc) ^ {i_data, 8'd0};
		for(cnt_bit = 0; cnt_bit < 8; cnt_bit = cnt_bit + 1)begin
			if(dec_crc[15])begin
				dec_crc = (dec_crc << 1) ^ 16'h1021;
			end else begin
				dec_crc = dec_crc << 1;
			end
		end
	end
	// 顺序与旧协议一致，错误包不覆盖已接受配置。
	always@(*)begin
		// 截断头部不能读取尚未接收到的字节；缓存无需异步清零。
		dec_length = 16'd16;
		if(cnt_bytes >= 16'd3 && reg_bytes[2] == 8'd1)begin
			dec_length = 16'd11;
		end else if(cnt_bytes >= 16'd4 && reg_bytes[3] == 8'd2)begin
			dec_length = 16'd10;
		end
		dec_sequence = 16'd0;
		dec_threshold = 16'd128;
		// Enable-only commands merge against the shared accepted baseline.
		// This includes a frozen PCIe update awaiting its frame boundary.
		dec_config = i_base_config;
		if(reg_bytes[2] == 8'd1)begin
			dec_sequence = {reg_bytes[5],reg_bytes[6]};
			dec_threshold = {reg_bytes[7],reg_bytes[8]};
			dec_config = DEFAULT_CONFIG;
			case(reg_bytes[4])
				8'd1:begin dec_config[63:56] = 8'h04; end
				8'd2:begin dec_config[63:56] = 8'h02; end
				8'd3:begin dec_config[63:56] = 8'h08; end
				default:begin dec_config[63:56] = 8'h00; end
			endcase
			dec_config[15:0] = dec_threshold == 16'd0 ? 16'd128 : dec_threshold;
		end else if(reg_bytes[3] == 8'd2)begin
			dec_sequence = {reg_bytes[6],reg_bytes[7]};
			dec_config[56] = reg_bytes[4][0];
		end else begin
			dec_sequence = {reg_bytes[10],reg_bytes[11]};
			dec_threshold = {reg_bytes[12],reg_bytes[13]};
			dec_config = {reg_bytes[4],reg_bytes[5],reg_bytes[6],reg_bytes[7],reg_bytes[8],reg_bytes[9],dec_threshold == 16'd0 ? 16'd128 : dec_threshold};
		end
		dec_error = 8'd0;
		if(i_error || flag_error)begin dec_error = 8'd8; end
		else if(!flag_receiving || cnt_bytes < 16'd4 || cnt_bytes != dec_length || i_length != dec_length)begin dec_error = 8'd1; end
		else if({reg_bytes[0],reg_bytes[1]} != 16'ha55a)begin dec_error = 8'd2; end
		else if(reg_bytes[2] != 8'd1 && reg_bytes[2] != 8'd2)begin dec_error = 8'd3; end
		else if(reg_bytes[3] != 8'd1 && !(reg_bytes[2] == 8'd2 && reg_bytes[3] == 8'd2))begin dec_error = 8'd4; end
		else if(reg_bytes[2] == 8'd1 && reg_bytes[4] > 8'd3)begin dec_error = 8'd5; end
		else if(reg_crc != 16'd0)begin dec_error = 8'd6; end
		else if((reg_bytes[2] == 8'd1 || reg_bytes[3] == 8'd1) && dec_threshold > 16'd1443)begin dec_error = 8'd9; end
		else if(reg_bytes[2] == 8'd2 && reg_bytes[3] == 8'd1 && reg_bytes[4][7:4] != 4'd0)begin dec_error = 8'd10; end
		else if(reg_bytes[2] == 8'd2 && reg_bytes[3] == 8'd1 && reg_bytes[5][7:4] != 4'd0)begin dec_error = 8'd11; end
		else if(reg_bytes[2] == 8'd2 && ((reg_bytes[3] == 8'd2 && reg_bytes[4] > 8'd1) || (reg_bytes[3] == 8'd1 && (reg_bytes[7] == 8'd0 || reg_bytes[8] == 8'd0 || reg_bytes[8] > 8'd250))))begin dec_error = 8'd12; end
		else if(reg_bytes[2] == 8'd2 && ((reg_bytes[3] == 8'd2 && reg_bytes[5] != 8'd0) || (reg_bytes[3] == 8'd1 && reg_bytes[9][7:1] != 7'd0)))begin dec_error = 8'd13; end
		else if(i_busy)begin dec_error = 8'd7; end
	end
	// 浅寄存器报文缓存不属于RAM替身，不依赖存储IP。
	always@(posedge i_clk)begin
		if(i_rstn && i_valid)begin
			if(i_start)begin reg_bytes[0] <= i_data; end
			else if(flag_receiving && cnt_bytes < 16'd16)begin reg_bytes[cnt_bytes[3:0]] <= i_data; end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			cnt_bytes <= 16'd0;
			reg_crc <= 16'hffff;
			flag_receiving <= 1'b0;
			flag_error <= 1'b0;
		end else if(i_start)begin
			cnt_bytes <= i_valid ? 16'd1 : 16'd0;
			reg_crc <= i_valid ? dec_crc : 16'hffff;
			flag_receiving <= 1'b1;
			flag_error <= i_error;
		end else begin
			if(i_end)begin flag_receiving <= 1'b0; end
			if(flag_receiving && i_valid)begin
				if(cnt_bytes != 16'hffff)begin cnt_bytes <= cnt_bytes + 16'd1; end
				reg_crc <= dec_crc;
			end
			if(flag_receiving && i_error)begin flag_error <= 1'b1; end
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			accepted_o <= 1'b0;
			rejected_o <= 1'b0;
			error_o <= 8'd0;
			version_o <= 8'd1;
			opcode_o <= 8'd1;
			sequence_o <= 16'd0;
			shadow_o <= DEFAULT_CONFIG;
		end else begin
			accepted_o <= 1'b0;
			rejected_o <= 1'b0;
			if(i_end)begin
				accepted_o <= dec_error == 8'd0;
				rejected_o <= dec_error != 8'd0;
				error_o <= dec_error;
				version_o <= cnt_bytes >= 16'd3 && reg_bytes[2] == 8'd2 ? 8'd2 : 8'd1;
				opcode_o <= cnt_bytes >= 16'd4 ? reg_bytes[3] : 8'd1;
				sequence_o <= cnt_bytes >= dec_length - 16'd2 ? dec_sequence : 16'd0;
				if(dec_error == 8'd0)begin shadow_o <= dec_config; end
			end
		end
	end
endmodule
