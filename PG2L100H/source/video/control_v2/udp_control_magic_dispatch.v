//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 按控制包 magic 分发逐字节命令，以 32 字节×4 项短包 FIFO 隔离网络接收与下游重放。
// Module Name: udp_control_magic_dispatch
// File Name: source/video/control_v2/udp_control_magic_dispatch.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 按控制包 magic 分发逐字节命令，以 32 字节×4 项短包 FIFO 隔离网络接收与下游重放。
// 允许连续控制包先排队再按原字节时序重放，支持可选 IS 分支；满队列不得覆盖正在重放的缓存。
//
// Dependencies: 上游完整 UDP 字节流；各 magic 对应控制接收模块。
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
// Create Date: 2026-10-13
// Design Name: UDP control packet dispatcher
// Module Name: udp_control_magic_dispatch
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 控制包按magic分流，并以四项短包FIFO隔离接收和重放。
//              原单缓存版本在重放期间静默丢弃下一帧；FIFO允许以太网背靠背
//              控制包先排队，再保持原有逐字节输出时序及下游单请求语义。
// Dependencies: 无；控制包缓存为32字节×4项寄存器阵列，不引入CRC/哈希。
// Revision: 0.02 - 修复快速连续控制包在flag_emit期间被丢弃的问题。
//           0.03 - 可选IS分支；满队列包首字节禁止覆盖重放缓存。
//           0.04 - 2026-10-21 重放出口寄存，缓存读与接收端解析分拍。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module udp_control_magic_dispatch
#(
	parameter integer C_LUT_ENABLE = 0,
	parameter integer C_ISP_ENABLE = 0
)
(
	input i_clk,
	input i_rstn,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	output o_algo_start,
	output o_algo_valid,
	output [7:0] o_algo_data,
	output o_algo_end,
	output [15:0] o_algo_length,
	output o_algo_error,
	output o_route_start,
	output o_route_valid,
	output [7:0] o_route_data,
	output o_route_end,
	output [15:0] o_route_length,
	output o_route_error,
	output o_lut_start,
	output o_lut_valid,
	output [7:0] o_lut_data,
	output o_lut_end,
	output [15:0] o_lut_length,
	output o_lut_error,
	output o_behavior_start,
	output o_behavior_valid,
	output [7:0] o_behavior_data,
	output o_behavior_end,
	output [15:0] o_behavior_length,
	output o_behavior_error,
	output o_isp_start,
	output o_isp_valid,
	output [7:0] o_isp_data,
	output o_isp_end,
	output [15:0] o_isp_length,
	output o_isp_error
);
	localparam integer C_QUEUE_DEPTH = 4;
	reg [7:0] reg_bytes [0:3][0:31];
	reg [15:0] reg_packet_length [0:3];
	reg reg_packet_error [0:3];
	reg [2:0] reg_packet_target [0:3];
	reg [1:0] reg_write;
	reg [1:0] reg_read;
	reg [2:0] reg_count;
	reg [15:0] cnt_bytes;
	reg [5:0] cnt_emit;
	reg [15:0] reg_length;
	reg flag_error;
	reg flag_receiving;
	reg flag_drop_packet;
	reg flag_emit;
	reg [7:0] emit_data_o;
	reg [15:0] emit_length_o;
	reg [2:0] emit_target_o;
	reg emit_start_o;
	reg emit_valid_o;
	reg emit_end_o;
	reg emit_error_o;
	integer byte_index;
	integer slot_index;
	wire flag_byte;
	wire flag_last;
	wire flag_pop;
	wire [2:0] input_target;
	wire [15:0] input_length_total;
	wire [15:0] input_length_clamped;
	wire input_magic_known;
	wire flag_enqueue;
	assign input_target = (reg_bytes[reg_write][0] == 8'hA5 && reg_bytes[reg_write][1] == 8'h5A) ? 3'd1 :
		(reg_bytes[reg_write][0] == 8'h41 && reg_bytes[reg_write][1] == 8'h56) ? 3'd2 :
		(reg_bytes[reg_write][0] == 8'h4C && reg_bytes[reg_write][1] == 8'h55) ? 3'd3 :
		(reg_bytes[reg_write][0] == 8'h42 && reg_bytes[reg_write][1] == 8'h56) ? 3'd4 :
		(reg_bytes[reg_write][0] == 8'h49 && reg_bytes[reg_write][1] == 8'h53) ? 3'd5 : 3'd0;
	assign input_magic_known = input_target != 3'd0 &&
		(input_target != 3'd3 || C_LUT_ENABLE != 0) &&
		(input_target != 3'd5 || C_ISP_ENABLE != 0);
	assign input_length_total = cnt_bytes + (i_valid ? 16'd1 : 16'd0);
	assign input_length_clamped = input_length_total > 16'd32 ? 16'd32 : input_length_total;
	assign flag_pop = flag_emit && flag_last;
	assign flag_enqueue = i_end && flag_receiving && !flag_drop_packet &&
		input_length_total >= 16'd2 && input_magic_known &&
		(reg_count < C_QUEUE_DEPTH);
	assign flag_byte = flag_emit && cnt_emit < reg_packet_length[reg_read];
	assign flag_last = flag_emit && cnt_emit == reg_packet_length[reg_read];
	assign o_algo_start = emit_start_o && emit_target_o == 1;
	assign o_algo_valid = emit_valid_o && emit_target_o == 1;
	assign o_algo_data = emit_data_o;
	assign o_algo_end = emit_end_o && emit_target_o == 1;
	assign o_algo_length = emit_length_o;
	assign o_algo_error = emit_error_o && emit_target_o == 1;
	assign o_route_start = emit_start_o && emit_target_o == 2;
	assign o_route_valid = emit_valid_o && emit_target_o == 2;
	assign o_route_data = emit_data_o;
	assign o_route_end = emit_end_o && emit_target_o == 2;
	assign o_route_length = emit_length_o;
	assign o_route_error = emit_error_o && emit_target_o == 2;
	assign o_lut_start = emit_start_o && emit_target_o == 3;
	assign o_lut_valid = emit_valid_o && emit_target_o == 3;
	assign o_lut_data = emit_data_o;
	assign o_lut_end = emit_end_o && emit_target_o == 3;
	assign o_lut_length = emit_length_o;
	assign o_lut_error = emit_error_o && emit_target_o == 3;
	assign o_behavior_start = emit_start_o && emit_target_o == 4;
	assign o_behavior_valid = emit_valid_o && emit_target_o == 4;
	assign o_behavior_data = emit_data_o;
	assign o_behavior_end = emit_end_o && emit_target_o == 4;
	assign o_behavior_length = emit_length_o;
	assign o_behavior_error = emit_error_o && emit_target_o == 4;
	assign o_isp_start = emit_start_o && emit_target_o == 5;
	assign o_isp_valid = emit_valid_o && emit_target_o == 5;
	assign o_isp_data = emit_data_o;
	assign o_isp_end = emit_end_o && emit_target_o == 5;
	assign o_isp_length = emit_length_o;
	assign o_isp_error = emit_error_o && emit_target_o == 5;

	// 数据与包控制同拍寄存；队列释放后不再依赖原缓存槽的内容。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			emit_data_o <= 8'd0;
			emit_length_o <= 16'd0;
			emit_target_o <= 3'd0;
			emit_start_o <= 1'b0;
			emit_valid_o <= 1'b0;
			emit_end_o <= 1'b0;
			emit_error_o <= 1'b0;
		end else begin
			emit_data_o <= reg_bytes[reg_read][cnt_emit[4:0]];
			emit_length_o <= reg_packet_length[reg_read];
			emit_target_o <= reg_packet_target[reg_read];
			emit_start_o <= flag_byte && cnt_emit == 0;
			emit_valid_o <= flag_byte;
			emit_end_o <= flag_last;
			emit_error_o <= flag_last && reg_packet_error[reg_read];
		end
	end

	// 四项固定短包缓存。接收和重放可以并行，写指针只在完整包提交时前进。
	always @(posedge i_clk or negedge i_rstn) begin
		if(!i_rstn) begin
			for(slot_index = 0; slot_index < C_QUEUE_DEPTH; slot_index = slot_index + 1) begin
				reg_packet_length[slot_index] <= 16'd0;
				reg_packet_error[slot_index] <= 1'b0;
				reg_packet_target[slot_index] <= 2'd0;
				for(byte_index = 0; byte_index < 32; byte_index = byte_index + 1) begin
					reg_bytes[slot_index][byte_index] <= 8'd0;
				end
			end
		end else begin
			if(i_valid && flag_receiving && !flag_drop_packet && cnt_bytes < 16'd32)
				reg_bytes[reg_write][cnt_bytes[4:0]] <= i_data;
			// 满队列时flag_drop_packet尚未更新，首字节必须直接检查当前容量。
			if(i_start && i_valid && !flag_receiving && !flag_drop_packet &&
				reg_count < C_QUEUE_DEPTH)
				reg_bytes[reg_write][0] <= i_data;
			if(flag_enqueue) begin
				reg_packet_length[reg_write] <= input_length_clamped;
				reg_packet_error[reg_write] <= flag_error || i_error || input_length_total > 16'd32;
				reg_packet_target[reg_write] <= input_target;
			end
		end
	end

	// 输入包完整缓存后入队；队列满时只丢弃当前完整包，后续包仍可恢复。
	always @(posedge i_clk or negedge i_rstn) begin
		if(!i_rstn) begin
			reg_write <= 2'd0;
			reg_read <= 2'd0;
			reg_count <= 3'd0;
			cnt_bytes <= 16'd0;
			reg_length <= 16'd0;
			flag_error <= 1'b0;
			flag_receiving <= 1'b0;
			flag_drop_packet <= 1'b0;
			flag_emit <= 1'b0;
			cnt_emit <= 6'd0;
		end else begin
			if(i_start) begin
				cnt_bytes <= i_valid ? 16'd1 : 16'd0;
				reg_length <= i_length;
				flag_receiving <= 1'b1;
				flag_error <= i_error;
				// 已满时保留输入时序但禁止写入，避免覆盖尚未重放的包。
				flag_drop_packet <= reg_count >= C_QUEUE_DEPTH;
			end else if(flag_receiving) begin
				if(i_valid && cnt_bytes != 16'hffff)
					cnt_bytes <= cnt_bytes + 1'b1;
				if(i_error) flag_error <= 1'b1;
			end
			if(i_end && flag_receiving) begin
				flag_receiving <= 1'b0;
				reg_length <= i_length;
				flag_error <= flag_error || i_error || input_length_total > 16'd32;
				flag_drop_packet <= 1'b0;
				if(flag_enqueue) reg_write <= reg_write + 1'b1;
			end
			if(!flag_emit && reg_count != 0) begin
				flag_emit <= 1'b1;
				cnt_emit <= 6'd0;
			end else if(flag_emit) begin
				if(flag_last) begin
					flag_emit <= 1'b0;
					cnt_emit <= 6'd0;
					reg_read <= reg_read + 1'b1;
				end else begin
					cnt_emit <= cnt_emit + 1'b1;
				end
			end
			case({flag_enqueue,flag_pop})
				2'b10: reg_count <= reg_count + 1'b1;
				2'b01: reg_count <= reg_count - 1'b1;
				default: reg_count <= reg_count;
			endcase
		end
	end
endmodule
