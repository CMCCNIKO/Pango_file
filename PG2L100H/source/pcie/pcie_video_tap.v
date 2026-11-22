//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-06
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 从已寄存的最终 HDMI 栅格采样 RGB888 并转换为 RGB565，每 8 像素组成一个 128 位 FIFO 字，同时累计帧 CRC32。
// Module Name: pcie_video_tap
// File Name: source/pcie/pcie_video_tap.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 从已寄存的最终 HDMI 栅格采样 RGB888 并转换为 RGB565，每 8 像素组成一个 128 位 FIFO 字，同时累计帧 CRC32。
// 仅消费完整帧授权，不能反压源；校验宽高及 DE 行结构，FIFO 满或短帧产生 drop/error，错误尾部不补字。
//
// Dependencies: 无子模块；下游异步 FIFO 的写入许可及 frame_ctrl 单帧授权。
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
// Create Date: 2026-10-06
// Design Name: pcie_video_tap
// Module Name: pcie_video_tap
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 最终 HDMI 已寄存像素到 RGB565/128-bit FIFO；按完整帧授权，不反压。
// Revision: 0.03 - Explicit accept, raster validation and reset/flush boundary guard.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_video_tap #(
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_V_ACTIVE = 720,
	parameter integer C_FRAME_PIXELS = C_H_ACTIVE*C_V_ACTIVE,
	parameter integer C_WORD_COUNT = C_FRAME_PIXELS/8
)(
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_hdmi_vs,
	input i_hdmi_hs,
	input i_hdmi_de,
	input [23:0] i_hdmi_rgb,
	input i_capture_enable,
	input i_flush,
	input i_fifo_full,
	input i_fifo_almost_full,
	output o_fifo_wr_en,
	output [127:0] o_fifo_wr_data,
	output o_tap_frame_start,
	output o_tap_frame_accept,
	output o_tap_frame_end,
	output o_tap_frame_drop,
	output o_tap_frame_error,
	output o_tap_frame_active,
	output [31:0] o_tap_frame_id,
	output [31:0] o_tap_byte_count,
	output [17:0] o_tap_word_count,
	output [31:0] o_tap_crc32,
	output o_tap_meta_toggle,
	output o_tap_boundary_toggle,
	output o_flush_ack_toggle
);
	// 输入/输出全部属于 pixel_clk；VS 高为帧间同步，DE 的每个连续高段为一行。
	// 首像素锁存授权并发出 accept，即使该像素遇满也消耗授权并报告错误帧。
	// 参数须 H*V == FRAME_PIXELS == WORD_COUNT*8，且 0 < FRAME_PIXELS < 2^21-1。
	// 正常帧按低像素先、每像素低字节先排列；错误短尾不填充、不额外提交 word。
	localparam [31:0] C_CRC_INIT = 32'hffffffff;
	localparam [31:0] C_CRC_XOROUT = 32'hffffffff;
	localparam C_CONFIG_BAD = (C_H_ACTIVE <= 0) || (C_V_ACTIVE <= 0) ||
		(C_FRAME_PIXELS != C_H_ACTIVE*C_V_ACTIVE) ||
		(C_FRAME_PIXELS != C_WORD_COUNT*8) || (C_FRAME_PIXELS >= 2097151);
	reg prev_vs;
	reg prev_de;
	reg prev_flush;
	reg frame_wait_de;
	reg wait_for_vs;
	reg wait_seen_vs_high;
	reg frame_started;
	reg frame_capture;
	reg frame_drop;
	reg frame_error;
	reg [31:0] frame_sequence;
	reg [31:0] frame_id_active;
	reg [20:0] pixel_count;
	reg [20:0] line_pixel_count;
	reg [20:0] line_count;
	reg [17:0] word_count;
	reg [2:0] pack_index;
	reg [127:0] pack_reg;
	reg [31:0] crc_reg;
	reg [127:0] fifo_wr_data_o;
	reg fifo_wr_en_o;
	reg tap_frame_start_o;
	reg tap_frame_accept_o;
	reg tap_frame_end_o;
	reg tap_frame_drop_o;
	reg tap_frame_error_o;
	reg [31:0] tap_frame_id_o;
	reg [31:0] tap_byte_count_o;
	reg [17:0] tap_word_count_o;
	reg [31:0] tap_crc32_o;
	reg tap_meta_toggle_o;
	reg tap_boundary_toggle_o;
	reg flush_ack_toggle_o;
	wire [15:0] pixel_rgb565;
	wire fifo_blocked;
	wire vs_fall;
	wire vs_rise;
	wire frame_start_now;
	wire write_accept;
	wire [17:0] words_after_commit;
	wire pixel_out_of_range;
	wire frame_size_bad;
	wire [31:0] crc_after_pixel;

	assign pixel_rgb565 = {i_hdmi_rgb[23:19], i_hdmi_rgb[15:10], i_hdmi_rgb[7:3]};
	assign fifo_blocked = i_fifo_full || i_fifo_almost_full;
	assign vs_fall = prev_vs && !i_hdmi_vs;
	assign vs_rise = !prev_vs && i_hdmi_vs;
	assign frame_start_now = !i_hdmi_vs && i_hdmi_de && !frame_started &&
		((!wait_for_vs && (frame_wait_de || vs_fall)) ||
		(wait_for_vs && wait_seen_vs_high && vs_fall));
	assign write_accept = fifo_wr_en_o && !i_fifo_full && !i_flush;
	assign words_after_commit = word_count + {{17{1'b0}}, write_accept};
	assign pixel_out_of_range = (pixel_count >= C_FRAME_PIXELS) ||
		(prev_de && line_pixel_count >= C_H_ACTIVE) ||
		(!prev_de && line_count >= C_V_ACTIVE);
	assign frame_size_bad = C_CONFIG_BAD || (pixel_count != C_FRAME_PIXELS) ||
		(line_count != C_V_ACTIVE) || (line_pixel_count != C_H_ACTIVE) || i_hdmi_de ||
		(frame_capture && ((pack_index != 3'd0) || (words_after_commit != C_WORD_COUNT)));

	function [31:0] crc32_byte;
		input [31:0] crc_in;
		input [7:0] data_in;
		integer bit_index;
		reg [31:0] crc_work;
		begin
			crc_work = crc_in ^ {24'd0, data_in};
			for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
				if (crc_work[0]) crc_work = (crc_work >> 1) ^ 32'hedb88320;
				else crc_work = crc_work >> 1;
			end
			crc32_byte = crc_work;
		end
	endfunction

	assign crc_after_pixel = crc32_byte(crc32_byte(crc_reg,
		pixel_rgb565[7:0]), pixel_rgb565[15:8]);
	assign o_fifo_wr_en = write_accept;
	assign o_fifo_wr_data = fifo_wr_data_o;
	assign o_tap_frame_start = tap_frame_start_o;
	assign o_tap_frame_accept = tap_frame_accept_o;
	assign o_tap_frame_end = tap_frame_end_o;
	assign o_tap_frame_drop = tap_frame_drop_o;
	assign o_tap_frame_error = tap_frame_error_o;
	assign o_tap_frame_active = frame_started;
	assign o_tap_frame_id = tap_frame_id_o;
	assign o_tap_byte_count = tap_byte_count_o;
	assign o_tap_word_count = tap_word_count_o;
	assign o_tap_crc32 = tap_crc32_o;
	assign o_tap_meta_toggle = tap_meta_toggle_o;
	assign o_tap_boundary_toggle = tap_boundary_toggle_o;
	assign o_flush_ack_toggle = flush_ack_toggle_o;

	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin
			prev_vs <= 1'b1;
			prev_de <= 1'b0;
			prev_flush <= 1'b0;
			frame_wait_de <= 1'b0;
			wait_for_vs <= 1'b1;
			wait_seen_vs_high <= 1'b0;
			frame_started <= 1'b0;
			frame_capture <= 1'b0;
			frame_drop <= 1'b0;
			frame_error <= 1'b0;
			frame_sequence <= 32'd0;
			frame_id_active <= 32'd0;
			pixel_count <= 21'd0;
			line_pixel_count <= 21'd0;
			line_count <= 21'd0;
			word_count <= 18'd0;
			pack_index <= 3'd0;
			pack_reg <= 128'd0;
			crc_reg <= C_CRC_INIT;
			fifo_wr_data_o <= 128'd0;
			fifo_wr_en_o <= 1'b0;
			tap_frame_start_o <= 1'b0;
			tap_frame_accept_o <= 1'b0;
			tap_frame_end_o <= 1'b0;
			tap_frame_drop_o <= 1'b0;
			tap_frame_error_o <= 1'b0;
			tap_frame_id_o <= 32'd0;
			tap_byte_count_o <= 32'd0;
			tap_word_count_o <= 18'd0;
			tap_crc32_o <= 32'd0;
			tap_meta_toggle_o <= 1'b0;
			tap_boundary_toggle_o <= 1'b0;
			flush_ack_toggle_o <= 1'b0;
		end else begin
			prev_vs <= i_hdmi_vs;
			prev_de <= i_hdmi_de;
			prev_flush <= i_flush;
			fifo_wr_en_o <= 1'b0;
			tap_frame_start_o <= 1'b0;
			tap_frame_accept_o <= 1'b0;
			tap_frame_end_o <= 1'b0;
			tap_frame_drop_o <= 1'b0;
			tap_frame_error_o <= 1'b0;
			if (write_accept) word_count <= words_after_commit;

			// 高电平整个期间抑制写出与授权；只在上升沿应答一次。
			if (i_flush) begin
				if (!prev_flush) begin
					flush_ack_toggle_o <= ~flush_ack_toggle_o;
					if (frame_started) begin
						tap_frame_end_o <= 1'b1;
						tap_frame_drop_o <= 1'b1;
						tap_frame_error_o <= 1'b1;
						tap_frame_id_o <= frame_id_active;
						tap_byte_count_o <= {11'd0, pixel_count} << 1;
						tap_word_count_o <= word_count;
						tap_crc32_o <= crc_reg ^ C_CRC_XOROUT;
						tap_meta_toggle_o <= ~tap_meta_toggle_o;
					end
				end
				frame_started <= 1'b0;
				frame_capture <= 1'b0;
				frame_drop <= 1'b0;
				frame_error <= 1'b0;
				frame_wait_de <= 1'b0;
				wait_for_vs <= 1'b1;
				wait_seen_vs_high <= 1'b0;
				pixel_count <= 21'd0;
				line_pixel_count <= 21'd0;
				line_count <= 21'd0;
				word_count <= 18'd0;
				pack_index <= 3'd0;
				pack_reg <= 128'd0;
				crc_reg <= C_CRC_INIT;
			end else begin
				// reset/flush 释放后必须真实观察 VS 高及随后的下降沿。
				if (wait_for_vs) begin
					if (i_hdmi_vs) wait_seen_vs_high <= 1'b1;
					if (wait_seen_vs_high && vs_fall) begin
						wait_for_vs <= 1'b0;
						frame_wait_de <= 1'b1;
					end
				end else if (vs_fall && !frame_started) begin
					frame_wait_de <= 1'b1;
				end
				// 前拍 word 若遭 full 拒收，当前帧立即标坏；计数不包含拒收 word。
				if (fifo_wr_en_o && i_fifo_full && frame_started) begin
					frame_drop <= 1'b1;
					frame_error <= 1'b1;
				end
				if (vs_rise) begin
					tap_boundary_toggle_o <= ~tap_boundary_toggle_o;
					if (frame_started) begin
						tap_frame_end_o <= 1'b1;
						tap_frame_drop_o <= frame_drop || frame_size_bad ||
							(fifo_wr_en_o && i_fifo_full);
						tap_frame_error_o <= frame_error || frame_size_bad ||
							(fifo_wr_en_o && i_fifo_full);
						tap_frame_id_o <= frame_id_active;
						tap_byte_count_o <= {11'd0, pixel_count} << 1;
						tap_word_count_o <= words_after_commit;
						tap_crc32_o <= crc_reg ^ C_CRC_XOROUT;
						tap_meta_toggle_o <= ~tap_meta_toggle_o;
						frame_started <= 1'b0;
						frame_capture <= 1'b0;
						frame_wait_de <= 1'b0;
						pack_index <= 3'd0;
						pack_reg <= 128'd0;
					end
				end else if (frame_start_now) begin
					frame_wait_de <= 1'b0;
					frame_started <= 1'b1;
					frame_capture <= i_capture_enable;
					frame_drop <= !i_capture_enable || fifo_blocked || C_CONFIG_BAD;
					frame_error <= C_CONFIG_BAD || (i_capture_enable && fifo_blocked);
					frame_id_active <= frame_sequence;
					frame_sequence <= frame_sequence + 1'b1;
					pixel_count <= 21'd1;
					line_pixel_count <= 21'd1;
					line_count <= 21'd1;
					word_count <= 18'd0;
					tap_frame_start_o <= 1'b1;
					tap_frame_accept_o <= i_capture_enable;
					if (i_capture_enable && !fifo_blocked && !C_CONFIG_BAD) begin
						pack_reg <= {112'd0, pixel_rgb565};
						pack_index <= 3'd1;
						crc_reg <= crc32_byte(crc32_byte(C_CRC_INIT,
							pixel_rgb565[7:0]), pixel_rgb565[15:8]);
					end else begin
						pack_reg <= 128'd0;
						pack_index <= 3'd0;
						crc_reg <= C_CRC_INIT;
					end
				end else if (frame_started) begin
					if (!i_hdmi_de && prev_de && line_pixel_count != C_H_ACTIVE) begin
						frame_drop <= 1'b1;
						frame_error <= 1'b1;
					end
					if (i_hdmi_de) begin
						if (pixel_count <= C_FRAME_PIXELS) pixel_count <= pixel_count + 1'b1;
						if (!prev_de) begin
							line_pixel_count <= 21'd1;
							if (line_count <= C_V_ACTIVE) line_count <= line_count + 1'b1;
						end else if (line_pixel_count <= C_H_ACTIVE) begin
							line_pixel_count <= line_pixel_count + 1'b1;
						end
						if (pixel_out_of_range) begin
							frame_drop <= 1'b1;
							frame_error <= 1'b1;
						end else if (frame_capture && !frame_drop) begin
							if (fifo_blocked) begin
								frame_drop <= 1'b1;
								frame_error <= 1'b1;
							end else begin
								crc_reg <= crc_after_pixel;
								case (pack_index)
									3'd0: pack_reg[15:0] <= pixel_rgb565;
									3'd1: pack_reg[31:16] <= pixel_rgb565;
									3'd2: pack_reg[47:32] <= pixel_rgb565;
									3'd3: pack_reg[63:48] <= pixel_rgb565;
									3'd4: pack_reg[79:64] <= pixel_rgb565;
									3'd5: pack_reg[95:80] <= pixel_rgb565;
									3'd6: pack_reg[111:96] <= pixel_rgb565;
									3'd7: begin
										fifo_wr_data_o <= {pixel_rgb565, pack_reg[111:0]};
										fifo_wr_en_o <= 1'b1;
										pack_reg <= 128'd0;
									end
									default: pack_reg <= 128'd0;
								endcase
								pack_index <= pack_index + 1'b1;
							end
						end
					end
				end
			end
		end
	end
endmodule
