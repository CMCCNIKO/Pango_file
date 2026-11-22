//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 PGV1 RGB565 小端字节流按 32 字节组合为 256 位 DDR 用户字，word[15:0] 对应首像素。
// Module Name: rgb565_byte_packer
// File Name: source/video/rgb565_byte_packer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 PGV1 RGB565 小端字节流按 32 字节组合为 256 位 DDR 用户字，word[15:0] 对应首像素。
// 仅在整字完成时检查 ready；由于上游 UDP 无反压，未就绪时报告 overflow，供接收器将整帧判错。
//
// Dependencies: pgv1_rx 的字节流及 c6_net_to_ddr_fifo_wrapper 的写入许可。
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
//
// Create Date: 2026-09-10
// Design Name: Ethernet_HDMI
// Module Name: rgb565_byte_packer
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 将 PGV1 的 RGB565 小端字节流组合为 DDR 用户接口使用的
//              256-bit 宽字；word[15:0] 始终为首个 RGB565 像素。
//
// Dependencies: pgv1_rx、C6 Pango DRM Based FIFO（待 PDS 生成）
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// i_word_ready 只在输出完整 32 字节时采样。上游 UDP 无 ready 反压，因此
// 未就绪时报告溢出并由 pgv1_rx 作废本帧，而不是丢失后伪装为完整帧。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module rgb565_byte_packer(
	input i_clk,
	input i_rst_n,
	input i_frame_prepare,
	input i_frame_abort,
	input i_payload_valid,
	input [7:0] i_payload_data,
	input i_word_ready,
	output reg o_word_valid,
	output reg [255:0] o_word_data,
	output reg o_overflow
);
	reg [255:0] reg_pack_data;
	reg [4:0] cnt_pack_bytes;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_pack_data <= 256'd0;
			cnt_pack_bytes <= 5'd0;
			o_word_valid <= 1'b0;
			o_word_data <= 256'd0;
			o_overflow <= 1'b0;
		end else begin
			o_word_valid <= 1'b0;
			o_overflow <= 1'b0;
			if (i_frame_prepare || i_frame_abort) begin
				reg_pack_data <= 256'd0;
				cnt_pack_bytes <= 5'd0;
			end else if (i_payload_valid) begin
				if (cnt_pack_bytes == 5'd31) begin
					if (i_word_ready) begin
						o_word_valid <= 1'b1;
						o_word_data <= {i_payload_data, reg_pack_data[247:0]};
						cnt_pack_bytes <= 5'd0;
					end else begin
						o_overflow <= 1'b1;
						cnt_pack_bytes <= 5'd0;
					end
				end else begin
					reg_pack_data[cnt_pack_bytes * 8 +: 8] <= i_payload_data;
					cnt_pack_bytes <= cnt_pack_bytes + 1'b1;
				end
			end
		end
	end
endmodule
