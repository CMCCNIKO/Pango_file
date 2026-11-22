//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 解析 UDP 导出的 PGV1 单帧视频负载，只接收 640×320 RGB565、1440 字节顺序分片及完整 CRC32 校验的图像。
// Module Name: pgv1_rx
// File Name: source/video/pgv1_rx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 解析 UDP 导出的 PGV1 单帧视频负载，只接收 640×320 RGB565、1440 字节顺序分片及完整 CRC32 校验的图像。
// 每个帧槽 grant 只授权一帧，输出像素字节和完成/错误事件，防止快速来包覆盖 READY 或 DISPLAYING 槽。
//
// Dependencies: rgb565_byte_packer、外部帧槽管理器及网络到 DDR 的官方 FIFO 包装。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 原头“FIFO 待生成”属于旧记录，本工程已携带官方 FIFO 依赖。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Design Name: Ethernet_HDMI
// Module Name: pgv1_rx
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: C6 单帧 PGV1 视频负载解析器。输入为现有 udp_rx 导出的
//              同时起始/首字节、尾字节后一拍结束的字节流；仅接受 640x320
//              RGB565、1440-byte 顺序分片和完整 CRC32 的单帧。
//
// Dependencies: rgb565_byte_packer、独立的网络到 DDR 官方异步 FIFO（待 PDS 生成）
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//  - PGV1 头部、字节序和 CRC 以 host/application/src/protocol.cpp 为准。
//  - C7 使用来自帧槽管理器的单周期 grant；每个 grant 只允许接收一帧，
//    因此不会因网络包过快到达而覆盖 DISPLAYING/READY 槽。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pgv1_rx(
	input i_clk,
	input i_rst_n,
	input i_packet_start,
	input i_data_valid,
	input [7:0] i_data,
	input i_packet_end,
	input i_frame_grant,
	input [1:0] i_frame_grant_slot,
	input i_packer_overflow,
	output reg o_frame_prepare,
	output reg o_payload_valid,
	output reg [7:0] o_payload_data,
	output reg o_frame_commit,
	output reg o_frame_abort,
	output reg o_frame_active,
	output reg [1:0] o_frame_slot,
	output reg [31:0] o_frame_sequence,
	output reg [31:0] o_crc_error_count,
	output reg [31:0] o_sequence_error_count,
	output reg [31:0] o_length_error_count,
	output reg [31:0] o_dropped_frame_count
);
	localparam [31:0] C_MAGIC = 32'h5047_5631;
	localparam [7:0] C_VERSION = 8'd1;
	localparam [7:0] C_VIDEO_TYPE = 8'd1;
	localparam [7:0] C_HEADER_BYTES = 8'd32;
	localparam [7:0] C_HEADER_FLAGS = 8'd1;
	localparam [15:0] C_WIDTH = 16'd640;
	localparam [15:0] C_HEIGHT = 16'd320;
	localparam [15:0] C_FRAGMENT_BYTES = 16'd1440;
	localparam [15:0] C_FRAGMENT_COUNT = 16'd285;
	localparam [15:0] C_LAST_FRAGMENT = 16'd284;
	localparam [15:0] C_LAST_FRAGMENT_BYTES = 16'd640;

	reg flag_packet_active;
	reg flag_packet_accept;
	reg flag_packet_bad;
	reg flag_frame_reserved;
	reg flag_grant_available;
	reg [15:0] cnt_packet_bytes;
	reg [255:0] reg_header;
	reg [31:0] reg_crc;
	reg [31:0] reg_crc_next;
	reg [31:0] reg_crc_start;
	reg [31:0] reg_session;
	reg [31:0] reg_sequence;
	reg [15:0] reg_fragment;
	reg [15:0] reg_payload_bytes;
	reg [15:0] reg_next_fragment;
	integer cnt_crc_bit;

	wire [255:0] header_complete;
	wire [15:0] header_expected_payload_bytes;
	wire header_format_good;
	wire header_sequence_good;

	assign header_complete = {reg_header[247:0], i_data};
	assign header_expected_payload_bytes =
		(header_complete[127:112] == C_LAST_FRAGMENT) ?
		C_LAST_FRAGMENT_BYTES : C_FRAGMENT_BYTES;
	assign header_format_good =
		(header_complete[255:224] == C_MAGIC) &&
		(header_complete[223:216] == C_VERSION) &&
		(header_complete[215:208] == C_VIDEO_TYPE) &&
		(header_complete[207:200] == C_HEADER_BYTES) &&
		(header_complete[199:192] == C_HEADER_FLAGS) &&
		(header_complete[191:160] != 32'd0) &&
		(header_complete[111:96] == C_FRAGMENT_COUNT) &&
		(header_complete[127:112] < C_FRAGMENT_COUNT) &&
		(header_complete[95:80] == header_expected_payload_bytes) &&
		(header_complete[79:64] == C_WIDTH) &&
		(header_complete[63:48] == C_HEIGHT) &&
		(header_complete[47:32] != 16'd0) &&
		(header_complete[47:32] <= 16'd60);
	assign header_sequence_good =
		(header_complete[127:112] == 16'd0) ?
		(!o_frame_active && flag_frame_reserved) :
		(o_frame_active &&
		 (header_complete[191:160] == reg_session) &&
		 (header_complete[159:128] == reg_sequence) &&
		 (header_complete[127:112] == reg_next_fragment));

	// PGV1 CRC32: initial state 0xffffffff, header byte 0 through 27 and
	// payload participate, header byte 28 through 31 are the transmitted CRC.
always@(*)begin
		reg_crc_next = reg_crc ^ {24'd0, i_data};
		reg_crc_start = 32'hffff_ffff ^ {24'd0, i_data};
		for(cnt_crc_bit = 0; cnt_crc_bit < 8; cnt_crc_bit = cnt_crc_bit + 1)begin
			if(reg_crc_next[0] == 1'b1)begin
				reg_crc_next = (reg_crc_next >> 1) ^ 32'hedb8_8320;
			end else begin
				reg_crc_next = reg_crc_next >> 1;
			end
		end
		for(cnt_crc_bit = 0; cnt_crc_bit < 8; cnt_crc_bit = cnt_crc_bit + 1)begin
			if(reg_crc_start[0] == 1'b1)begin
				reg_crc_start = (reg_crc_start >> 1) ^ 32'hedb8_8320;
			end else begin
				reg_crc_start = reg_crc_start >> 1;
			end
		end
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			flag_packet_active <= 1'b0;
			flag_packet_accept <= 1'b0;
			flag_packet_bad <= 1'b0;
			flag_frame_reserved <= 1'b0;
			flag_grant_available <= 1'b0;
			cnt_packet_bytes <= 16'd0;
			reg_header <= 256'd0;
			reg_crc <= 32'hffff_ffff;
			reg_session <= 32'd0;
			reg_sequence <= 32'd0;
			reg_fragment <= 16'd0;
			reg_payload_bytes <= 16'd0;
			reg_next_fragment <= 16'd0;
			o_frame_prepare <= 1'b0;
			o_payload_valid <= 1'b0;
			o_payload_data <= 8'd0;
			o_frame_commit <= 1'b0;
			o_frame_abort <= 1'b0;
			o_frame_active <= 1'b0;
			o_frame_slot <= 2'd0;
			o_frame_sequence <= 32'd0;
			o_crc_error_count <= 32'd0;
			o_sequence_error_count <= 32'd0;
			o_length_error_count <= 32'd0;
			o_dropped_frame_count <= 32'd0;
		end else begin
			o_frame_prepare <= 1'b0;
			o_payload_valid <= 1'b0;
			o_frame_commit <= 1'b0;
			o_frame_abort <= 1'b0;
			if (i_frame_grant) begin
				flag_grant_available <= 1'b1;
				o_frame_slot <= i_frame_grant_slot;
			end

			if (i_packer_overflow && flag_frame_reserved) begin
				flag_packet_accept <= 1'b0;
				flag_packet_bad <= 1'b1;
				o_frame_active <= 1'b0;
				flag_frame_reserved <= 1'b0;
				o_frame_abort <= 1'b1;
				o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
			end

			if (i_packet_start) begin
				// An inactive receiver may have stale words from a prior rejected
				// frame.  The C6 FIFO wrapper uses this pulse to clear them while
				// the following 32-byte PGV1 header is being received.
				if (!flag_frame_reserved && flag_grant_available) begin
					o_frame_prepare <= 1'b1;
					flag_frame_reserved <= 1'b1;
					flag_grant_available <= 1'b0;
				end
				if (flag_packet_active && flag_frame_reserved) begin
					o_frame_active <= 1'b0;
					flag_frame_reserved <= 1'b0;
					o_frame_abort <= 1'b1;
					o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
				end
				// 已保留槽位的后续 fragment 不需要新的 grant；只有 fragment 0
				// 消耗 grant 并触发 prepare。没有槽位时整个 UDP 包保持被忽略。
				flag_packet_active <= flag_frame_reserved ||
					(!flag_frame_reserved && flag_grant_available);
				flag_packet_accept <= 1'b0;
				flag_packet_bad <= 1'b0;
				if (flag_frame_reserved || flag_grant_available) begin
					cnt_packet_bytes <= i_data_valid ? 16'd1 : 16'd0;
					reg_header <= i_data_valid ? {248'd0, i_data} : 256'd0;
					reg_crc <= i_data_valid ? reg_crc_start : 32'hffff_ffff;
				end
			end else if (flag_packet_active && i_data_valid) begin
				if (cnt_packet_bytes < C_HEADER_BYTES) begin
					reg_header <= {reg_header[247:0], i_data};
				end
				if ((cnt_packet_bytes < 16'd28) || (cnt_packet_bytes >= C_HEADER_BYTES)) begin
					reg_crc <= reg_crc_next;
				end
				if (cnt_packet_bytes == 16'd31) begin
					reg_payload_bytes <= header_complete[95:80];
					reg_fragment <= header_complete[127:112];
					if (header_format_good && header_sequence_good) begin
						flag_packet_accept <= 1'b1;
						if (header_complete[127:112] == 16'd0) begin
							o_frame_active <= 1'b1;
							reg_session <= header_complete[191:160];
							reg_sequence <= header_complete[159:128];
							o_frame_sequence <= header_complete[159:128];
							reg_next_fragment <= 16'd0;
						end
					end else begin
						flag_packet_bad <= 1'b1;
						o_sequence_error_count <= o_sequence_error_count + 1'b1;
					end
				end
				if (flag_packet_accept && (cnt_packet_bytes >= C_HEADER_BYTES)) begin
					o_payload_valid <= 1'b1;
					o_payload_data <= i_data;
				end
				if (cnt_packet_bytes != 16'hffff) begin
					cnt_packet_bytes <= cnt_packet_bytes + 1'b1;
				end else begin
					flag_packet_bad <= 1'b1;
				end
			end

			if (i_packet_end && flag_packet_active) begin
				flag_packet_active <= 1'b0;
				flag_packet_accept <= 1'b0;
				if (flag_packet_bad || !flag_packet_accept ||
					(cnt_packet_bytes != (C_HEADER_BYTES + reg_payload_bytes)) ||
					((reg_crc ^ 32'hffff_ffff) != reg_header[31:0])) begin
					if (flag_frame_reserved) begin
						o_frame_active <= 1'b0;
						flag_frame_reserved <= 1'b0;
						o_frame_abort <= 1'b1;
						o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
					end
					if ((reg_crc ^ 32'hffff_ffff) != reg_header[31:0]) begin
						o_crc_error_count <= o_crc_error_count + 1'b1;
					end else if (cnt_packet_bytes != (C_HEADER_BYTES + reg_payload_bytes)) begin
						o_length_error_count <= o_length_error_count + 1'b1;
					end
				end else if (reg_fragment == C_LAST_FRAGMENT) begin
					o_frame_active <= 1'b0;
					flag_frame_reserved <= 1'b0;
					o_frame_commit <= 1'b1;
				end else begin
					reg_next_fragment <= reg_next_fragment + 1'b1;
				end
			end
		end
	end
endmodule
