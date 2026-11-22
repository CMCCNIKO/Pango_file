//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 把 DDR 256-bit 小端 word 流重组成连续 96-bit LUT entry；使用固定 32-bit 边界拼接和小型协议移位寄存器处理跨 word 条目。
// Module Name: avm_lut_stream_unpack
// File Name: source/avm/avm_lut_stream_unpack.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 把 DDR 256-bit 小端 word 流重组成连续 96-bit LUT entry；使用固定 32-bit 边界拼接和小型协议移位寄存器处理跨 word 条目。
// 完整 640x640 LUT 含 409600 entry/153600 word；frame_start 只在旧帧 DDR 请求排空后接受，输出按 ready/valid 反压。
//
// Dependencies: 无整帧 LUT 缓存 IP；上游 DDR 返回与下游 LUT 消费接口。
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
// Module Name: avm_lut_stream_unpack
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: DDR低bit在前256bit词→连续little endian96bit LUT协议重组。
// Dependencies: 无存储IP；352bit协议移位寄存器，不是LUT整帧缓存。
// Contract: frame_start只能在上帧DDR已排空后；409600entry=153600word。
// Revision: 2026-10-19 以固定32bit边界拼接替代可变移位，保持原拍数与握手。
//////////////////////////////////////////////////////////////////////////////////
module avm_lut_stream_unpack (
	input i_clk,
	input i_rst_n,
	input i_frame_start,
	input i_word_valid,
	output o_word_ready,
	input [255:0] i_word_data,
	output o_lut_valid,
	input i_lut_ready,
	output [95:0] o_lut,
	output [18:0] o_lut_index,
	output o_frame_done
);
	reg [351:0] shift_data;
	reg [8:0] valid_bits;
	reg [17:0] accepted_words;
	reg [18:0] lut_index;
	reg frame_running,frame_done_o;
	wire word_take=i_word_valid && o_word_ready;
	wire lut_take=o_lut_valid && i_lut_ready;
	assign o_word_ready=frame_running && accepted_words<153600 && (valid_bits<=96 || (lut_take && valid_bits<=192));
	assign o_lut_valid=frame_running && valid_bits>=96;
	assign o_lut=shift_data[95:0];
	assign o_lut_index=lut_index;
	assign o_frame_done=frame_done_o;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin shift_data<=0; valid_bits<=0; accepted_words<=0; lut_index<=0; frame_running<=0; frame_done_o<=0; end
		else begin
			frame_done_o<=0;
			if(i_frame_start) begin shift_data<=0; valid_bits<=0; accepted_words<=0; lut_index<=0; frame_running<=1; end
			else begin
				case({word_take,lut_take})
					// valid_bits只能为32的整数倍；单独接收词时可用0/32/64/96四种固定拼接。
					2'b10: begin
						case(valid_bits[6:5])
							2'b00: shift_data<={96'd0,i_word_data};
							2'b01: shift_data<={64'd0,i_word_data,shift_data[31:0]};
							2'b10: shift_data<={32'd0,i_word_data,shift_data[63:0]};
							2'b11: shift_data<={i_word_data,shift_data[95:0]};
							default: shift_data<=shift_data;
						endcase
						valid_bits<=valid_bits+9'd256;
					end
					2'b01: begin shift_data<=shift_data>>96; valid_bits<=valid_bits-9'd96; end
					// 同拍输出和接收时，偏移为valid_bits-96，仍只有0/32/64/96四种情况。
					2'b11: begin
						case(valid_bits[6:5])
							// valid_bits=96，偏移0
							2'b11: shift_data<={96'd0,i_word_data};
							// valid_bits=128/160/192，偏移32/64/96
							2'b00: shift_data<={64'd0,i_word_data,shift_data[127:96]};
							2'b01: shift_data<={32'd0,i_word_data,shift_data[159:96]};
							2'b10: shift_data<={i_word_data,shift_data[191:96]};
							default: shift_data<=shift_data;
						endcase
						valid_bits<=valid_bits+9'd160;
					end
					default: begin end
				endcase
				if(word_take) accepted_words<=accepted_words+1'b1;
				if(lut_take) begin
					if(lut_index==409599) begin frame_running<=0; frame_done_o<=1; end
					else lut_index<=lut_index+1'b1;
				end
			end
		end
	end
endmodule
