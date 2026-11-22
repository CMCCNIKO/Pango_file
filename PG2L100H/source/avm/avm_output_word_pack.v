//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 把完整 AVM 640x640 渲染像素按 16 个 RGB565 低像素优先打包为 256-bit DDR word，并按下游 ready 保持数据。
// Module Name: avm_output_word_pack
// File Name: source/avm/avm_output_word_pack.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 把完整 AVM 640x640 渲染像素按 16 个 RGB565 低像素优先打包为 256-bit DDR word，并按下游 ready 保持数据。
// 一帧 25600 word；words_done 只表示下游接收了全部打包字，真正 DDR 写完及 stream drained 仍由 writer 确认后才能发布输出槽。
//
// Dependencies: AVM 像素流、avm_single_beat_writer；frame_start 需等待旧 writer 排空。
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
// Module Name: avm_output_word_pack
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 完整640x640 LUT像素按16个RGB565低bit像素优先打包256bit。
// Contract: frame_start只在旧writer drained后；words_done只是25600词被下游接收，
//           DDR完整写入/stream drained仍须writer另行确认后才能发布输出slot。
//////////////////////////////////////////////////////////////////////////////////
module avm_output_word_pack (
	input i_clk,
	input i_rst_n,
	input i_frame_start,
	input i_pixel_valid,
	output o_pixel_ready,
	input [15:0] i_pixel_rgb565,
	input [18:0] i_pixel_index,
	input i_pixel_error,
	output o_word_valid,
	input i_word_ready,
	output [255:0] o_word_data,
	output [14:0] o_word_index,
	output o_words_done,
	output o_frame_error
);
	localparam integer C_INPUT_WIDTH=640;
	localparam integer C_INPUT_HEIGHT=640;
	localparam integer C_CROP_X=80;
	localparam integer C_OUTPUT_WIDTH=480;
	localparam integer C_OUTPUT_HEIGHT=640;
	localparam integer C_OUTPUT_PIXELS=C_OUTPUT_WIDTH*C_OUTPUT_HEIGHT;
	localparam integer C_OUTPUT_WORDS=C_OUTPUT_PIXELS/16;
	reg [255:0] word_data_o;
	reg word_valid_o,frame_running,frame_error_o,words_done_o;
	reg [18:0] next_input_index;
	reg [18:0] next_output_index;
	reg [9:0] input_x;
	reg [14:0] word_index_o;
	wire pixel_take=i_pixel_valid && o_pixel_ready;
	wire word_take=o_word_valid && i_word_ready;
	assign o_pixel_ready=frame_running && !frame_error_o && (!word_valid_o || i_word_ready) && next_input_index<(C_INPUT_WIDTH*C_INPUT_HEIGHT);
	assign o_word_valid=word_valid_o && !frame_error_o;
	assign o_word_data=word_data_o;
	assign o_word_index=word_index_o;
	assign o_words_done=words_done_o;
	assign o_frame_error=frame_error_o;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin word_data_o<=0; word_valid_o<=0; frame_running<=0; frame_error_o<=0;
			words_done_o<=0; next_input_index<=0; next_output_index<=0; input_x<=0; word_index_o<=0; end
		else begin
			words_done_o<=0;
			if(i_frame_start) begin word_data_o<=0; word_valid_o<=0; frame_running<=1; frame_error_o<=0;
				next_input_index<=0; next_output_index<=0; input_x<=0; word_index_o<=0; end
			else begin
				if(word_take) begin
					word_valid_o<=0;
					if(word_index_o==C_OUTPUT_WORDS-1) begin words_done_o<=1; frame_running<=0; end
					else word_index_o<=word_index_o+1'b1;
				end
				if(pixel_take) begin
					if(i_pixel_error || i_pixel_index!=next_input_index) frame_error_o<=1;
					next_input_index<=next_input_index+1'b1;
					if(input_x>=C_CROP_X && input_x<(C_CROP_X+C_OUTPUT_WIDTH)) begin
						word_data_o[next_output_index[3:0]*16+:16]<=i_pixel_rgb565;
						next_output_index<=next_output_index+1'b1;
						if(next_output_index[3:0]==15) word_valid_o<=1;
					end
					if(input_x==C_INPUT_WIDTH-1) input_x<=0;
					else input_x<=input_x+1'b1;
				end
			end
		end
	end
endmodule
