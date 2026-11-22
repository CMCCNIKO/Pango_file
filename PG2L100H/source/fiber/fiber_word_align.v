//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026/09/17
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: HSST 32-bit 接收字节对齐器，用 previous/current 64-bit 窗口把合法 K28.5 对齐到 byte0。
// Module Name: fiber_word_align
// File Name: source/fiber/fiber_word_align.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// HSST 32-bit 接收字节对齐器，用 previous/current 64-bit 窗口把合法 K28.5 对齐到 byte0。
// 数据、K 标志和逐字节 decode/disparity 错误使用同一偏移窗口；失锁或偏移变化输出 resync，使下游立即作废旧帧事务。
//
// Dependencies: HSST 同拍 32-bit 数据/K/错误接口；无 IP 依赖。
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
// Create Date: 2026/09/17
// Design Name: Fiber RX Word Aligner
// Module Name: fiber_word_align
// Project Name: Ethernet_HDMI_OV5640_SD_HDMI_OUT
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 使用 previous/current 64-bit 窗口，将合法 K28.5 对齐到 byte 0。
//
// Dependencies: None
//
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - 2026/10/21 错误字节同窗口对齐，失锁/偏移变化中止旧事务。
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module fiber_word_align(
	input         i_clk,
	input         i_rst_n,
	input         i_rx_valid,
	input  [31:0] i_rxd,
	input  [3:0]  i_rxk,
	input  [3:0]  i_decode_error,
	input  [3:0]  i_disparity_error,
	output        o_aligned_valid,
	output [31:0] o_aligned_rxd,
	output [3:0]  o_aligned_rxk,
	output        o_lock,
	output        o_align_error,
	output        o_aligned_phy_error,
	output        o_resync
);

	//--------------寄存器信号--------------//
	reg [31:0] reg_previous_rxd;			// 上一个有效 RX word，构成 64-bit 窗口低半部。
	reg [3:0]  reg_previous_rxk;			// 上一个有效 RX word 的 K 标志。
	reg [3:0]  reg_previous_error;			// 与 previous 字节对应的 PCS 错误。
	reg        reg_previous_valid;			// previous word 已有效，允许输出完整对齐字。
	reg [1:0]  reg_byte_offset;			// K28.5 在原始 RX word 内的 byte 偏移。
	reg [1:0]  dec_byte_offset;			// 当前合法 K28.5 解码得到的候选偏移。
	reg        flag_legal_k28_5;			// rxk onehot 且对应 byte 为 K28.5。
	reg [63:0] reg_data_window;			// {current_rxd, previous_rxd} 的跨字窗口。
	reg [7:0]  reg_k_window;			// {current_rxk, previous_rxk} 的跨字 K 窗口。
	reg [63:0] reg_shifted_data;			// 按已锁定 byte 偏移右移后的数据窗口。
	reg [7:0]  reg_shifted_k;			// 按已锁定 byte 偏移右移后的 K 窗口。
	reg [7:0]  reg_error_window;			// 错误位使用相同 previous/current 窗口。
	reg [7:0]  reg_shifted_error;			// 与重组后字节一一对应的错误位。

	//---------------输出信号---------------//
	reg        aligned_valid_o;			// 完整对齐字在当前拍有效。
	reg [31:0] aligned_rxd_o;			// 跨字窗口重组后的 32-bit 数据。
	reg [3:0]  aligned_rxk_o;			// 与数据使用相同偏移的 K 标志。
	reg        lock_o;				// 已从合法 K28.5 获得字节偏移。
	reg        align_error_o;			// 当前锁定期间观察到非法 K 标志。
	reg        aligned_phy_error_o;		// 当前对齐 word 的四个字节错误归约。
	reg        resync_o;				// 本拍失锁或换偏移，旧事务必须作废。

	//-------------输出信号连线-------------//
	assign o_aligned_valid = aligned_valid_o;		// 输出桥接不增加寄存级。
	assign o_aligned_rxd = aligned_rxd_o;		// 保持原对齐数据拍数。
	assign o_aligned_rxk = aligned_rxk_o;		// 保持 K sideband 与数据同拍。
	assign o_lock = lock_o;				// 输出当前偏移锁定状态。
	assign o_align_error = align_error_o;		// 输出当前对齐错误状态。
	assign o_aligned_phy_error = aligned_phy_error_o;	// 与 aligned_valid/data 同拍。
	assign o_resync = resync_o;			// 异常事件不改变正常流拍数。

	//-----------主要任务处理区域-----------//
	// 仅接受 onehot K 标志且 K 所在字节确为 8'hBC 的 K28.5。
	always@(*)begin
		flag_legal_k28_5 = 1'b0;
		dec_byte_offset = 2'd0;
		case(i_rxk)
			4'b0001:begin
				if(i_rxd[7:0] == 8'hBC)begin
					flag_legal_k28_5 = 1'b1;
					dec_byte_offset = 2'd0;
				end
			end
			4'b0010:begin
				if(i_rxd[15:8] == 8'hBC)begin
					flag_legal_k28_5 = 1'b1;
					dec_byte_offset = 2'd1;
				end
			end
			4'b0100:begin
				if(i_rxd[23:16] == 8'hBC)begin
					flag_legal_k28_5 = 1'b1;
					dec_byte_offset = 2'd2;
				end
			end
			4'b1000:begin
				if(i_rxd[31:24] == 8'hBC)begin
					flag_legal_k28_5 = 1'b1;
					dec_byte_offset = 2'd3;
				end
			end
			default:begin
				flag_legal_k28_5 = 1'b0;
				dec_byte_offset = 2'd0;
			end
		endcase
	end

	// previous/current 组成窗口；右移后的低 32/4 bit 即为一个完整发送字。
	always@(*)begin
		reg_data_window = {i_rxd, reg_previous_rxd};
		reg_k_window = {i_rxk, reg_previous_rxk};
		reg_error_window = {i_decode_error | i_disparity_error, reg_previous_error};
		case(reg_byte_offset)
			2'd0:begin
				reg_shifted_data = reg_data_window >> 0;
				reg_shifted_k = reg_k_window >> 0;
				reg_shifted_error = reg_error_window >> 0;
			end
			2'd1:begin
				reg_shifted_data = reg_data_window >> 8;
				reg_shifted_k = reg_k_window >> 1;
				reg_shifted_error = reg_error_window >> 1;
			end
			2'd2:begin
				reg_shifted_data = reg_data_window >> 16;
				reg_shifted_k = reg_k_window >> 2;
				reg_shifted_error = reg_error_window >> 2;
			end
			2'd3:begin
				reg_shifted_data = reg_data_window >> 24;
				reg_shifted_k = reg_k_window >> 3;
				reg_shifted_error = reg_error_window >> 3;
			end
			default:begin
				reg_shifted_data = reg_data_window >> 0;
				reg_shifted_k = reg_k_window >> 0;
				reg_shifted_error = reg_error_window >> 0;
			end
		endcase
	end

	// 保存每个有效 RX word，并在下一有效 word 到来时输出其跨边界对齐结果。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			reg_previous_rxd <= 32'h00000000;
			reg_previous_rxk <= 4'b0000;
			reg_previous_error <= 4'b0000;
			reg_previous_valid <= 1'b0;
			reg_byte_offset <= 2'd0;
			aligned_valid_o <= 1'b0;
			aligned_rxd_o <= 32'h00000000;
			aligned_rxk_o <= 4'b0000;
			lock_o <= 1'b0;
			align_error_o <= 1'b0;
			aligned_phy_error_o <= 1'b0;
			resync_o <= 1'b0;
		end else begin
			aligned_valid_o <= 1'b0;
			aligned_phy_error_o <= 1'b0;
			resync_o <= 1'b0;
			if(i_rx_valid == 1'b1)begin
				aligned_valid_o <= reg_previous_valid && lock_o;
				aligned_rxd_o <= reg_shifted_data[31:0];
				aligned_rxk_o <= reg_shifted_k[3:0];
				aligned_phy_error_o <= |reg_shifted_error[3:0];
				reg_previous_rxd <= i_rxd;
				reg_previous_rxk <= i_rxk;
				reg_previous_error <= i_decode_error | i_disparity_error;
				reg_previous_valid <= 1'b1;
				if(flag_legal_k28_5 == 1'b1)begin
					// 换偏移当拍禁止输出旧窗口；当前原始字作为新窗口首字。
					if((lock_o == 1'b1) && (dec_byte_offset != reg_byte_offset))begin
						aligned_valid_o <= 1'b0;
						aligned_phy_error_o <= 1'b0;
						resync_o <= 1'b1;
					end
					reg_byte_offset <= dec_byte_offset;
					lock_o <= 1'b1;
					align_error_o <= 1'b0;
				end else if((|i_rxk) == 1'b1)begin
					// 非 K28.5 或多 K 字不能继续提供可信窗口。
					aligned_valid_o <= 1'b0;
					aligned_phy_error_o <= 1'b0;
					reg_previous_rxd <= 32'd0;
					reg_previous_rxk <= 4'd0;
					reg_previous_error <= 4'd0;
					reg_previous_valid <= 1'b0;
					reg_byte_offset <= 2'd0;
					lock_o <= 1'b0;
					align_error_o <= 1'b1;
					resync_o <= 1'b1;
				end
			end else begin
				// A gap means the recovered stream is no longer trustworthy
				// (LOS/lane reset or clock recovery).  Drop previous bytes and
				// the learned offset so recovery cannot splice old and new data.
				reg_previous_rxd <= 32'h00000000;
				reg_previous_rxk <= 4'b0000;
				reg_previous_error <= 4'b0000;
				reg_previous_valid <= 1'b0;
				reg_byte_offset <= 2'd0;
				lock_o <= 1'b0;
				align_error_o <= 1'b0;
				resync_o <= lock_o || reg_previous_valid;
			end
		end
	end

endmodule
