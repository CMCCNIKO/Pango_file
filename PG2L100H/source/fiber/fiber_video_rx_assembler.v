//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026/09/17
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 检查光纤视频的 VSYNC/HSTART/DATA/HEND 协议及每行/每帧长度，完整帧为 640x320 RGB565。
// Module Name: fiber_video_rx_assembler
// File Name: source/fiber/fiber_video_rx_assembler.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 检查光纤视频的 VSYNC/HSTART/DATA/HEND 协议及每行/每帧长度，完整帧为 640x320 RGB565。
// 连续八个 32-bit DATA 拼为一个 256-bit word，先到像素放低位；完整帧输出 12800 word，协议错误、PHY 错误或重对齐立即作废活动帧并更新计数。
//
// Dependencies: fiber_word_align 的同拍数据/K/错误；下游 source2 帧接收及 CDC。
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
// Design Name: Fiber Video RX Assembler
// Module Name: fiber_video_rx_assembler
// Project Name: Ethernet_HDMI_OV5640_SD_HDMI_OUT
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 校验光纤视频帧协议，并将连续八个 32-bit DATA 拼成 DDR 256-bit word。
//
// Dependencies: None
//
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - 2026/10/21 暴露组合帧活动状态，重新对齐立即作废活动帧。
// Revision 0.03 - 2026/10/21 独立 DATA 译码和拼包缓存写入，降低异常清空扇出。
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module fiber_video_rx_assembler
(
	//---------------全局信号---------------//
	input          i_clk,                         // HSST RX fabric 时钟
	input          i_rst_n,                       // 本时钟域低有效异步复位

	//--------------链路字接口--------------//
	input          i_link_ready,                  // PLL、RX lane 与 SFP LOS 汇总后的链路就绪
	input          i_phy_error,                   // 与当前对齐 word 同拍的 PCS 错误
	input          i_resync,                      // 对齐器失锁/偏移变化事件
	input          i_word_valid,                  // 当前 32-bit 接收字有效
	input  [31:0]  i_rxd,                         // 对齐后的链路数据
	input  [3:0]   i_rxk,                         // 对齐后的每字节 K 标志

	//--------------帧事件接口--------------//
	output         o_frame_start,                 // 接收新帧 VSYNC 时输出一拍
	output         o_frame_end,                   // 完整 640x320 帧结束时输出一拍
	output         o_frame_error,                 // 当前活动帧协议错误时输出一拍
	output         o_frame_active,                // 组合帧状态，最后 HEND 后不多延迟一拍

	//-------------DDR 字流接口-------------//
	output         o_word256_valid,               // 每收齐八个 DATA words 输出一拍
	output [255:0] o_word256_data,                // 16 个 RGB565 像素，先到像素位于低位

	//--------------调试计数接口------------//
	output [9:0]   o_x,                           // 当前帧内下一期望像素横坐标
	output [8:0]   o_y,                           // 当前帧内下一期望行坐标
	output [31:0]  o_good_frame_count,            // 复位后完整帧累计数
	output [31:0]  o_bad_frame_count              // 复位后作废帧累计数
);

	//-------------协议参数区域-------------//
	localparam [31:0] C_IDLE   = 32'h000000BC;    // 任意状态均可忽略的空闲控制字
	localparam [31:0] C_VSYNC  = 32'hFFAA55BC;    // 新帧开始控制字
	localparam [31:0] C_HSTART = 32'hAA0000BC;    // 新行开始控制字
	localparam [31:0] C_HEND   = 32'hCC0000BC;    // 当前行结束控制字

	localparam [8:0]  C_LINE_DATA_WORDS = 9'd320;     // 每行两个像素一字，共 320 DATA words
	localparam [8:0]  C_FRAME_ROWS = 9'd320;          // 每帧 320 行
	localparam [16:0] C_FRAME_DATA_WORDS = 17'd102400;// 每帧 32-bit DATA word 总数
	localparam [13:0] C_FRAME_DDR_WORDS = 14'd12800;  // 每帧 256-bit DDR word 总数

	//-------------状态参数区域-------------//
	localparam [1:0] ST_WAIT_VSYNC  = 2'd0;       // 等待下一次可接受的帧同步
	localparam [1:0] ST_WAIT_HSTART = 2'd1;       // 活动帧中等待下一行开始
	localparam [1:0] ST_IN_LINE     = 2'd2;       // 接收当前行的 320 个 DATA words

	//--------------状态机信号--------------//
	reg [1:0] state_current;                      // 当前协议接收阶段
	reg [1:0] state_next;                         // 下一拍协议接收阶段

	//---------------计数信号---------------//
	reg [8:0]  cnt_line_data;                     // 当前行已接收的 DATA word 数
	reg [8:0]  cnt_row;                           // 当前活动帧的行索引
	reg [16:0] cnt_frame_data;                    // 当前帧已接收的 DATA word 总数
	reg [13:0] cnt_word256;                       // 当前帧已输出的 256-bit word 总数
	reg [2:0]  cnt_pack_phase;                    // 当前 256-bit word 已填入的 32-bit 槽号
	reg [9:0]  cnt_x;                             // 下一期望像素横坐标

	//--------------寄存器信号--------------//
	reg [223:0] reg_pack_data;                    // 前七个 DATA words 的拼包缓存

	//---------------标志信号---------------//
	reg flag_accept_vsync;                        // 当前字启动一个新帧
	reg flag_accept_hstart;                       // 当前字启动一行
	wire flag_accept_data;                        // 当前字是合法行内 DATA
	reg flag_accept_hend;                         // 当前字合法结束一行
	reg flag_complete_frame;                      // 当前 HEND 完整结束最后一行
	reg flag_abort_frame;                         // 当前拍必须作废活动帧

	//---------------输出信号---------------//
	reg         frame_start_o;                    // 新帧开始单拍事件
	reg         frame_end_o;                      // 完整帧结束单拍事件
	reg         frame_error_o;                    // 错误帧作废单拍事件
	reg         word256_valid_o;                  // DDR word 有效单拍事件
	reg [255:0] word256_data_o;                   // 当前输出 DDR word
	reg [31:0]  good_frame_count_o;               // 完整帧累计计数
	reg [31:0]  bad_frame_count_o;                // 错误帧累计计数

	//-------------输出信号连线-------------//
	assign o_frame_start = frame_start_o;          // 输出桥接不增加寄存级
	assign o_frame_end = frame_end_o;              // 输出桥接不增加寄存级
	assign o_frame_error = frame_error_o;          // 输出桥接不增加寄存级
	assign o_frame_active = state_current != ST_WAIT_VSYNC; // 只读状态，不参与像素背压
	assign o_word256_valid = word256_valid_o;      // 与 word256_data_o 同拍有效
	assign o_word256_data = word256_data_o;        // 保持接收端原始像素位序
	assign o_x = cnt_x;                            // 暴露下一期望像素横坐标
	assign o_y = cnt_row;                          // 暴露下一期望行坐标
	assign o_good_frame_count = good_frame_count_o;// 完整帧累计统计
	assign o_bad_frame_count = bad_frame_count_o;  // 错误帧累计统计
	// DATA 的 K=0，与所有 K=1 控制字互斥，无需经过控制字/abort 的优先链。
	assign flag_accept_data = i_link_ready && !i_phy_error && !i_resync &&
		i_word_valid && (i_rxk == 4'd0) && (state_current == ST_IN_LINE) &&
		(cnt_line_data < C_LINE_DATA_WORDS);

	//--------------状态机区域--------------//
	// 状态寄存器只保存协议接收阶段，非法状态由下一状态逻辑恢复。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			state_current <= ST_WAIT_VSYNC;
		end else begin
			state_current <= state_next;
		end
	end

	// 根据当前已译码事件决定下一协议阶段。
	always@(*)begin
		state_next = state_current;
		if(flag_abort_frame == 1'b1)begin
			state_next = ST_WAIT_VSYNC;
		end else begin
			case(state_current)
				ST_WAIT_VSYNC:begin
					if(flag_accept_vsync == 1'b1)begin
						state_next = ST_WAIT_HSTART;
					end
				end
				ST_WAIT_HSTART:begin
					if(flag_accept_hstart == 1'b1)begin
						state_next = ST_IN_LINE;
					end
				end
				ST_IN_LINE:begin
					if(flag_complete_frame == 1'b1)begin
						state_next = ST_WAIT_VSYNC;
					end else if(flag_accept_hend == 1'b1)begin
						state_next = ST_WAIT_HSTART;
					end
				end
				default:begin
					state_next = ST_WAIT_VSYNC;
				end
			endcase
		end
	end

	//-----------状态任务处理区域-----------//
	// 严格译码控制字和 DATA，并把所有活动帧错误汇总为一次 abort 事件。
	always@(*)begin
		flag_accept_vsync = 1'b0;
		flag_accept_hstart = 1'b0;
		flag_accept_hend = 1'b0;
		flag_complete_frame = 1'b0;
		flag_abort_frame = 1'b0;

		if((i_link_ready == 1'b0) || (i_phy_error == 1'b1) || (i_resync == 1'b1))begin
			if(state_current != ST_WAIT_VSYNC)begin
				flag_abort_frame = 1'b1;
			end
		end else if((i_link_ready == 1'b1) && (i_word_valid == 1'b1))begin
			if((i_rxk == 4'b0001) && (i_rxd == C_IDLE))begin
				// IDLE 在所有状态均忽略，不改变任何计数或状态。
			end else if((i_rxk == 4'b0001) && (i_rxd == C_VSYNC))begin
				if(state_current == ST_WAIT_VSYNC)begin
					flag_accept_vsync = 1'b1;
				end else begin
					// 当前 VSYNC 只负责作废旧帧，下一 VSYNC 才能重新开始。
					flag_abort_frame = 1'b1;
				end
			end else if((i_rxk == 4'b0001) && (i_rxd == C_HSTART))begin
				if(state_current == ST_WAIT_HSTART)begin
					flag_accept_hstart = 1'b1;
				end else if(state_current != ST_WAIT_VSYNC)begin
					flag_abort_frame = 1'b1;
				end
			end else if((i_rxk == 4'b0001) && (i_rxd == C_HEND))begin
				if(state_current == ST_IN_LINE)begin
					if(cnt_line_data == C_LINE_DATA_WORDS)begin
						if(cnt_row == (C_FRAME_ROWS - 1'b1))begin
							if((cnt_frame_data == C_FRAME_DATA_WORDS) &&
								(cnt_pack_phase == 3'd0) &&
								(cnt_word256 == C_FRAME_DDR_WORDS))begin
								flag_accept_hend = 1'b1;
								flag_complete_frame = 1'b1;
							end else begin
								flag_abort_frame = 1'b1;
							end
						end else begin
							flag_accept_hend = 1'b1;
						end
					end else begin
						flag_abort_frame = 1'b1;
					end
				end else if(state_current != ST_WAIT_VSYNC)begin
					flag_abort_frame = 1'b1;
				end
			end else if(i_rxk == 4'b0000)begin
				if(state_current == ST_IN_LINE)begin
					if(cnt_line_data >= C_LINE_DATA_WORDS)begin
						flag_abort_frame = 1'b1;
					end
				end else if(state_current != ST_WAIT_VSYNC)begin
					flag_abort_frame = 1'b1;
				end
			end else if(state_current != ST_WAIT_VSYNC)begin
				// 活动帧中任何其它 K 组合或未知控制字均作废整帧。
				flag_abort_frame = 1'b1;
			end
		end
	end

	//-----------主要任务处理区域-----------//
	// 只有合法 DATA 更新像素寄存器；每次输出前七槽均已被本轮 DATA 覆盖。
	// abort/VSYNC 仍清 phase/计数并抑制 DATA，不必向 224-bit 缓存分发清零。
	// 输出数据在无效周期也保持旧值，所有外部端口拍数保持不变。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			reg_pack_data <= 224'd0;
			word256_data_o <= 256'd0;
		end else if(flag_accept_data == 1'b1)begin
			if(cnt_pack_phase == 3'd7)begin
				word256_data_o <= {i_rxd, reg_pack_data};
			end else begin
				reg_pack_data[cnt_pack_phase*32 +: 32] <= i_rxd;
			end
		end
	end

	// 接收事件更新行列、拼包和帧统计；错误优先级高于同拍协议字。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			cnt_line_data <= 9'd0;
			cnt_row <= 9'd0;
			cnt_frame_data <= 17'd0;
			cnt_word256 <= 14'd0;
			cnt_pack_phase <= 3'd0;
			cnt_x <= 10'd0;
			frame_start_o <= 1'b0;
			frame_end_o <= 1'b0;
			frame_error_o <= 1'b0;
			word256_valid_o <= 1'b0;
			good_frame_count_o <= 32'd0;
			bad_frame_count_o <= 32'd0;
		end else begin
			frame_start_o <= 1'b0;
			frame_end_o <= 1'b0;
			frame_error_o <= 1'b0;
			word256_valid_o <= 1'b0;

			if(flag_abort_frame == 1'b1)begin
				cnt_line_data <= 9'd0;
				cnt_row <= 9'd0;
				cnt_frame_data <= 17'd0;
				cnt_word256 <= 14'd0;
				cnt_pack_phase <= 3'd0;
				cnt_x <= 10'd0;
				frame_error_o <= 1'b1;
				bad_frame_count_o <= bad_frame_count_o + 1'b1;
			end else if(flag_accept_vsync == 1'b1)begin
				cnt_line_data <= 9'd0;
				cnt_row <= 9'd0;
				cnt_frame_data <= 17'd0;
				cnt_word256 <= 14'd0;
				cnt_pack_phase <= 3'd0;
				cnt_x <= 10'd0;
				frame_start_o <= 1'b1;
			end else if(flag_accept_hstart == 1'b1)begin
				cnt_line_data <= 9'd0;
				cnt_x <= 10'd0;
			end else if(flag_accept_data == 1'b1)begin
				cnt_line_data <= cnt_line_data + 1'b1;
				cnt_frame_data <= cnt_frame_data + 1'b1;
				cnt_x <= cnt_x + 2'd2;
				if(cnt_pack_phase == 3'd7)begin
					cnt_pack_phase <= 3'd0;
					cnt_word256 <= cnt_word256 + 1'b1;
					word256_valid_o <= 1'b1;
				end else begin
					cnt_pack_phase <= cnt_pack_phase + 1'b1;
				end
			end else if(flag_accept_hend == 1'b1)begin
				cnt_line_data <= 9'd0;
				cnt_x <= 10'd0;
				if(flag_complete_frame == 1'b1)begin
					cnt_row <= 9'd0;
					cnt_frame_data <= 17'd0;
					cnt_word256 <= 14'd0;
					cnt_pack_phase <= 3'd0;
					frame_end_o <= 1'b1;
					good_frame_count_o <= good_frame_count_o + 1'b1;
				end else begin
					cnt_row <= cnt_row + 1'b1;
				end
			end
		end
	end

endmodule
