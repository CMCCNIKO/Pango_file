//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026/10/21
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旁路监听帧间 LANE-META-V1 四个 32-bit 车道元数据字，完整合法 END 后才发布 {sample_seq,payload}。
// Module Name: fiber_lane_meta_rx
// File Name: source/fiber/fiber_lane_meta_rx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旁路监听帧间 LANE-META-V1 四个 32-bit 车道元数据字，完整合法 END 后才发布 {sample_seq,payload}。
// 候选等待最多八个 RX 时钟，不回压视频、不写入视频 FIFO，也不把属性绑定到视频帧；无链路 ACK/CRC。
//
// Dependencies: fiber_word_align 同拍数据/K/错误、稳定数据 mailbox；调用方提供 RX 域同步释放复位。
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
// Create Date: 2026/10/21
// Design Name: Fiber Lane Metadata Receiver
// Module Name: fiber_lane_meta_rx
// Project Name: PG2L100H_AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 帧间旁路解析四个 32-bit 车道字，不阻塞视频；完整 END 才发布
//              {sample_seq,payload}。候选最多等待八个 RX 时钟，不含视频帧绑定。
// Dependencies: fiber_word_align 的同拍数据/K/错误；下游稳定数据 mailbox。
// Revision:
// Revision 0.01 - 2026/10/21 首次实现 LANE-META-V1。
// Additional Comments: i_rst_n 由调用方在 RX 域同步释放；无链路 ACK/CRC。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module fiber_lane_meta_rx
(
	input i_clk,                                  // HSST 恢复时钟 78.125 MHz
	input i_rst_n,                                // 本时钟域低有效复位
	input i_link_ready,                           // 当前链路及对齐流可用
	input i_stream_error,                         // 与当前对齐 word 同拍的字节错误归约
	input i_resync,                               // 失锁或换偏移事件，最高优先级
	input i_video_frame_active,                   // assembler 当前组合帧活动状态
	input i_word_valid,                           // 当前对齐 32-bit 字有效
	input [31:0] i_rxd,                           // byte0 位于 [7:0]
	input [3:0] i_rxk,                            // 每个字节的 K 标志
	output o_snapshot_valid,                      // 完整包提交单拍，无背压
	output [63:0] o_snapshot                      // {序号,规范化车道 payload}
);

	//-------------协议参数区域-------------//
	localparam [31:0] C_META_BEGIN = 32'h4C4D01BC; // 唯一版本 1 车道包头
	localparam [31:0] C_META_END = 32'h4C4DFFBC;   // 四字定长包尾
	localparam [31:0] C_VSYNC = 32'hFFAA55BC;      // 视频始终直接旁路，这里只放弃候选
	localparam [1:0] ST_IDLE = 2'd0;              // 仅在视频帧间等待 BEGIN
	localparam [1:0] ST_SEQUENCE = 2'd1;          // 期待无 K 的 32-bit 发布序号
	localparam [1:0] ST_PAYLOAD = 2'd2;           // 期待无 K 的完整车道字段
	localparam [1:0] ST_END = 2'd3;               // 期待精确 END，禁止提前发布

	//--------------状态机信号--------------//
	reg [1:0] state_current;                     // 当前候选解析阶段
	reg [1:0] state_next;                        // 完整覆盖的下一状态
	reg [2:0] cnt_timeout;                       // BEGIN 后的 RX 时钟数，7 后作废

	//--------------寄存器信号--------------//
	reg [31:0] reg_sequence;                     // 当前候选序号，不比较单调性
	reg [31:0] reg_payload;                      // 当前候选字段，END 前不可见
	reg [31:0] dec_normalized_payload;           // 屏蔽保留位与无效字段

	//---------------标志信号---------------//
	wire flag_global_abort;                      // 断链/错误/视频活动抢占候选
	wire flag_begin;                             // 完整且 K 正确的 BEGIN
	wire flag_vsync;                             // 正确视频帧头，最高协议优先级
	wire flag_timeout;                           // 候选已用完八时钟预算
	wire flag_accept_sequence;                   // 本拍采样候选序号
	wire flag_accept_payload;                    // 本拍采样候选字段
	wire flag_commit;                            // 本拍精确包尾验证通过

	//---------------输出信号---------------//
	reg snapshot_valid_o;                        // 发布事件，数值相同也刷新
	reg [63:0] snapshot_o;                       // 已完整验证的最新整包

	//-------------其他信号连线-------------//
	assign flag_global_abort = !i_link_ready || i_stream_error || i_resync || i_video_frame_active;
	assign flag_begin = i_word_valid && (i_rxk == 4'b0001) && (i_rxd == C_META_BEGIN);
	assign flag_vsync = i_word_valid && (i_rxk == 4'b0001) && (i_rxd == C_VSYNC);
	assign flag_timeout = (state_current != ST_IDLE) && (cnt_timeout == 3'd7);
	assign flag_accept_sequence = !flag_global_abort && !flag_timeout &&
		(state_current == ST_SEQUENCE) && i_word_valid && (i_rxk == 4'd0);
	assign flag_accept_payload = !flag_global_abort && !flag_timeout &&
		(state_current == ST_PAYLOAD) && i_word_valid && (i_rxk == 4'd0);
	assign flag_commit = !flag_global_abort && !flag_timeout &&
		(state_current == ST_END) && i_word_valid && (i_rxk == 4'b0001) && (i_rxd == C_META_END);

	//-------------输出信号连线-------------//
	assign o_snapshot_valid = snapshot_valid_o;  // 输出桥接不增加拍数
	assign o_snapshot = snapshot_o;              // END 后一次性发布整个 64-bit 载荷

	//--------------状态机区域--------------//
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			state_current <= ST_IDLE;
		end else begin
			state_current <= state_next;
		end
	end

	// 错误最高优先，VSYNC 不吞视频；新 BEGIN 在空闲视频流中可重启候选。
	always@(*)begin
		state_next = state_current;
		if(flag_global_abort || flag_vsync || flag_timeout)begin
			state_next = ST_IDLE;
		end else if(flag_begin)begin
			state_next = ST_SEQUENCE;
		end else begin
			case(state_current)
				ST_IDLE:begin
					state_next = ST_IDLE;
				end
				ST_SEQUENCE:begin
					if(i_word_valid)begin
						if(i_rxk == 4'd0)begin
							state_next = ST_PAYLOAD;
						end else begin
							state_next = ST_IDLE;
						end
					end
				end
				ST_PAYLOAD:begin
					if(i_word_valid)begin
						if(i_rxk == 4'd0)begin
							state_next = ST_END;
						end else begin
							state_next = ST_IDLE;
						end
					end
				end
				ST_END:begin
					if(i_word_valid)begin
						state_next = ST_IDLE;
					end
				end
				default:begin
					state_next = ST_IDLE;
				end
			endcase
		end
	end

	//-----------状态任务处理区域-----------//
	// 八时钟是恢复上界；正常连续四字只需 BEGIN 后三拍。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_timeout <= 3'd0;
		end else if(flag_global_abort || flag_vsync || flag_timeout || flag_begin || (state_current == ST_IDLE))begin
			cnt_timeout <= 3'd0;
		end else begin
			cnt_timeout <= cnt_timeout + 1'b1;
		end
	end

	// 两个 DATA 分别锁存；异常后旧候选不可能绕过新的两字接收直接提交。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_sequence <= 32'd0;
			reg_payload <= 32'd0;
		end else begin
			if(flag_accept_sequence)begin
				reg_sequence <= i_rxd;
			end
			if(flag_accept_payload)begin
				reg_payload <= i_rxd;
			end
		end
	end

	// 无效数量清零；类型 3、无效类型及有效数量 0 均不能保留旧虚实线。
	always@(*)begin
		dec_normalized_payload = {8'd0, reg_payload[23:0]};
		if(!reg_payload[20])begin
			dec_normalized_payload[7:0] = 8'd0;
		end
		if(!reg_payload[21])begin
			dec_normalized_payload[15:8] = 8'd0;
		end
		if(!reg_payload[22] || (reg_payload[17:16] == 2'd3) ||
			(reg_payload[20] && (reg_payload[7:0] == 8'd0)))begin
			dec_normalized_payload[17:16] = 2'd0;
			dec_normalized_payload[22] = 1'b0;
		end
		if(!reg_payload[23] || (reg_payload[19:18] == 2'd3) ||
			(reg_payload[21] && (reg_payload[15:8] == 8'd0)))begin
			dec_normalized_payload[19:18] = 2'd0;
			dec_normalized_payload[23] = 1'b0;
		end
	end

	//--------------输出信号处理区域--------------//
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			snapshot_valid_o <= 1'b0;
			snapshot_o <= 64'd0;
		end else begin
			snapshot_valid_o <= flag_commit;
			if(flag_commit)begin
				snapshot_o <= {reg_sequence, dec_normalized_payload};
			end
		end
	end

endmodule
