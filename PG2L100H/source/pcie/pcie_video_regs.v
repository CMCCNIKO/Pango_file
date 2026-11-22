//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 实现 PCIe 视频 DMA 的 BAR1 控制、统计与四个缓冲描述符，校验 4KiB 对齐地址并管理 FREE/READY/HW/DONE/ERROR 所有权。
// Module Name: pcie_video_regs
// File Name: source/pcie/pcie_video_regs.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 实现 PCIe 视频 DMA 的 BAR1 控制、统计与四个缓冲描述符，校验 4KiB 对齐地址并管理 FREE/READY/HW/DONE/ERROR 所有权。
// 停止操作通过 generation 隔离旧事务，提供候选描述符、完成/错误/丢帧/溢出统计与寄存摘要。
//
// Dependencies: pcie_video_regs.vh；视频 DMA 引擎候选、claim 与完成接口。
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
// Create Date: 2026-10-07
// Design Name: PCIe 视频寄存器与缓冲所有权
// Module Name: pcie_video_regs
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 单 PCIe 时钟域；4KiB 对齐地址，四描述符，停止代际隔离。
// Dependencies: pcie_video_regs.vh
// Revision: 0.01 - File Created
//           0.02 - 2026-10-19 独立 CSR 接受条件、候选预译码及局部所有权更新。
//           0.03 - 2026-10-21 拒绝统计按字节分段，缩短命令接受条件后的进位链。
//           0.04 - 2026-10-21 完成、错误、丢帧和溢出统计按字节分段。
//           0.05 - 2026-10-28 ARM 通过有效位清除完成信息，降低宽校验后的清零扇出。
//           0.06 - 2026-10-28 预测下一所有权并同沿寄存 HW 摘要，隔离 DMA 仲裁组合链。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`include "pcie_video_regs.vh"
module pcie_video_regs
(
	input i_clk,
	input i_rst_n,
	input i_link_up,
	input i_busy,
	input i_legacy_busy,
	input i_flush_done,
	input i_legacy_reject_event,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [31:0] o_rd_data,
	output [127:0] o_rd_window_data,
	output o_enable,
	output o_stop,
	output o_rate_30,
	output [31:0] o_generation,
	output o_video_mode,
	output o_candidate_valid,
	output [1:0] o_candidate_index,
	output [63:0] o_candidate_addr,
	output [31:0] o_candidate_capacity,
	output [31:0] o_candidate_generation,
	input i_claim,
	input i_complete,
	input [1:0] i_complete_index,
	input [31:0] i_complete_generation,
	input [31:0] i_complete_status,
	input [31:0] i_complete_frame_id,
	input [31:0] i_complete_length,
	input [31:0] i_complete_crc32,
	input i_drop_event,
	input i_overflow_event
);
	reg enable_o;
	reg stop_o;
	reg rate_30_o;
	reg [31:0] generation_o;
	reg link_previous;
	reg [31:0] count_reject;
	reg [31:0] count_done;
	reg [31:0] count_error;
	reg [31:0] count_drop;
	reg [31:0] count_overflow;
	reg [63:0] desc_addr [0:3];
	reg [31:0] desc_capacity [0:3];
	reg desc_capacity_valid [0:3];
	// 已配置区间上界在地址/容量写入时锁存，避免 ARM 同拍重复执行四路 65 位加法。
	// ARM 只会在配置写入完成后的后续事务到达，因此该缓存与原子校验语义一致。
	reg [64:0] desc_end [0:3];
	reg [31:0] desc_generation [0:3];
	reg [2:0] desc_state [0:3];
	reg [31:0] desc_frame [0:3];
	reg [31:0] desc_length [0:3];
	reg [31:0] desc_crc [0:3];
	// 完成信息仅供 CSR 读取；ARM 同拍撤销有效位，避免宽区间校验驱动 96 位清零。
	reg desc_metadata_valid [0:3];
	wire candidate_valid_o;
	wire [1:0] candidate_index_o;
	wire candidate_allowed;
	wire [3:0] candidate_eligible;
	wire [3:0] candidate_select;
	wire [3:0] claim_select;
	wire [3:0] desc_hw_next;
	(* syn_preserve = 1 *) reg any_hw;
	reg any_ready;
	reg [127:0] rd_window_data_o;
	reg [31:0] read_word;
	reg [11:0] read_address;
	reg [1:0] read_number;
	integer read_lane;
	reg arm_valid;
	wire write_valid;
	reg [64:0] arm_end;
	reg [64:0] other_end;
	integer check_index;
	integer candidate_index;
	integer reset_index;
	integer update_index;
	genvar eligible_index;
	wire [1:0] write_index;
	wire descriptor_write;
	wire write_full;
	wire stop_request;
	wire invalidate;
	wire config_allowed;
	wire complete_valid;
	wire complete_success;
	wire write_active;
	wire write_start;
	wire write_clear_stats;
	wire write_generation;
	wire write_rate;
	wire write_desc_config;
	wire write_desc_low;
	wire write_desc_high;
	wire write_desc_capacity;
	wire write_desc_arm;
	wire write_desc_release;
	wire reject_csr_event;
	wire reject_any_event;
	wire reject_both_events;
	wire [2:0] reject_byte_carry;
	wire [2:0] done_byte_carry;
	wire [2:0] error_byte_carry;
	wire [2:0] drop_byte_carry;
	wire [2:0] overflow_byte_carry;
	assign write_index = i_wr_addr[6:5];
	assign descriptor_write = i_wr_addr >= 12'h500 && i_wr_addr < 12'h580;
	assign write_full = i_wr_en && i_wr_strb == 4'hf && i_wr_addr[1:0] == 2'b00;
	assign stop_request = write_full && i_wr_addr == `PGV_REG_COMMAND && i_wr_data == 32'd2;
	assign invalidate = stop_request || (link_previous && !i_link_up);
	assign config_allowed = !enable_o && !i_busy && !any_hw;
	assign complete_valid = i_complete && desc_state[i_complete_index] == `PGV_STATE_HW &&
		desc_generation[i_complete_index] == i_complete_generation;
	assign complete_success = i_complete_status == 32'd1 && i_complete_length == `PGV_FRAME_BYTES &&
		i_complete_generation == generation_o && i_link_up && !invalidate;
	// 各操作独立接受，普通配置写入不再经过 ARM 的宽区间比较。
	assign write_active = write_full && !invalidate;
	assign write_start = write_active && i_wr_addr == `PGV_REG_COMMAND && i_wr_data == 32'd1 &&
		config_allowed && i_link_up && !i_legacy_busy && generation_o != 32'hffffffff;
	assign write_clear_stats = write_active && i_wr_addr == `PGV_REG_COMMAND && i_wr_data == 32'd4 && config_allowed;
	assign write_generation = write_active && i_wr_addr == `PGV_REG_GENERATION &&
		config_allowed && !any_ready && i_wr_data > generation_o;
	assign write_rate = write_active && i_wr_addr == `PGV_REG_RATE30 && config_allowed && i_wr_data <= 32'd1;
	assign write_desc_config = write_active && descriptor_write && config_allowed && desc_state[write_index] == `PGV_STATE_FREE;
	assign write_desc_low = write_desc_config && i_wr_addr[4:0] == 5'h00;
	assign write_desc_high = write_desc_config && i_wr_addr[4:0] == 5'h04;
	assign write_desc_capacity = write_desc_config && i_wr_addr[4:0] == 5'h08;
	assign write_desc_arm = write_active && descriptor_write && i_wr_addr[4:0] == 5'h10 && i_wr_data == 32'd1 &&
		desc_state[write_index] == `PGV_STATE_FREE && arm_valid && generation_o != 32'hffffffff;
	assign write_desc_release = write_active && descriptor_write && i_wr_addr[4:0] == 5'h10 && i_wr_data == 32'd2 &&
		(desc_state[write_index] == `PGV_STATE_DONE || desc_state[write_index] == `PGV_STATE_ERROR);
	// 汇总值只用于拒绝计数；STOP 即使触发 invalidate 仍被接受。
	assign write_valid = stop_request || write_start || write_clear_stats || write_generation || write_rate ||
		write_desc_low || write_desc_high || write_desc_capacity || write_desc_arm || write_desc_release;
	assign reject_csr_event = i_wr_en && !write_valid;
	assign reject_any_event = reject_csr_event || i_legacy_reject_event;
	assign reject_both_events = reject_csr_event && i_legacy_reject_event;
	// 每拍最多加二；各高字节只需旧低位前缀溢出的单次进位。
	assign reject_byte_carry[0] = (reject_any_event && (&count_reject[7:0])) ||
		(reject_both_events && count_reject[7:0] == 8'hfe);
	assign reject_byte_carry[1] = (reject_any_event && (&count_reject[15:0])) ||
		(reject_both_events && count_reject[15:0] == 16'hfffe);
	assign reject_byte_carry[2] = (reject_any_event && (&count_reject[23:0])) ||
		(reject_both_events && count_reject[23:0] == 24'hfffffe);
	// 前缀进位与事件并行译码，清统计不再串过整个 32 位计数使能链。
	assign done_byte_carry = {(&count_done[23:0]),(&count_done[15:0]),(&count_done[7:0])};
	assign error_byte_carry = {(&count_error[23:0]),(&count_error[15:0]),(&count_error[7:0])};
	assign drop_byte_carry = {(&count_drop[23:0]),(&count_drop[15:0]),(&count_drop[7:0])};
	assign overflow_byte_carry = {(&count_overflow[23:0]),(&count_overflow[15:0]),(&count_overflow[7:0])};
	assign o_enable = enable_o;
	assign o_stop = stop_o;
	assign o_rate_30 = rate_30_o;
	assign o_generation = generation_o;
	assign o_video_mode = enable_o || any_hw || i_busy;
	assign o_candidate_valid = candidate_valid_o;
	assign o_candidate_index = candidate_index_o;
	assign o_candidate_addr = desc_addr[candidate_index_o];
	assign o_candidate_capacity = desc_capacity[candidate_index_o];
	assign o_candidate_generation = desc_generation[candidate_index_o];

	// READY 与代际比较并行完成；全局 STOP/掉链/flush 仅在选择结果末端门控。
	// 内部保留未门控 onehot，避免 STOP 经输出 index 选择再返回所有权状态。
	generate
		for(eligible_index = 0; eligible_index < 4; eligible_index = eligible_index + 1)begin:g_candidate_eligible
			assign candidate_eligible[eligible_index] = desc_state[eligible_index] == `PGV_STATE_READY &&
				desc_generation[eligible_index] == generation_o;
			// 与下方所有权更新同沿：HW 只在 flush/有效完成时退出，READY 只在合法 claim 时进入。
			assign desc_hw_next[eligible_index] =
				(desc_state[eligible_index] == `PGV_STATE_HW && !i_flush_done &&
					!(complete_valid && i_complete_index == eligible_index)) ||
				(desc_state[eligible_index] == `PGV_STATE_READY && !invalidate && claim_select[eligible_index]);
		end
	endgenerate
	assign candidate_select[0] = candidate_eligible[0];
	assign candidate_select[1] = candidate_eligible[1] && !candidate_eligible[0];
	assign candidate_select[2] = candidate_eligible[2] && !(|candidate_eligible[1:0]);
	assign candidate_select[3] = candidate_eligible[3] && !(|candidate_eligible[2:0]);
	assign candidate_allowed = enable_o && i_link_up && !invalidate && !i_flush_done;
	assign candidate_valid_o = candidate_allowed && (|candidate_eligible);
	// 无候选时 index 仍为零，与原接口的空闲读出值保持一致。
	assign candidate_index_o = candidate_allowed ?
		{(candidate_select[2] || candidate_select[3]),(candidate_select[1] || candidate_select[3])} : 2'd0;
	assign claim_select = candidate_select & {4{i_claim && candidate_allowed}};

	// 预测的是本沿更新后的状态，摘要与 desc_state 始终同拍，不给模式或配置接受增加延迟。
	// 保留独立 FF，避免 HW 解码再经视频仲裁与官方 DMA 地址 CE 串成跨层长路径。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			any_hw <= 1'b0;
		end else begin
			any_hw <= |desc_hw_next;
		end
	end

	always@(*)begin
		any_ready = 1'b0;
		for(candidate_index = 0; candidate_index < 4; candidate_index = candidate_index + 1)begin
			if(desc_state[candidate_index] == `PGV_STATE_READY)begin
				any_ready = 1'b1;
			end
		end
	end

	// 地址配置可分高低 DWORD；ARM 才原子校验完整地址和所有权区间。
	always@(*)begin
		arm_end = desc_end[write_index];
		other_end = 65'd0;
		arm_valid = desc_addr[write_index] != 64'd0 && desc_addr[write_index][11:0] == 12'd0 &&
			desc_capacity_valid[write_index] && !arm_end[64];
		for(check_index = 0; check_index < 4; check_index = check_index + 1)begin
			other_end = desc_end[check_index];
			if(check_index != write_index && desc_state[check_index] != `PGV_STATE_FREE &&
				{1'b0, desc_addr[write_index]} < other_end && {1'b0, desc_addr[check_index]} < arm_end)begin
				arm_valid = 1'b0;
			end
		end
	end

	// 一组所有权寄存器共用过程，以明确定义命令与完成的同拍优先级。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			enable_o <= 1'b0;
			stop_o <= 1'b0;
			rate_30_o <= 1'b0;
			generation_o <= 32'd1;
			link_previous <= 1'b0;
			for(reset_index = 0; reset_index < 4; reset_index = reset_index + 1)begin
				desc_addr[reset_index] <= 64'd0;
				desc_capacity[reset_index] <= 32'd0;
				desc_capacity_valid[reset_index] <= 1'b0;
				desc_end[reset_index] <= 65'd0;
				desc_generation[reset_index] <= 32'd0;
				desc_state[reset_index] <= `PGV_STATE_FREE;
				desc_frame[reset_index] <= 32'd0;
				desc_length[reset_index] <= 32'd0;
				desc_crc[reset_index] <= 32'd0;
				desc_metadata_valid[reset_index] <= 1'b0;
			end
		end else begin
			link_previous <= i_link_up;
			stop_o <= invalidate;
			if(complete_valid)begin
				desc_frame[i_complete_index] <= i_complete_frame_id;
				desc_length[i_complete_index] <= i_complete_length;
				desc_crc[i_complete_index] <= i_complete_crc32;
				desc_metadata_valid[i_complete_index] <= 1'b1;
			end
			if(write_start)begin
				enable_o <= 1'b1;
			end
			if(write_generation)begin
				generation_o <= i_wr_data;
			end
			if(write_rate)begin
				rate_30_o <= i_wr_data[0];
			end
			if(write_desc_config)begin
					case(i_wr_addr[4:0])
						5'h00:begin
							desc_addr[write_index][31:0] <= i_wr_data;
							desc_end[write_index] <= {1'b0,desc_addr[write_index][63:32],i_wr_data} + {33'd0,desc_capacity[write_index]};
						end
						5'h04:begin
							desc_addr[write_index][63:32] <= i_wr_data;
							desc_end[write_index] <= {1'b0,i_wr_data,desc_addr[write_index][31:0]} + {33'd0,desc_capacity[write_index]};
						end
						5'h08:begin
							desc_capacity[write_index] <= i_wr_data;
							desc_capacity_valid[write_index] <= i_wr_data >= `PGV_BUFFER_BYTES && i_wr_data[11:0] == 12'd0;
							desc_end[write_index] <= {1'b0,desc_addr[write_index]} + {33'd0,i_wr_data};
						end
						default:begin end
					endcase
			end
			if(write_desc_arm)begin
				desc_generation[write_index] <= generation_o;
				desc_metadata_valid[write_index] <= 1'b0;
			end
			if(invalidate)begin
				enable_o <= 1'b0;
				if(generation_o != 32'hffffffff)begin
					generation_o <= generation_o + 32'd1;
				end
			end
			// 事件均按旧状态判定：FREE 只接受 ARM，READY 只接受 claim/失效，
			// HW 只接受完成/flush，DONE/ERROR 只接受 release；各类无串行优先链。
			// 唯一重叠的 HW 完成与 flush 保留 flush 优先；READY 保留失效优先。
			for(update_index = 0; update_index < 4; update_index = update_index + 1)begin
				case(desc_state[update_index])
					`PGV_STATE_FREE:begin
						if(write_desc_arm && write_index == update_index)begin
							desc_state[update_index] <= `PGV_STATE_READY;
						end
					end
					`PGV_STATE_READY:begin
						if(invalidate)begin
							desc_state[update_index] <= `PGV_STATE_ERROR;
						end else if(claim_select[update_index])begin
							desc_state[update_index] <= `PGV_STATE_HW;
						end
					end
					`PGV_STATE_HW:begin
						if(i_flush_done)begin
							desc_state[update_index] <= `PGV_STATE_ERROR;
						end else if(complete_valid && i_complete_index == update_index)begin
							desc_state[update_index] <= complete_success ? `PGV_STATE_DONE : `PGV_STATE_ERROR;
						end
					end
					`PGV_STATE_DONE,`PGV_STATE_ERROR:begin
						if(write_desc_release && write_index == update_index)begin
							desc_state[update_index] <= `PGV_STATE_FREE;
						end
					end
					default:begin end
				endcase
			end
		end
	end

	// 统计按事件计数；同拍 CSR 拒绝和 legacy 拒绝均计入。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			count_reject <= 32'd0;
			count_done <= 32'd0;
			count_error <= 32'd0;
			count_drop <= 32'd0;
			count_overflow <= 32'd0;
		end else if(write_clear_stats)begin
			count_reject <= 32'd0;
			count_done <= 32'd0;
			count_error <= 32'd0;
			count_drop <= 32'd0;
			count_overflow <= 32'd0;
		end else begin
			if(reject_both_events)begin
				count_reject[7:0] <= count_reject[7:0] + 8'd2;
			end else if(reject_any_event)begin
				count_reject[7:0] <= count_reject[7:0] + 8'd1;
			end
			if(reject_byte_carry[0])begin
				count_reject[15:8] <= count_reject[15:8] + 8'd1;
			end
			if(reject_byte_carry[1])begin
				count_reject[23:16] <= count_reject[23:16] + 8'd1;
			end
			if(reject_byte_carry[2])begin
				count_reject[31:24] <= count_reject[31:24] + 8'd1;
			end
			if(complete_valid && !i_flush_done)begin
				if(complete_success)begin
					count_done[7:0] <= count_done[7:0] + 8'd1;
					if(done_byte_carry[0])begin
						count_done[15:8] <= count_done[15:8] + 8'd1;
					end
					if(done_byte_carry[1])begin
						count_done[23:16] <= count_done[23:16] + 8'd1;
					end
					if(done_byte_carry[2])begin
						count_done[31:24] <= count_done[31:24] + 8'd1;
					end
				end else begin
					count_error[7:0] <= count_error[7:0] + 8'd1;
					if(error_byte_carry[0])begin
						count_error[15:8] <= count_error[15:8] + 8'd1;
					end
					if(error_byte_carry[1])begin
						count_error[23:16] <= count_error[23:16] + 8'd1;
					end
					if(error_byte_carry[2])begin
						count_error[31:24] <= count_error[31:24] + 8'd1;
					end
				end
			end
			if(i_drop_event)begin
				count_drop[7:0] <= count_drop[7:0] + 8'd1;
			end
			if(i_drop_event && drop_byte_carry[0])begin
				count_drop[15:8] <= count_drop[15:8] + 8'd1;
			end
			if(i_drop_event && drop_byte_carry[1])begin
				count_drop[23:16] <= count_drop[23:16] + 8'd1;
			end
			if(i_drop_event && drop_byte_carry[2])begin
				count_drop[31:24] <= count_drop[31:24] + 8'd1;
			end
			if(i_overflow_event)begin
				count_overflow[7:0] <= count_overflow[7:0] + 8'd1;
			end
			if(i_overflow_event && overflow_byte_carry[0])begin
				count_overflow[15:8] <= count_overflow[15:8] + 8'd1;
			end
			if(i_overflow_event && overflow_byte_carry[1])begin
				count_overflow[23:16] <= count_overflow[23:16] + 8'd1;
			end
			if(i_overflow_event && overflow_byte_carry[2])begin
				count_overflow[31:24] <= count_overflow[31:24] + 8'd1;
			end
		end
	end

	// 四路组合 DWORD 解码直接读取状态，避免 Verilog function 隐藏敏感信号。
	always@(*)begin
		rd_window_data_o = 128'd0;
		read_word = 32'd0;
		read_address = 12'd0;
		read_number = 2'd0;
		for(read_lane = 0; read_lane < 4; read_lane = read_lane + 1)begin
			read_address = {i_rd_addr[11:4],read_lane[1:0],2'b00};
			read_number = read_address[6:5];
			read_word = 32'd0;
			case(read_address)
				`PGV_REG_MAGIC:begin read_word = `PGV_MAGIC; end
				`PGV_REG_VERSION:begin read_word = `PGV_VERSION; end
				`PGV_REG_STATUS:begin read_word = {27'd0,any_hw,o_video_mode,i_link_up,i_busy,enable_o}; end
				`PGV_REG_GENERATION:begin read_word = generation_o; end
				`PGV_REG_REJECT:begin read_word = count_reject; end
				`PGV_REG_DONE:begin read_word = count_done; end
				`PGV_REG_ERROR:begin read_word = count_error; end
				`PGV_REG_DROP:begin read_word = count_drop; end
				`PGV_REG_OVERFLOW:begin read_word = count_overflow; end
				`PGV_REG_RATE30:begin read_word = {31'd0,rate_30_o}; end
				default:begin
					if(read_address >= 12'h500 && read_address < 12'h580)begin
						case(read_address[4:0])
							5'h00:begin read_word = desc_addr[read_number][31:0]; end
							5'h04:begin read_word = desc_addr[read_number][63:32]; end
							5'h08:begin read_word = desc_capacity[read_number]; end
							5'h0c:begin read_word = desc_generation[read_number]; end
							5'h10:begin read_word = {29'd0,desc_state[read_number]}; end
							5'h14:begin read_word = desc_metadata_valid[read_number] ? desc_frame[read_number] : 32'd0; end
							5'h18:begin read_word = desc_metadata_valid[read_number] ? desc_length[read_number] : 32'd0; end
							5'h1c:begin read_word = desc_metadata_valid[read_number] ? desc_crc[read_number] : 32'd0; end
							default:begin read_word = 32'd0; end
						endcase
					end
				end
			endcase
			rd_window_data_o[read_lane*32+:32] = read_word;
		end
	end
	assign o_rd_window_data = rd_window_data_o;
	assign o_rd_data = i_rd_addr[1:0] == 2'b00 ? rd_window_data_o[i_rd_addr[3:2]*32+:32] : 32'd0;
endmodule
