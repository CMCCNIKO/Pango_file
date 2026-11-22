//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 缓存算法控制 accepted/applied 状态快照并按 UDP 发送握手提供 v1/v2 ACK。
// Module Name: algo_status_queue_v2
// File Name: source/video/control_v2/algo_status_queue_v2.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 缓存算法控制 accepted/applied 状态快照并按 UDP 发送握手提供 v1/v2 ACK。
// 采用四项浅寄存队列，满时保持稳定队头并淘汰最旧 pending，以便保留最新事件而不阻塞模式命令。
//
// Dependencies: algorithm_control_v2 状态事件及变量长度 UDP 发送仲裁。
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
// Create Date: 2026-10-10
// Design Name: HDMI Algorithm Control Status
// Module Name: algo_status_queue_v2
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 四项浅寄存队列；ACK积压不阻塞模式命令。
//              满时保留稳定队头，淘汰最旧pending以保存最新accepted/applied。
// Dependencies: algorithm_control_v2、变量长度UDP仲裁
// Revision: 0.01 - 基于现有浅寄存队列拓宽为v1/v2快照，不修改原模块
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module algo_status_queue_v2
(
	input i_clk,
	input i_rst_n,
	input i_accepted,
	input i_applied,
	input i_rejected,
	input [7:0] i_error_code,
	input [15:0] i_sequence,
	input [15:0] i_event_sequence,
	input [7:0] i_version,
	input [7:0] i_opcode,
	input [63:0] i_config,
	input [7:0] i_event_version,
	input [7:0] i_event_opcode,
	input [63:0] i_active_config,
	output o_cmd_ready,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	input i_ack_ready,
	output [31:0] o_dropped_status_count
);
	// 仅516-bit浅寄存队列，不引入异步存储或新的IP接口。
	reg [128:0] reg_headers [0:3];
	reg [128:0] next_headers [0:3];
	reg [128:0] event_headers [0:1];
	reg [2:0] cnt_entries;
	reg [2:0] cnt_entries_next;
	reg flag_reserved;
	reg flag_reserved_next;
	reg [31:0] dropped_status_count_o;
	wire flag_pop;
	wire [128:0] header_front;
	reg [1:0] cnt_push;
	reg [1:0] cnt_dropped;
	integer cnt_index;
	integer cnt_event;
	integer cnt_store;
	reg [15:0] crc_front;
	reg [15:0] crc_byte_next;
	reg [128:0] reg_crc_header;
	reg [127:0] reg_crc_shift;
	reg [4:0] cnt_crc_remaining;
	reg flag_crc_busy;
	reg ack_valid_o;
	reg [143:0] ack_payload_o;
	reg [15:0] ack_length_o;
	integer cnt_bit;

	assign o_cmd_ready = !flag_reserved;
	assign o_ack_valid = ack_valid_o;
	assign flag_pop = o_ack_valid && i_ack_ready;
	assign header_front = reg_headers[0];
	assign o_ack_payload = ack_payload_o;
	assign o_ack_length = ack_length_o;
	assign o_dropped_status_count = dropped_status_count_o;

	reg [1:0] dec_mode;
	always@(*)begin
		case(i_config[63:56])
			8'h04:begin dec_mode = 2'd1; end
			8'h02:begin dec_mode = 2'd2; end
			8'h08:begin dec_mode = 2'd3; end
			default:begin dec_mode = 2'd0; end
		endcase
	end
	// 状态包: A5 5A 01 event error mode seq_hi seq_lo threshold_hi threshold_lo CRC16。
	// accepted=81, applied=82, rejected=E1；拒绝包mode/threshold为0，无状态含义。
	always@(*)begin
		for(cnt_index = 0; cnt_index < 4; cnt_index = cnt_index + 1)begin
			next_headers[cnt_index] = reg_headers[cnt_index];
		end
		event_headers[0] = 129'd0;
		event_headers[1] = 129'd0;
		cnt_push = 2'd0;
		cnt_dropped = 2'd0;
		cnt_entries_next = cnt_entries;
		flag_reserved_next = flag_reserved;
		// Pop first; a stalled head must never be evicted.
		if(flag_pop)begin
			for(cnt_index = 0; cnt_index < 3; cnt_index = cnt_index + 1)begin
				next_headers[cnt_index] = reg_headers[cnt_index + 1];
			end
			cnt_entries_next = cnt_entries_next - 1'b1;
		end
		if(i_accepted)begin
			if(!flag_reserved)begin
				event_headers[0] = i_version == 8'd2 ? {1'b1,16'ha55a,8'd2,(i_opcode | 8'h80),8'h81,8'd0,i_config[63:16],i_sequence,i_config[15:0]} : {1'b0,16'ha55a,8'd1,8'h81,8'd0,6'd0,dec_mode,i_sequence,i_config[15:0],48'd0};
				cnt_push = 2'd1;
				flag_reserved_next = 1'b1;
			end else begin
				cnt_dropped = cnt_dropped + 1'b1;
			end
		end
		if(i_applied)begin
			if(flag_reserved_next)begin
				event_headers[cnt_push] = i_version == 8'd2 ? {1'b1,16'ha55a,8'd2,(i_opcode | 8'h80),8'h82,8'd0,i_config[63:16],i_sequence,i_config[15:0]} : {1'b0,16'ha55a,8'd1,8'h82,8'd0,6'd0,dec_mode,i_sequence,i_config[15:0],48'd0};
				cnt_push = cnt_push + 1'b1;
				flag_reserved_next = 1'b0;
			end else begin
				cnt_dropped = cnt_dropped + 1'b1;
			end
		end
		// Preserve the head and newest normal events; evict pending index1.
		for(cnt_event = 0; cnt_event < 2; cnt_event = cnt_event + 1)begin
			if(cnt_event < cnt_push)begin
				if(cnt_entries_next == 3'd4)begin
					next_headers[1] = next_headers[2];
					next_headers[2] = next_headers[3];
					cnt_entries_next = 3'd3;
					cnt_dropped = cnt_dropped + 1'b1;
				end
				next_headers[cnt_entries_next] = event_headers[cnt_event];
				cnt_entries_next = cnt_entries_next + 1'b1;
			end
		end
		if(i_rejected)begin
			if(cnt_push < 2 && cnt_entries_next < 3'd4)begin
				next_headers[cnt_entries_next] = i_event_version == 8'd2 ? {1'b1,16'ha55a,8'd2,(i_event_opcode | 8'h80),8'he1,i_error_code,i_active_config[63:16],i_event_sequence,i_active_config[15:0]} : {1'b0,16'ha55a,8'd1,8'he1,i_error_code,8'd0,i_event_sequence,16'd0,48'd0};
				cnt_entries_next = cnt_entries_next + 1'b1;
			end else begin
				cnt_dropped = cnt_dropped + 1'b1;
			end
		end
	end

	// 每拍处理一个字节，切断旧的整包组合CRC至公共ACK仲裁关键路径。
	// 计算期间队头仍保留在原队列，只有真实ACK握手才pop，容量及淘汰语义不变。
	always@(*)begin
		crc_byte_next = crc_front ^ {reg_crc_shift[127:120],8'd0};
		for(cnt_bit=0;cnt_bit<8;cnt_bit=cnt_bit+1)begin
			if(crc_byte_next[15])begin crc_byte_next=(crc_byte_next<<1)^16'h1021;end
			else begin crc_byte_next=crc_byte_next<<1;end
		end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			crc_front<=16'hffff;reg_crc_header<=0;reg_crc_shift<=0;
			cnt_crc_remaining<=0;flag_crc_busy<=0;ack_valid_o<=0;
			ack_payload_o<=0;ack_length_o<=0;
		end else begin
			if(flag_pop)begin ack_valid_o<=0;end
			if(!flag_crc_busy && !ack_valid_o && cnt_entries!=0)begin
				reg_crc_header<=header_front;reg_crc_shift<=header_front[127:0];
				cnt_crc_remaining<=header_front[128] ? 16 : 10;
				crc_front<=16'hffff;flag_crc_busy<=1;
			end else if(flag_crc_busy)begin
				crc_front<=crc_byte_next;reg_crc_shift<={reg_crc_shift[119:0],8'd0};
				cnt_crc_remaining<=cnt_crc_remaining-1'b1;
				if(cnt_crc_remaining==1)begin
					ack_payload_o<=reg_crc_header[128] ? {reg_crc_header[127:0],crc_byte_next} : {reg_crc_header[127:48],crc_byte_next,48'd0};
					ack_length_o<=reg_crc_header[128] ? 18 : 12;
					ack_valid_o<=1;flag_crc_busy<=0;
				end
			end
		end
	end

	// Valid count protects uninitialized register storage.
	always@(posedge i_clk)begin
		if(i_rst_n)begin
			for(cnt_store = 0; cnt_store < 4; cnt_store = cnt_store + 1)begin
				reg_headers[cnt_store] <= next_headers[cnt_store];
			end
		end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			cnt_entries <= 3'd0;
			flag_reserved <= 1'b0;
			dropped_status_count_o <= 32'd0;
		end else begin
			cnt_entries <= cnt_entries_next;
			flag_reserved <= flag_reserved_next;
			dropped_status_count_o <= dropped_status_count_o + {30'd0, cnt_dropped};
		end
	end
endmodule
