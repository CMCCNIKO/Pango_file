//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将授权的视频缓冲描述符转换为 64 位地址的 MWr 包命令，整包预取像素字，再依次发送图像、帧记录与最终提交序号。
// Module Name: pcie_video_mwr_engine
// File Name: source/pcie/pcie_video_mwr_engine.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将授权的视频缓冲描述符转换为 64 位地址的 MWr 包命令，整包预取像素字，再依次发送图像、帧记录与最终提交序号。
// 依据链路、enable/stop、generation 及采集复位中止旧事务；等待包完成和元数据握手后释放缓冲所有权。
//
// Dependencies: pcie_video_packet_bridge 的命令/载荷握手；pcie_video_capture 的流与元数据 token。
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
// Design Name: T04 video DMA
// Module Name: pcie_video_mwr_engine
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 整包预取、64位地址分包、帧记录与最后提交。
// Dependencies: pcie_video_packet_bridge; T03 capture metadata token protocol.
// Revision 0.01 - File Created
// Revision 0.02 - 2026-10-21: Free-running address registers isolate packet CE timing.
// Revision 0.03 - 2026-10-24: 空闲预清发送计数，移除 claim 到计数 CE 的长路径。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module pcie_video_mwr_engine #(
	parameter C_FRAME_BYTES = 1843200,
	parameter integer C_TRUST_VALIDATED_DESCRIPTOR = 0
)(
	input i_clk,
	input i_rst_n,
	input i_link_up,
	input i_enable,
	input i_stop,
	input [31:0] i_generation,
	input [2:0] i_cfg_max_payload_size,
	input i_candidate_valid,
	input [1:0] i_candidate_index,
	input [63:0] i_candidate_addr,
	input [31:0] i_candidate_capacity,
	input [31:0] i_candidate_generation,
	output o_claim,
	input i_stream_valid,
	input [127:0] i_stream_data,
	output o_stream_ready,
	input i_meta_valid,
	input [31:0] i_meta_frame_id,
	input [31:0] i_meta_byte_count,
	input [17:0] i_meta_word_count,
	input [31:0] i_meta_crc32,
	input i_meta_drop,
	input i_meta_error,
	input i_meta_ack_toggle,
	output o_meta_consume_ack_toggle,
	input i_capture_busy,
	input i_capture_flush_busy,
	input i_capture_flush_done,
	input i_capture_reset_n,
	output o_capture_buffer_ready,
	output o_packet_valid,
	output [63:0] o_packet_addr,
	output [9:0] o_packet_dw_count,
	input i_packet_ready,
	output o_payload_valid,
	output [127:0] o_payload_data,
	output o_payload_last,
	input i_payload_ready,
	input i_packet_done,
	input i_packet_busy,
	output o_busy,
	output o_flush_done,
	output o_complete,
	output [1:0] o_complete_index,
	output [31:0] o_complete_generation,
	output [31:0] o_complete_status,
	output [31:0] o_complete_frame_id,
	output [31:0] o_complete_length,
	output [31:0] o_complete_crc32
);
	localparam ST_IDLE=4'd0, ST_SELECT=4'd1, ST_LOAD=4'd2,
		ST_COMMAND=4'd3, ST_SEND=4'd4, ST_WAIT=4'd5,
		ST_RECORD=4'd6, ST_COMMIT=4'd7, ST_RELEASE=4'd8,
		ST_CANCEL=4'd9;
	localparam KIND_DATA=2'd0,KIND_RECORD=2'd1,KIND_COMMIT=2'd2;
	reg [3:0] state;
	reg [1:0] kind;
	reg [63:0] frame_addr;
	reg [63:0] reg_data_addr;
	reg [63:0] reg_record_addr;
	reg [63:0] reg_commit_addr;
	reg [31:0] generation,sent_bytes;
	reg [1:0] descriptor_index;
	reg [9:0] packet_dw_count;
	reg [3:0] packet_words,load_count,send_count;
	// Payload has no reset: ST_SEND is reached only after this packet is loaded.
	// State/count resets invalidate all old contents; no new RAM/IP is required.
	reg [127:0] prefetched [0:7];
	reg meta_seen,meta_bad,meta_token;
	reg [31:0] meta_frame_id,meta_crc32;
	reg cancelled,flush_seen;
	reg [31:0] commit_seq,pending_commit;
	reg flush_done_o,complete_o,meta_ack_o;
	reg [31:0] complete_status_o,complete_length_o;
	wire active = state!=ST_IDLE;
	wire abort_now = active && (!i_link_up || !i_capture_reset_n || !i_enable || i_stop || i_generation!=generation || i_cfg_max_payload_size>3'd5);
	wire new_bad_meta = i_meta_valid && (i_meta_drop || i_meta_error ||
		i_meta_byte_count!=C_FRAME_BYTES || i_meta_word_count!=(C_FRAME_BYTES/16));
	wire frame_bad = meta_bad || new_bad_meta;
	wire [63:0] next_addr = frame_addr + {32'd0,sent_bytes};
	wire [31:0] remaining_words = (C_FRAME_BYTES-sent_bytes)>>4;
	wire [8:0] boundary_words = 9'd256 - {1'b0,next_addr[11:4]};
	wire [3:0] bounded_words = remaining_words<8 ? remaining_words[3:0] : 4'd8;
	wire [3:0] next_words = boundary_words<bounded_words ? boundary_words[3:0] : bounded_words;
	// 仅当上游 ARM 已验证完整描述符时可设 1，移除同周期重复宽比较。
	// 默认 0 保留独立模块的地址、容量、代际及溢出防御检查。
	wire descriptor_checked = i_candidate_addr[3:0]==4'd0 &&
		i_candidate_capacity>=C_FRAME_BYTES+32 && i_candidate_generation==i_generation &&
		i_candidate_addr <= (64'hffffffffffffffff-C_FRAME_BYTES-31);
	wire legal_candidate = i_candidate_valid && i_cfg_max_payload_size<=3'd5 &&
		(C_TRUST_VALIDATED_DESCRIPTOR!=0 || descriptor_checked);
	assign o_claim=state==ST_IDLE && i_enable && !i_stop && i_link_up && i_capture_reset_n && !i_capture_busy && !i_capture_flush_busy && !i_meta_valid && !i_packet_busy && legal_candidate;
	assign o_meta_consume_ack_toggle=meta_ack_o;
	assign o_capture_buffer_ready=active && !cancelled && !abort_now && !meta_seen && !i_meta_valid &&
		state!=ST_CANCEL && i_capture_reset_n;
	assign o_stream_ready=state==ST_LOAD && !cancelled && !abort_now && !frame_bad;
	assign o_packet_valid=state==ST_COMMAND && !cancelled && !abort_now && (kind!=KIND_DATA || !frame_bad);
	assign o_packet_addr=kind==KIND_RECORD ? reg_record_addr :
		kind==KIND_COMMIT ? reg_commit_addr : reg_data_addr;
	assign o_packet_dw_count=packet_dw_count;
	assign o_payload_valid=state==ST_SEND && i_link_up;
	assign o_payload_data=prefetched[send_count[2:0]];
	assign o_payload_last=send_count==packet_words-1'b1;
	assign o_busy=state!=ST_IDLE;
	assign o_flush_done=flush_done_o;
	assign o_complete=complete_o;
	assign o_complete_index=descriptor_index;
	assign o_complete_generation=generation;
	assign o_complete_status=complete_status_o;
	assign o_complete_frame_id=meta_frame_id;
	assign o_complete_length=complete_length_o;
	assign o_complete_crc32=meta_crc32;
	// Address registers run without CE. SELECT/LOAD supplies the data-address
	// settling cycle; frame_addr and sent_bytes stay fixed while COMMAND stalls.
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n)begin
			reg_data_addr <= 64'd0;
			reg_record_addr <= 64'd0;
			reg_commit_addr <= 64'd0;
		end else begin
			reg_data_addr <= next_addr;
			reg_record_addr <= frame_addr + C_FRAME_BYTES;
			reg_commit_addr <= frame_addr + C_FRAME_BYTES + 28;
		end
	end
	// Preserve the control FSM's abort priority. Data/record/commit words are
	// written before ST_COMMAND, then stay stable for every stalled valid beat.
	always @(posedge i_clk) begin
		if(i_rst_n && !(abort_now && state!=ST_CANCEL &&
			(!i_link_up || (state!=ST_SEND && state!=ST_WAIT))))begin
			case(state)
				ST_LOAD:if(!frame_bad && i_stream_valid && o_stream_ready)
					prefetched[load_count[2:0]]<=i_stream_data;
				ST_RECORD:begin
					prefetched[0]<={meta_frame_id,generation,32'd1,32'h31564750};
					prefetched[1]<={32'd0,(frame_bad ? 32'd2:32'd1),meta_crc32,sent_bytes};
				end
				ST_COMMIT:prefetched[0]<={96'd0,(commit_seq==32'hffffffff ? 32'd1:commit_seq+1'b1)};
				default:begin end
			endcase
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			state<=ST_IDLE;kind<=KIND_DATA;frame_addr<=0;
			generation<=0;sent_bytes<=0;descriptor_index<=0;packet_dw_count<=0;
			packet_words<=0;load_count<=0;send_count<=0;
			meta_seen<=0;meta_bad<=0;meta_token<=0;meta_frame_id<=0;meta_crc32<=0;
			cancelled<=0;flush_seen<=0;commit_seq<=0;pending_commit<=0;
			flush_done_o<=0;complete_o<=0;meta_ack_o<=0;
			complete_status_o<=0;complete_length_o<=0;
		end else begin
			flush_done_o<=0;complete_o<=0;
			if(active && (i_capture_flush_done || !i_capture_reset_n))flush_seen<=1;
			if(active && i_meta_valid && !meta_seen)begin
				meta_seen<=1;meta_bad<=new_bad_meta;meta_token<=i_meta_ack_toggle;
				meta_frame_id<=i_meta_frame_id;meta_crc32<=i_meta_crc32;
			end
			if(abort_now)cancelled<=1;
			if(abort_now && state!=ST_CANCEL && (!i_link_up || (state!=ST_SEND && state!=ST_WAIT)))begin
				state<=ST_CANCEL;
			end else case(state)
				ST_IDLE:begin
					cancelled<=0;flush_seen<=0;
					// 空闲期没有有效包，下一次 claim 前即可清零旧帧计数。
					// 所有有效命令/记录仍从 claim 后的同一数值与同一拍开始。
					sent_bytes<=0;
					if(o_claim)begin
						descriptor_index<=i_candidate_index;frame_addr<=i_candidate_addr;
						generation<=i_candidate_generation;meta_seen<=0;meta_bad<=0;
						meta_frame_id<=0;meta_crc32<=0;state<=ST_SELECT;
					end
				end
				ST_SELECT:begin
					if(frame_bad && (meta_seen || i_meta_valid))state<=ST_RECORD;
					else if(sent_bytes==C_FRAME_BYTES)begin
						if(meta_seen)state<=ST_RECORD;
					end else begin
						packet_words<=next_words;
						packet_dw_count<={4'd0,next_words,2'b00};load_count<=0;kind<=KIND_DATA;state<=ST_LOAD;
					end
				end
				ST_LOAD:begin
					if(frame_bad)state<=ST_RECORD;
					else if(i_stream_valid && o_stream_ready)begin
						load_count<=load_count+1'b1;
						if(load_count==packet_words-1'b1)state<=ST_COMMAND;
					end
				end
				ST_COMMAND:begin
					if(kind==KIND_DATA && frame_bad)state<=ST_RECORD;
					else if(o_packet_valid && i_packet_ready)begin send_count<=0;state<=ST_SEND;end
				end
				ST_SEND:if(i_payload_ready && o_payload_valid)begin
					send_count<=send_count+1'b1;
					if(o_payload_last)state<=ST_WAIT;
				end
				ST_WAIT:if(i_packet_done)begin
					if(kind==KIND_DATA)sent_bytes<=sent_bytes+{20'd0,packet_dw_count,2'b00};
					if(cancelled || abort_now)state<=ST_CANCEL;
					else if(kind==KIND_DATA)state<=ST_SELECT;
					else if(kind==KIND_RECORD)state<=ST_COMMIT;
					else begin
						commit_seq<=pending_commit;complete_o<=1;meta_ack_o<=meta_token;state<=ST_RELEASE;
					end
				end
				ST_RECORD:begin
					packet_dw_count<=10'd7;packet_words<=4'd2;
					complete_status_o<=frame_bad ? 32'd2:32'd1;complete_length_o<=sent_bytes;
					kind<=KIND_RECORD;state<=ST_COMMAND;
				end
				ST_COMMIT:begin
					pending_commit<=commit_seq==32'hffffffff ? 32'd1:commit_seq+1'b1;
					packet_dw_count<=10'd1;packet_words<=4'd1;
					kind<=KIND_COMMIT;state<=ST_COMMAND;
				end
				ST_CANCEL:begin
					if(!i_packet_busy)begin
						if(meta_seen)meta_ack_o<=meta_token;
						if(flush_seen || i_capture_flush_done || !i_capture_reset_n)begin
							flush_done_o<=1;state<=ST_IDLE;
						end
					end
				end
				ST_RELEASE:if(!i_capture_busy && !i_capture_flush_busy && !i_meta_valid && !i_packet_busy)state<=ST_IDLE;
				default:state<=ST_CANCEL;
			endcase
		end
	end
endmodule






