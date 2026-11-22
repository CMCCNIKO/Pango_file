//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: LU 协议的 UDP 在线 LUT 装载器，跨网络/DDR 域保存单包描述符，将有效 payload 累积为小端 256-bit DDR 单拍写。
// Module Name: avm_lut_udp_loader
// File Name: source/avm/avm_lut_udp_loader.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// LU 协议的 UDP 在线 LUT 装载器，跨网络/DDR 域保存单包描述符，将有效 payload 累积为小端 256-bit DDR 单拍写。
// BEGIN 立即撤销 LUT 有效，END 只有末尾真实 AW/W 接受后才置 ready；事务须使用共同 board reset，不可因像素 PLL 复位撤销已接受 AW。
//
// Dependencies: ddr_write_owner_arbiter 的 LUT 写客户端；浅控制寄存器，不依赖存储 IP。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 原修订中的“尚未顶层接入”为历史状态；当前 avm_monitor_top 已在 UDP 备用生成分支实例化。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-13
// Module Name: avm_lut_udp_loader
// Project Name: PG2L100H HDMI AVM Monitor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: LU固定LUT loader。单包FF mailbox，24byte累积为256bit单beat。
// BEGIN立即撤销LUT有效；END仅真实AW/W最后接受后置有效。无hash/CRC。
// Contract: net/DD复位从共同board reset同步释放；不得用PLL reset取消已接受AW。
// HMIC地址单位32bit word；写仲裁必须锁定AW至W接受，不混入quad完成事件。
// Dependencies: 无RAM/FIFO IP，仅浅控制FF。
// Revision: 0.01 - 独立模块，尚未顶层接入。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module avm_lut_udp_loader
#(
	parameter [27:0] C_LUT_BASE = 28'h0300000,
	parameter [31:0] C_LUT_BYTES = 32'd4915200
)
(
	input i_net_clk,
	input i_net_rst_n,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	output o_ack_valid,
	output [143:0] o_ack_payload,
	output [15:0] o_ack_length,
	input i_ack_ready,
	output [31:0] o_dropped_packet_count,
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_lut_in_use,
	output o_upload_active,
	output o_update_pending,
	output o_lut_ready,
	output [15:0] o_upload_sequence,
	output [31:0] o_next_offset,
	output [31:0] o_committed_bytes,
	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	input i_axi_awready,
	output [255:0] o_axi_wdata,
	output [31:0] o_axi_wstrb,
	output o_axi_wvalid,
	input i_axi_wready
);
	localparam [2:0] ST_IDLE=0, ST_EVAL=1, ST_AW=2, ST_W=3, ST_RESPONSE=4;
	reg [255:0] reg_packet;
	reg [15:0] cnt_packet;
	reg [15:0] reg_packet_length;
	reg [255:0] reg_completed_packet;
	reg [15:0] reg_completed_count;
	reg flag_completed_error;
	reg flag_receiving,flag_rx_error,flag_parse,flag_mailbox_busy;
	reg [288:0] reg_mailbox;
	reg flag_request_toggle;
	reg [1:0] reg_response_sync;
	reg flag_response_seen;
	reg ack_valid_o;
	reg [127:0] ack_payload_o;
	reg [15:0] reg_net_sequence;
	reg [31:0] reg_net_next,reg_net_committed,dropped_packet_count_o;
	reg [1:0] reg_request_sync;
	reg flag_request_seen;
	reg [2:0] state_current;
	reg [255:0] reg_command;
	reg [15:0] reg_command_count,reg_command_length;
	reg flag_command_error;
	reg [127:0] reg_response;
	reg flag_response_toggle;
	reg [7:0] reg_response_opcode,reg_response_event,reg_response_error;
	reg [15:0] reg_response_sequence;
	reg upload_active_o,lut_ready_o,flag_session;
	reg [15:0] upload_sequence_o;
	reg [31:0] next_offset_o,committed_bytes_o;
	reg [191:0] reg_carry,reg_last_payload;
	reg [5:0] cnt_carry;
	reg [31:0] reg_last_offset;
	reg flag_last_data;
	reg [255:0] reg_write_data;
	reg [7:0] dec_net_opcode;
	reg [15:0] dec_net_sequence;
	reg [7:0] dec_opcode,dec_version,dec_error;
	reg [15:0] dec_magic,dec_sequence,dec_reserved;
	reg [31:0] dec_base,dec_length,dec_offset;
	reg [191:0] dec_payload;
	reg [383:0] dec_accumulator;
	reg [6:0] dec_total;
	integer byte_index;
	wire flag_response_pending;
	wire flag_submit;
	assign flag_response_pending=reg_response_sync[1]!=flag_response_seen;
	assign flag_submit=flag_parse && !ack_valid_o && !flag_response_pending;
	assign o_ack_valid=ack_valid_o;
	assign o_ack_payload={ack_payload_o,16'd0};
	assign o_ack_length=16'd16;
	assign o_dropped_packet_count=dropped_packet_count_o;
	assign o_upload_active=upload_active_o;
	assign o_update_pending=state_current==ST_EVAL && dec_opcode==1;
	assign o_lut_ready=lut_ready_o;
	assign o_upload_sequence=upload_sequence_o;
	assign o_next_offset=next_offset_o;
	assign o_committed_bytes=committed_bytes_o;
	assign o_axi_awaddr=C_LUT_BASE+(committed_bytes_o>>2);
	assign o_axi_awlen=4'd0;
	assign o_axi_awvalid=state_current==ST_AW;
	assign o_axi_wdata=reg_write_data;
	assign o_axi_wstrb=32'hffffffff;
	assign o_axi_wvalid=state_current==ST_W;
	// 完整请求与完整响应均保持到下一握手，禁止跨域逐字段同步。
	always@(posedge i_net_clk or negedge i_net_rst_n)begin
		if(!i_net_rst_n)begin
			reg_packet<=0;cnt_packet<=0;reg_packet_length<=0;flag_receiving<=0;
			reg_completed_packet<=0;reg_completed_count<=0;flag_completed_error<=0;
			flag_rx_error<=0;flag_parse<=0;
		end else begin
			if(flag_submit)begin flag_parse<=0;end
			if(i_start)begin
				reg_packet<=i_valid ? {248'd0,i_data} : 256'd0;cnt_packet<=i_valid ? 1 : 0;
				flag_receiving<=1;flag_rx_error<=i_error;
			end else if(flag_receiving)begin
				if(i_valid)begin reg_packet<={reg_packet[247:0],i_data};if(cnt_packet!=16'hffff)begin cnt_packet<=cnt_packet+1'b1;end end
				if(i_error)begin flag_rx_error<=1;end
			end
			if(i_end)begin
				flag_receiving<=0;
				// 待发送完整包独立保持；下一包RX不能撕裂它或提前提交半包。
				if(!flag_parse || flag_submit)begin
					flag_parse<=1;reg_packet_length<=i_length;
					reg_completed_packet<=i_valid ? {reg_packet[247:0],i_data} : reg_packet;
					reg_completed_count<=cnt_packet+(i_valid ? 1'b1 : 1'b0);
					flag_completed_error<=flag_rx_error || i_error || !flag_receiving;
				end
			end
		end
	end
	always@(*)begin
		dec_net_opcode=0;dec_net_sequence=reg_net_sequence;
		if(reg_completed_count==16)begin dec_net_opcode=reg_completed_packet[103:96];dec_net_sequence=reg_completed_packet[31:16];end
		else if(reg_completed_count==32)begin dec_net_opcode=reg_completed_packet[231:224];end
		else if(reg_completed_count==8)begin dec_net_opcode=reg_completed_packet[39:32];dec_net_sequence=reg_completed_packet[31:16];end
	end
	always@(posedge i_net_clk or negedge i_net_rst_n)begin
		if(!i_net_rst_n)begin
			reg_mailbox<=0;flag_request_toggle<=0;flag_mailbox_busy<=0;
			reg_response_sync<=0;flag_response_seen<=0;ack_valid_o<=0;ack_payload_o<=0;
			reg_net_sequence<=0;reg_net_next<=0;reg_net_committed<=0;dropped_packet_count_o<=0;
		end else begin
			reg_response_sync<={reg_response_sync[0],flag_response_toggle};
			if(ack_valid_o && i_ack_ready)begin ack_valid_o<=0;end
			if(flag_response_pending && !ack_valid_o)begin
				ack_payload_o<=reg_response;ack_valid_o<=1;flag_response_seen<=reg_response_sync[1];flag_mailbox_busy<=0;
				if(reg_response[71:64]==0)begin reg_net_sequence<=reg_response[95:80];end
				reg_net_next<=reg_response[63:32];reg_net_committed<=reg_response[31:0];
			end else if(flag_submit)begin
				if(flag_mailbox_busy)begin
					ack_payload_o<={16'h4c55,8'd1,(dec_net_opcode | 8'h80),dec_net_sequence,8'd3,8'd6,reg_net_next,reg_net_committed};ack_valid_o<=1;
				end else begin
					reg_mailbox<={flag_completed_error,reg_completed_count,reg_packet_length,reg_completed_packet};flag_request_toggle<=!flag_request_toggle;flag_mailbox_busy<=1;
				end
			end
			if(i_end && flag_parse && !flag_submit)begin dropped_packet_count_o<=dropped_packet_count_o+1'b1;end
		end
	end
	// 原始LE文件的第0字节写入WDATA[7:0]，不改变96bit LUT字段。
	always@(*)begin
		dec_opcode=0;dec_version=0;dec_magic=0;dec_sequence=upload_sequence_o;dec_reserved=0;
		dec_base=0;dec_length=0;dec_offset=0;
		if(reg_command_count==16)begin
			dec_magic=reg_command[127:112];dec_version=reg_command[111:104];dec_opcode=reg_command[103:96];
			dec_base=reg_command[95:64];dec_length=reg_command[63:32];dec_sequence=reg_command[31:16];dec_reserved=reg_command[15:0];
		end else if(reg_command_count==32)begin
			dec_magic=reg_command[255:240];dec_version=reg_command[239:232];dec_opcode=reg_command[231:224];dec_offset=reg_command[223:192];
		end else if(reg_command_count==8)begin
			dec_magic=reg_command[63:48];dec_version=reg_command[47:40];dec_opcode=reg_command[39:32];dec_sequence=reg_command[31:16];dec_reserved=reg_command[15:0];
		end
		for(byte_index=0;byte_index<24;byte_index=byte_index+1)begin dec_payload[byte_index*8+:8]=reg_command[191-byte_index*8-:8];end
		dec_total={1'b0,cnt_carry}+7'd24;
		dec_accumulator={192'd0,reg_carry}|({192'd0,dec_payload}<<(cnt_carry*8));
		dec_error=0;
		if(flag_command_error)begin dec_error=10;end
		else if(reg_command_count!=reg_command_length || (reg_command_count!=8 && reg_command_count!=16 && reg_command_count!=32))begin dec_error=1;end
		else if(dec_magic!=16'h4c55 || dec_version!=1)begin dec_error=2;end
		else if(dec_opcode<1 || dec_opcode>3)begin dec_error=3;end
		else if((dec_opcode==1 && reg_command_count!=16) || (dec_opcode==2 && reg_command_count!=32) || (dec_opcode==3 && reg_command_count!=8))begin dec_error=1;end
		else if(dec_opcode==1 && dec_base!={4'd0,C_LUT_BASE})begin dec_error=4;end
		else if(dec_opcode==1 && dec_length!=C_LUT_BYTES)begin dec_error=5;end
		else if(dec_opcode!=2 && dec_reserved!=0)begin dec_error=12;end
		// 任何renderer占用期间都拒绝BEGIN，包括重复sequence；否则同序BEGIN
		// 会清空next_offset并使正在读取的LUT失效。
		else if(dec_opcode==1 && i_lut_in_use)begin dec_error=6;end
		else if(dec_opcode==2 && !upload_active_o)begin dec_error=7;end
		else if(dec_opcode==2 && dec_offset!=next_offset_o && !(flag_last_data && dec_offset==reg_last_offset && dec_payload==reg_last_payload))begin dec_error=8;end
		else if(dec_opcode==2 && dec_offset==next_offset_o && next_offset_o>C_LUT_BYTES-24)begin dec_error=8;end
		else if(dec_opcode==3 && (!flag_session || dec_sequence!=upload_sequence_o))begin dec_error=11;end
		else if(dec_opcode==3 && (next_offset_o!=C_LUT_BYTES || committed_bytes_o!=C_LUT_BYTES || cnt_carry!=0))begin dec_error=9;end
	end
	always@(posedge i_ddr_clk or negedge i_ddr_rst_n)begin
		if(!i_ddr_rst_n)begin
			reg_request_sync<=0;flag_request_seen<=0;state_current<=ST_IDLE;
			reg_command<=0;reg_command_count<=0;reg_command_length<=0;flag_command_error<=0;
			reg_response<=0;flag_response_toggle<=0;reg_response_opcode<=0;reg_response_event<=0;reg_response_error<=0;reg_response_sequence<=0;
			upload_active_o<=0;lut_ready_o<=0;flag_session<=0;upload_sequence_o<=0;next_offset_o<=0;committed_bytes_o<=0;
			reg_carry<=0;cnt_carry<=0;reg_last_payload<=0;reg_last_offset<=0;flag_last_data<=0;reg_write_data<=0;
		end else begin
			reg_request_sync<={reg_request_sync[0],flag_request_toggle};
			case(state_current)
				ST_IDLE:begin
					if(reg_request_sync[1]!=flag_request_seen)begin
						{flag_command_error,reg_command_count,reg_command_length,reg_command}<=reg_mailbox;
						flag_request_seen<=reg_request_sync[1];state_current<=ST_EVAL;
					end
				end
				ST_EVAL:begin
					reg_response_opcode<=dec_opcode | 8'h80;reg_response_sequence<=dec_sequence;reg_response_error<=dec_error;reg_response_event<=3;state_current<=ST_RESPONSE;
					if(dec_error==0)begin
						reg_response_event<=1;
						if(dec_opcode==1)begin
							if(!flag_session || dec_sequence!=upload_sequence_o)begin
								upload_sequence_o<=dec_sequence;next_offset_o<=0;committed_bytes_o<=0;reg_carry<=0;cnt_carry<=0;flag_last_data<=0;
								upload_active_o<=1;lut_ready_o<=0;flag_session<=1;
							end
						end else if(dec_opcode==2)begin
							if(dec_offset==next_offset_o)begin
								next_offset_o<=next_offset_o+24;reg_last_offset<=dec_offset;reg_last_payload<=dec_payload;flag_last_data<=1;
								if(dec_total>=32)begin
									reg_write_data<=dec_accumulator[255:0];reg_carry<={64'd0,dec_accumulator[383:256]};cnt_carry<=dec_total-32;state_current<=ST_AW;
								end else begin reg_carry<=dec_accumulator[191:0];cnt_carry<=dec_total[5:0];end
							end
						end else begin
							lut_ready_o<=1;upload_active_o<=0;reg_response_event<=2;
						end
					end
				end
				ST_AW:begin if(i_axi_awready)begin state_current<=ST_W;end end
				ST_W:begin if(i_axi_wready)begin committed_bytes_o<=committed_bytes_o+32;state_current<=ST_RESPONSE;end end
				ST_RESPONSE:begin
					reg_response<={16'h4c55,8'd1,reg_response_opcode,reg_response_sequence,reg_response_event,reg_response_error,next_offset_o,committed_bytes_o};
					flag_response_toggle<=!flag_response_toggle;state_current<=ST_IDLE;
				end
				default:begin state_current<=ST_IDLE;end
			endcase
		end
	end
endmodule
