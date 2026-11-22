//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 27MHz 系统域通过 SD SPI CMD17 顺序读 LUT，32 字节低字节优先组成一个 256-bit beat，经稳定 mailbox 交给 DDR 单拍写入。
// Module Name: avm_lut_sd_loader
// File Name: source/avm/avm_lut_sd_loader.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 27MHz 系统域通过 SD SPI CMD17 顺序读 LUT，32 字节低字节优先组成一个 256-bit beat，经稳定 mailbox 交给 DDR 单拍写入。
// 默认读取 9600 块/4915200 字节，153600 次实际 W 接受后发布 LUT ready；DDR 失效先排空已开始写事务，再清会话并从头重装。
// 协议或背压超时锁存本会话错误，失败会话不发布 ready，重新初始化后恢复。
//
// Dependencies: sd_spi_reader、spi_byte_master、avm_sd_beat_mailbox、avm_single_beat_writer。
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
// Create Date: 2026-10-14
// Design Name: SD boot LUT loader
// Module Name: avm_lut_sd_loader
// Project Name: PG2L100H SD AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 27MHz CMD17连续读表，32字节小端拼包，经稳定mailbox写DDR。
//              默认9600块/4915200字节/153600次实际W接受后发布ready。
//              DDR失效先排空已开始写事务，再共同清除会话并整表重装。
//              协议/背压错误锁存，本次会话不得发布ready；重初始化恢复。
// Dependencies: sd_spi_reader, spi_byte_master, avm_sd_beat_mailbox,
//               avm_single_beat_writer；不新增存储或时钟IP。
// Revision: 0.01 - 首次实现。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_lut_sd_loader
#(
	parameter [31:0] C_FIRST_LBA = 32'd16672,
	parameter C_BLOCKS = 9600,
	parameter [27:0] C_LUT_BASE = 28'h0300000,
	parameter C_INIT_DIV = 54,
	parameter C_READ_DIV = 4,
	parameter C_POWER_WAIT = 270000,
	parameter C_PRELOAD_WAIT_CYCLES = 135000000,
	parameter C_TOKEN_TIMEOUT_CYCLES = 28620000,
	parameter C_PROGRESS_TIMEOUT_CYCLES = 27000000
)
(
	input i_sd_clk,
	input i_ddr_clk,
	input i_rstn,
	input i_ddr_init_done,
	output o_sd_clk,
	output o_sd_mosi,
	input i_sd_miso,
	output o_sd_cs,
	output o_lut_ready,
	output o_busy,
	output [7:0] o_error,
	output [31:0] o_committed_beats,
	output [31:0] o_loaded_bytes,
	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	input i_axi_awready,
	output [255:0] o_axi_wdata,
	output [31:0] o_axi_wstrb,
	output o_axi_wvalid,
	input i_axi_wready
);
	localparam [1:0] ST_RESET = 2'd0;
	localparam [1:0] ST_RUN = 2'd1;
	localparam [1:0] ST_DRAIN = 2'd2;
	localparam [3:0] ST_SD_WAIT = 4'd0;
	localparam [3:0] ST_SD_REQUEST = 4'd1;
	localparam [3:0] ST_SD_RECEIVE = 4'd2;
	localparam [3:0] ST_SD_POST = 4'd3;
	localparam [3:0] ST_SD_ACK = 4'd4;
	localparam [3:0] ST_SD_BLOCK_WAIT = 4'd5;
	localparam [3:0] ST_SD_NEXT = 4'd6;
	localparam [3:0] ST_SD_DONE = 4'd7;
	localparam [3:0] ST_SD_ERROR = 4'd8;
	localparam [31:0] LUT_BYTES = C_BLOCKS * 512;
	localparam [31:0] LUT_BEATS = C_BLOCKS * 16;
	reg [1:0] reg_ddr_reset_sync;
	reg [1:0] reg_sd_reset_sync;
	reg [1:0] reg_session_reset_sync;
	reg [1:0] state_current;
	reg [1:0] state_next;
	reg reg_session_active;
	reg [4:0] cnt_reset;
	reg [3:0] state_sd_current;
	reg [3:0] state_sd_next;
	reg [31:0] cnt_blocks;
	reg [9:0] cnt_block_bytes;
	reg [31:0] cnt_bytes;
	reg [4:0] cnt_pack;
	reg [31:0] cnt_progress;
	reg [31:0] cnt_preload;
	reg [255:0] reg_pack;
	reg [255:0] reg_data_hold;
	reg [27:0] reg_address_hold;
	reg [27:0] reg_next_address;
	reg reg_request;
	reg reg_ack_meta /* synthesis syn_preserve = 1 */;
	reg reg_ack_sync /* synthesis syn_preserve = 1 */;
	reg flag_block_done;
	reg flag_sd_done;
	reg [7:0] reg_sd_error;
	reg reg_done_meta /* synthesis syn_preserve = 1 */;
	reg reg_done_sync /* synthesis syn_preserve = 1 */;
	reg reg_error_meta /* synthesis syn_preserve = 1 */;
	reg reg_error_sync /* synthesis syn_preserve = 1 */;
	reg [7:0] error_o;
	reg lut_ready_o;
	reg [31:0] committed_beats_o;
	wire ddr_rstn;
	wire session_arstn;
	wire session_ddr_rstn;
	wire session_sd_rstn;
	wire reader_ready;
	wire reader_valid;
	wire reader_done;
	wire [7:0] reader_error;
	wire [7:0] reader_data;
	wire reader_start;
	wire payload_ready;
	wire payload_take;
	wire progress_expired;
	wire mailbox_ack;
	wire mailbox_valid;
	wire mailbox_idle;
	wire [27:0] mailbox_address;
	wire [255:0] mailbox_data;
	wire writer_ready;
	wire writer_drained;
	assign ddr_rstn = reg_ddr_reset_sync[1];
	assign session_arstn = i_rstn && reg_session_active;
	assign session_sd_rstn = reg_sd_reset_sync[1];
	assign session_ddr_rstn = reg_session_reset_sync[1];
	assign reader_start = (state_sd_current == ST_SD_REQUEST) && reader_ready;
	assign payload_ready = (state_sd_current == ST_SD_RECEIVE) && (cnt_block_bytes < 10'd512);
	assign payload_take = reader_valid && payload_ready;
	assign progress_expired = (state_sd_current == ST_SD_ACK) && (cnt_progress >= C_PROGRESS_TIMEOUT_CYCLES - 1);
	assign o_lut_ready = lut_ready_o && i_ddr_init_done;
	assign o_busy = !o_lut_ready && (error_o == 8'd0);
	assign o_error = error_o;
	assign o_committed_beats = committed_beats_o;
	assign o_loaded_bytes = committed_beats_o << 5;
	// 各域均异步置复位、同步释放；会话复位必须由排空状态机发起。
	always@(posedge i_ddr_clk or negedge i_rstn)begin
		if(!i_rstn)begin reg_ddr_reset_sync <= 2'b00; end
		else begin reg_ddr_reset_sync <= {reg_ddr_reset_sync[0], 1'b1}; end
	end
	always@(posedge i_sd_clk or negedge session_arstn)begin
		if(!session_arstn)begin reg_sd_reset_sync <= 2'b00; end
		else begin reg_sd_reset_sync <= {reg_sd_reset_sync[0], 1'b1}; end
	end
	always@(posedge i_ddr_clk or negedge session_arstn)begin
		if(!session_arstn)begin reg_session_reset_sync <= 2'b00; end
		else begin reg_session_reset_sync <= {reg_session_reset_sync[0], 1'b1}; end
	end
	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_RESET:begin
				if(i_ddr_init_done && (cnt_reset == 5'd31))begin state_next = ST_RUN; end
			end
			ST_RUN:begin
				if(!i_ddr_init_done)begin state_next = ST_DRAIN; end
			end
			ST_DRAIN:begin
				if(writer_drained && mailbox_idle)begin state_next = ST_RESET; end
			end
			default:begin state_next = ST_RESET; end
		endcase
	end
	always@(posedge i_ddr_clk or negedge ddr_rstn)begin
		if(!ddr_rstn)begin state_current <= ST_RESET; end
		else begin state_current <= state_next; end
	end
	always@(posedge i_ddr_clk or negedge ddr_rstn)begin
		if(!ddr_rstn)begin reg_session_active <= 1'b0; end
		else if(state_next == ST_RESET)begin reg_session_active <= 1'b0; end
		else if(state_next == ST_RUN)begin reg_session_active <= 1'b1; end
	end
	always@(posedge i_ddr_clk or negedge ddr_rstn)begin
		if(!ddr_rstn)begin cnt_reset <= 5'd0; end
		else if(state_current != ST_RESET)begin cnt_reset <= 5'd0; end
		else if(cnt_reset != 5'd31)begin cnt_reset <= cnt_reset + 1'b1; end
	end
	// 发送状态机：每次请求前一拍完成hold寄存器；ACK后才允许覆盖。
	always@(*)begin
		state_sd_next = state_sd_current;
		case(state_sd_current)
			ST_SD_WAIT:begin
				if((C_PRELOAD_WAIT_CYCLES == 0 || cnt_preload >= C_PRELOAD_WAIT_CYCLES) && reader_ready)begin
					state_sd_next = ST_SD_REQUEST;
				end
			end
			ST_SD_REQUEST:begin
				if(reader_ready)begin state_sd_next = ST_SD_RECEIVE; end
			end
			ST_SD_RECEIVE:begin
				if(payload_take && (cnt_pack == 5'd31))begin state_sd_next = ST_SD_POST; end
				else if(flag_block_done)begin
					if(cnt_block_bytes != 10'd512)begin state_sd_next = ST_SD_ERROR; end
					else if(cnt_blocks == C_BLOCKS - 1)begin state_sd_next = ST_SD_DONE; end
					else begin state_sd_next = ST_SD_NEXT; end
				end
			end
			// 最后一个payload字节后仍需等待reader完成CRC和CS释放，
			// 不能把payload打包完成误当作块完成。
			// 一个256-bit beat发送完成即可等待mailbox ACK；整块512-byte读完
			// 由flag_block_done单独标记，不能在这里等待，否则首个32-byte后死锁。
			ST_SD_POST:begin state_sd_next = ST_SD_ACK; end
			ST_SD_ACK:begin
				if(reg_ack_sync == reg_request)begin
					// cnt_block_bytes is already 512 after the last accepted byte.
					// The reader may still be consuming CRC/CS, so wait for its
					// independent block-done indication before advancing LBA.
					if(cnt_block_bytes == 10'd512)begin
						state_sd_next = ST_SD_BLOCK_WAIT;
					end else begin
						state_sd_next = ST_SD_RECEIVE;
					end
				end
			end
			ST_SD_BLOCK_WAIT:begin
				if(flag_block_done)begin
					if(cnt_block_bytes != 10'd512)begin state_sd_next = ST_SD_ERROR; end
					else if(cnt_blocks == C_BLOCKS - 1)begin state_sd_next = ST_SD_DONE; end
					else begin state_sd_next = ST_SD_NEXT; end
				end
			end
			ST_SD_NEXT:begin state_sd_next = ST_SD_REQUEST; end
			ST_SD_DONE:begin state_sd_next = ST_SD_DONE; end
			ST_SD_ERROR:begin state_sd_next = ST_SD_ERROR; end
			default:begin state_sd_next = ST_SD_ERROR; end
		endcase
		if((reader_error != 8'd0) || progress_expired)begin state_sd_next = ST_SD_ERROR; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin state_sd_current <= ST_SD_WAIT; end
		else begin state_sd_current <= state_sd_next; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin
			cnt_preload <= 32'd0;
		end else if(state_sd_current != ST_SD_WAIT)begin
			cnt_preload <= 32'd0;
		end else if(cnt_preload < C_PRELOAD_WAIT_CYCLES)begin
			cnt_preload <= cnt_preload + 1'b1;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin
			reg_ack_meta <= 1'b0;
			reg_ack_sync <= 1'b0;
		end else begin
			reg_ack_meta <= mailbox_ack;
			reg_ack_sync <= reg_ack_meta;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin reg_request <= 1'b0; end
		else if((state_sd_current == ST_SD_POST) && (state_sd_next == ST_SD_ACK))begin
			// 仅在块CRC/CS完成的那个状态沿翻转一次请求，避免POST等待时重复发送。
			reg_request <= !reg_request;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin cnt_progress <= 32'd0; end
		else if(state_sd_current != ST_SD_ACK)begin cnt_progress <= 32'd0; end
		else if(!progress_expired)begin cnt_progress <= cnt_progress + 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin cnt_blocks <= 32'd0; end
		else if(state_sd_current == ST_SD_NEXT)begin cnt_blocks <= cnt_blocks + 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin cnt_block_bytes <= 10'd0; end
		else if(state_sd_current == ST_SD_REQUEST)begin cnt_block_bytes <= 10'd0; end
		else if(payload_take)begin cnt_block_bytes <= cnt_block_bytes + 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin cnt_bytes <= 32'd0; end
		else if(payload_take)begin cnt_bytes <= cnt_bytes + 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin cnt_pack <= 5'd0; end
		else if(payload_take)begin cnt_pack <= cnt_pack + 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin reg_pack <= 256'd0; end
		else if(payload_take)begin reg_pack[cnt_pack * 8 +: 8] <= reader_data; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin
			reg_data_hold <= 256'd0;
			reg_address_hold <= C_LUT_BASE;
		end else if(payload_take && (cnt_pack == 5'd31))begin
			reg_data_hold <= {reader_data, reg_pack[247:0]};
			reg_address_hold <= reg_next_address;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin reg_next_address <= C_LUT_BASE; end
		else if((state_sd_current == ST_SD_ACK) && (reg_ack_sync == reg_request))begin
			reg_next_address <= reg_next_address + 28'd8;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin flag_block_done <= 1'b0; end
		else if(state_sd_current == ST_SD_REQUEST)begin flag_block_done <= 1'b0; end
		else if(reader_done)begin flag_block_done <= 1'b1; end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin flag_sd_done <= 1'b0; end
		else if((state_sd_current == ST_SD_DONE) && (cnt_bytes == LUT_BYTES) &&
			(cnt_pack == 5'd0) && (reg_request == reg_ack_sync) && (reg_sd_error == 8'd0))begin
			flag_sd_done <= 1'b1;
		end
	end
	always@(posedge i_sd_clk or negedge session_sd_rstn)begin
		if(!session_sd_rstn)begin reg_sd_error <= 8'd0; end
		else if(reg_sd_error == 8'd0)begin
			if(reader_error != 8'd0)begin reg_sd_error <= reader_error; end
			else if(progress_expired)begin reg_sd_error <= 8'h80; end
			else if(state_sd_next == ST_SD_ERROR)begin reg_sd_error <= 8'h81; end
		end
	end
	// 完成/错误均为会话内稳定电平；多位错误码在同步指示到达后捕获。
	always@(posedge i_ddr_clk or negedge session_ddr_rstn)begin
		if(!session_ddr_rstn)begin
			reg_done_meta <= 1'b0;
			reg_done_sync <= 1'b0;
			reg_error_meta <= 1'b0;
			reg_error_sync <= 1'b0;
		end else begin
			reg_done_meta <= flag_sd_done;
			reg_done_sync <= reg_done_meta;
			reg_error_meta <= |reg_sd_error;
			reg_error_sync <= reg_error_meta;
		end
	end
	always@(posedge i_ddr_clk or negedge session_ddr_rstn)begin
		if(!session_ddr_rstn)begin error_o <= 8'd0; end
		else if(reg_error_sync)begin error_o <= reg_sd_error; end
	end
	always@(posedge i_ddr_clk or negedge session_ddr_rstn)begin
		if(!session_ddr_rstn)begin committed_beats_o <= 32'd0; end
		else if(writer_ready)begin committed_beats_o <= committed_beats_o + 1'b1; end
	end
	always@(posedge i_ddr_clk or negedge session_ddr_rstn)begin
		if(!session_ddr_rstn)begin lut_ready_o <= 1'b0; end
		else if(!i_ddr_init_done || (state_current != ST_RUN) || reg_error_sync)begin
			lut_ready_o <= 1'b0;
		end else if(reg_done_sync && (committed_beats_o == LUT_BEATS) && mailbox_idle && writer_drained)begin
			lut_ready_o <= 1'b1;
		end
	end
	sd_spi_reader
	#(
		.C_INIT_DIV(C_INIT_DIV),
		.C_READ_DIV(C_READ_DIV),
		.C_POWER_WAIT(C_POWER_WAIT),
		.C_TOKEN_TIMEOUT_CYCLES(C_TOKEN_TIMEOUT_CYCLES),
		.C_BACKPRESSURE_TIMEOUT_CYCLES(C_PROGRESS_TIMEOUT_CYCLES)
	) sd_spi_reader_Inst_sd(
		.i_clk(i_sd_clk),
		.i_rstn(session_sd_rstn),
		.i_start(reader_start),
		.i_lba(C_FIRST_LBA + cnt_blocks),
		.o_ready(reader_ready),
		.o_init_done(),
		.o_error(reader_error),
		.o_data(reader_data),
		.o_valid(reader_valid),
		.i_ready(payload_ready),
		.o_done(reader_done),
		.o_sd_clk(o_sd_clk),
		.o_sd_mosi(o_sd_mosi),
		.i_sd_miso(i_sd_miso),
		.o_sd_cs(o_sd_cs),
		.o_last_r1(),
		.o_token(),
		.o_ccs()
	);
	avm_sd_beat_mailbox avm_sd_beat_mailbox_Inst_sd(
		.i_clk(i_ddr_clk),
		.i_rstn(session_ddr_rstn),
		.i_enable((state_current == ST_RUN) && i_ddr_init_done),
		.i_request(reg_request),
		.i_address_hold(reg_address_hold),
		.i_data_hold(reg_data_hold),
		.o_ack(mailbox_ack),
		.o_valid(mailbox_valid),
		.o_address(mailbox_address),
		.o_data(mailbox_data),
		.i_ready(writer_ready),
		.o_idle(mailbox_idle)
	);
	avm_single_beat_writer avm_single_beat_writer_Inst_sd(
		.i_clk(i_ddr_clk),
		.i_rst_n(session_ddr_rstn),
		.i_valid(mailbox_valid),
		.i_address(mailbox_address),
		.i_data(mailbox_data),
		.o_ready(writer_ready),
		.o_drained(writer_drained),
		.o_axi_awaddr(o_axi_awaddr),
		.o_axi_awlen(o_axi_awlen),
		.o_axi_awvalid(o_axi_awvalid),
		.i_axi_awready(i_axi_awready),
		.o_axi_wdata(o_axi_wdata),
		.o_axi_wstrb(o_axi_wstrb),
		.o_axi_wvalid(o_axi_wvalid),
		.i_axi_wready(i_axi_wready)
	);
endmodule
