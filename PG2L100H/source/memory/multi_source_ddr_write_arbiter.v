//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四源写请求的单 outstanding 仲裁器，near-full 源优先，同优先级从上次完成源的下一路开始轮询。
// Module Name: multi_source_ddr_write_arbiter
// File Name: source/memory/multi_source_ddr_write_arbiter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四源写请求的单 outstanding 仲裁器，near-full 源优先，同优先级从上次完成源的下一路开始轮询。
// 选择结果从 o_start 保持到 i_burst_done；读侧 urgent 阻止发起新写事务，上游须保证请求源已准备完整的配置 burst。
//
// Dependencies: 外部为所配置 C_BURST_BEATS 保证可消费 FIFO 字；multi_source_ddr_write_router。
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
// Create Date: 2026-09-13
// Design Name: Quad Video Mosaic
// Module Name: multi_source_ddr_write_arbiter
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 四路完整写 burst 请求的单 outstanding 仲裁。near-full 优先，
//              同一优先级从上次完成源的下一路循环选择；读侧紧急只阻止新 burst。
// Dependencies: 外部保证 i_request 对应源至少有完整 16 个 256-bit word。
// Revision:
// Revision 0.01 - File Created
// Additional Comments: o_source_id 从 o_start 到 i_burst_done 保持不变。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module multi_source_ddr_write_arbiter(
	input i_clk,
	input i_rstn,
	input [3:0] i_request,
	input [3:0] i_almost_full,
	input i_read_urgent,
	input i_engine_ready,
	input i_burst_done,
	input i_burst_failed,
	output o_start,
	output [1:0] o_source_id,
	output o_busy,
	output [127:0] o_burst_count
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_WAIT_DONE = 2'd1;

	reg [1:0] state_current;
	reg [1:0] state_next;
	reg [1:0] rr_pointer;
	reg [1:0] source_id_o;
	reg start_o;
	reg [31:0] burst_count0;
	reg [31:0] burst_count1;
	reg [31:0] burst_count2;
	reg [31:0] burst_count3;
	reg [3:0] eligible_mask;
	reg selected_valid;
	reg [1:0] selected_id;
	reg busy_o;
	integer offset;
	integer search_index;
	wire grant_condition;
	wire hdmi_emergency;

	assign o_start = start_o;
	assign o_source_id = source_id_o;
	assign o_busy = busy_o;
	assign o_burst_count = {burst_count3, burst_count2, burst_count1, burst_count0};
	// HDMI RX不能背压；网络接收同样没有逐字节 ready，source0 的 FIFO
	// 接近满时必须允许最小写服务，否则读侧 urgent 会把网络 FIFO 永久饿死。
	// 只在 near-full 救援时打破读优先，正常情况下仍由读侧优先。
	wire ethernet_emergency;
	assign hdmi_emergency = i_request[3] && i_almost_full[3];
	assign ethernet_emergency = i_request[0] && i_almost_full[0];
	assign grant_condition = (state_current == ST_IDLE) &&
		(i_engine_ready == 1'b1) &&
		(!i_read_urgent || hdmi_emergency || ethernet_emergency) && selected_valid;

	// near-full 来源优先；同组保持轮询公平性。
	always@(*)begin
		if(i_read_urgent)begin
			// urgent期间只放行接近满的不可背压源：source0 Ethernet和
			// source3 HDMI。两个救援源同时接近满时仍由轮询指针保证公平。
			eligible_mask = {hdmi_emergency,2'b00,ethernet_emergency};
		end else if((|(i_request & i_almost_full)) == 1'b1)begin
			eligible_mask = i_request & i_almost_full;
		end else begin
			eligible_mask = i_request;
		end
		selected_valid = 1'b0;
		selected_id = 2'd0;
		search_index = 0;
		for(offset = 0; offset < 4; offset = offset + 1)begin
			search_index = (rr_pointer + offset) & 3;
			if((selected_valid == 1'b0) && (eligible_mask[search_index] == 1'b1))begin
				selected_valid = 1'b1;
				selected_id = search_index[1:0];
			end
		end
	end

	// burst 授权后一直等实际完成，禁止中途重选。
	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(grant_condition)begin
					state_next = ST_WAIT_DONE;
				end
			end
			ST_WAIT_DONE:begin
				if((i_burst_done == 1'b1) || (i_burst_failed == 1'b1))begin
					state_next = ST_IDLE;
				end
			end
			default:begin
				state_next = ST_IDLE;
			end
		endcase
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(i_rstn == 1'b0)begin
			state_current <= ST_IDLE;
		end else begin
			state_current <= state_next;
		end
	end

	always@(*)begin
		busy_o = (state_current == ST_WAIT_DONE);
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(i_rstn == 1'b0)begin
			start_o <= 1'b0;
			source_id_o <= 2'd0;
		end else begin
			start_o <= grant_condition;
			if(grant_condition)begin
				source_id_o <= selected_id;
			end
		end
	end

	always@(posedge i_clk or negedge i_rstn)begin
		if(i_rstn == 1'b0)begin
			rr_pointer <= 2'd0;
			burst_count0 <= 32'd0;
			burst_count1 <= 32'd0;
			burst_count2 <= 32'd0;
			burst_count3 <= 32'd0;
		end else if((state_current == ST_WAIT_DONE) &&
			((i_burst_done == 1'b1) || (i_burst_failed == 1'b1)))begin
			rr_pointer <= source_id_o + 2'd1;
			if(i_burst_done == 1'b1)begin
			case(source_id_o)
				2'd0:begin
					burst_count0 <= burst_count0 + 32'd1;
				end
				2'd1:begin
					burst_count1 <= burst_count1 + 32'd1;
				end
				2'd2:begin
					burst_count2 <= burst_count2 + 32'd1;
				end
				2'd3:begin
					burst_count3 <= burst_count3 + 32'd1;
				end
				default:begin
				end
			endcase
			end
		end
	end
endmodule
