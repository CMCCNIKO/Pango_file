//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 把全局 CHW 字节特征按 tile halo 与 padding 采样，转换为 C8/P2 128 位加载字，支持 bottleneck 9×9 及 pointwise 8×8 布局。
// Module Name: f14_feature_tile_gather
// File Name: model/rtl/f14_feature_tile_gather_v4.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 把全局 CHW 字节特征按 tile halo 与 padding 采样，转换为 C8/P2 128 位加载字，支持 bottleneck 9×9 及 pointwise 8×8 布局。
// 逐字节或参数化奇偶配对读取，按输入 zero 填越界点；16 lane 排空后提交一个字，输出背压期间停止新读。
//
// Dependencies: 外部单拍 feature bank；可选奇偶 pair 读口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原修订0.04采用同步控制复位，i_rst_n 至少低两上升沿且消费者共同 flush；本文件定义 f14_feature_tile_gather，不能与其他同名版本同时编译。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 full-map CHW to C8/P2 gather
// Module Name: f14_feature_tile_gather
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: SOP tile-load layout, one128bit word, no convolution arithmetic.
// Dependencies: External one-cycle byte-addressed feature bank
// Revision: 0.04 - Synchronous reset for the gather control domain
// Reset contract: hold i_rst_n low for at least2 rising edges; consumers flush together.
// State/valid clear on the next clock, not asynchronously.
// V2 Parent: previous V2 source (digest omitted)
// Parent: f14_feature_tile_gather.v
// 同名替代源，每次构建只选v1或v2之一；缺响应当拍不再发读，done保持单拍。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 固定一拍读响应，每拍最多一个字节；连续16个lane后排空再提交一字。
// 9x9用于bottleneck halo，8x8用于pointwise；局部地址group*41+pair。
// C8组计数4bit覆盖1..9组。width仅允许F14 backbone的112/56/28/14。
// 输入origin可为-1；越界按输入zero填充，第82个9x9点也为padding。
// reset时调用方必须同步清空在途响应；输出背压期间不发新读。
module f14_feature_tile_gather #(
	parameter integer C_PAIR_ENABLE = 0
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	input wire [17:0] i_base_addr,
	input wire i_bank_b,
	input wire [6:0] i_input_h,
	input wire [6:0] i_input_w,
	input wire [3:0] i_input_groups,
	input wire signed [9:0] i_origin_y,
	input wire signed [9:0] i_origin_x,
	input wire i_grid9,
	input wire [7:0] i_input_zero,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy,
	output wire o_rd_en,
	output wire o_rd_bank_b,
	output wire [17:0] o_rd_addr,
	input wire i_rd_valid,
	input wire [7:0] i_rd_data,
	output wire o_word_valid,
	input wire i_word_ready,
	output wire [9:0] o_word_addr,
	output wire [127:0] o_word_data,
	output wire o_pair_en,
	output wire [17:0] o_pair_addr,
	input wire [7:0] i_pair_data
);
	localparam [2:0] ST_IDLE = 3'd0;
	localparam [2:0] ST_PREP = 3'd1;
	localparam [2:0] ST_ISSUE = 3'd2;
	localparam [2:0] ST_DRAIN = 3'd3;
	localparam [2:0] ST_SEND = 3'd4;
	localparam [2:0] ST_DONE = 3'd5;
	localparam [2:0] ST_CHECK = 3'd6;
	reg [2:0] state_q;
	reg [2:0] state_d;
	reg active_q;
	reg [3:0] groups_q;
	reg [3:0] group_q;
	reg [3:0] check_groups_q;
	reg [19:0] check_end_q;
	reg bank_b_q;
	reg [5:0] pair_q;
	reg [3:0] lane_q;
	reg [3:0] response_lane_q;
	reg expected_q;
	reg pair_mode_q, response_pair_q;
	reg [6:0] height_q;
	reg [6:0] width_q;
	reg signed [9:0] origin_y_q;
	reg signed [9:0] origin_x_q;
	reg grid9_q;
	reg [7:0] zero_q;
	reg [3:0] local_y_q;
	reg [3:0] local_x_q;
	reg [15:0] plane_q;
	reg [17:0] group_base_q;
	reg [17:0] channel_offset_q;
	reg [17:0] spatial0_q;
	reg [17:0] spatial1_q;
	reg valid0_q;
	reg valid1_q;
	reg error_q;
	reg [15:0] input_plane;
	reg [17:0] row0;
	reg [17:0] row1;
	wire job_fire;
	wire job_legal;
	wire [3:0] edge_x;
	wire [3:0] local_x1;
	wire [3:0] local_y1;
	wire signed [9:0] sy0;
	wire signed [9:0] sx0;
	wire signed [9:0] sy1;
	wire signed [9:0] sx1;
	wire [17:0] uy0;
	wire [17:0] uy1;
	wire [17:0] source_spatial;
	wire source_valid;
	wire response_error;
	wire range_error;
	wire [19:0] next_check_end;
	genvar byte_index;

	assign job_fire = i_job_valid && o_job_ready;
	assign job_legal = (i_input_h >= 7'd1) && (i_input_h <= 7'd84)
		&& ((i_input_w == 7'd112) || (i_input_w == 7'd56) || (i_input_w == 7'd28) || (i_input_w == 7'd14))
		&& (i_input_groups >= 4'd1) && (i_input_groups <= 4'd9)
		&& (i_origin_y >= -10'sd1) && (i_origin_y < $signed({3'd0,i_input_h}))
		&& (i_origin_x >= -10'sd1) && (i_origin_x < $signed({3'd0,i_input_w}));
	// 先逐组累计全图跨度，最多9拍；检查完整CHW范围后才允许第一次读。
	// 使用20bit中间值，拒绝18bit地址回绕及B bank容量小于A的情况。
	assign next_check_end = check_end_q + {1'b0,plane_q,3'b000};
	assign range_error = (state_q == ST_CHECK)
		&& (next_check_end > (bank_b_q ? 20'd150528 : 20'd225792));
	assign o_job_ready = active_q && (state_q == ST_IDLE);
	assign o_job_done = active_q && (state_q == ST_DONE);
	assign o_busy = active_q && (state_q != ST_IDLE);
	assign o_job_error = error_q;
	assign edge_x = grid9_q ? 4'd8 : 4'd7;
	assign local_x1 = (local_x_q == edge_x) ? 4'd0 : local_x_q + 4'd1;
	assign local_y1 = local_y_q + ((local_x_q == edge_x) ? 4'd1 : 4'd0);
	assign sy0 = origin_y_q + $signed({6'd0, local_y_q});
	assign sx0 = origin_x_q + $signed({6'd0, local_x_q});
	assign sy1 = origin_y_q + $signed({6'd0, local_y1});
	assign sx1 = origin_x_q + $signed({6'd0, local_x1});
	assign uy0 = {8'd0, sy0};
	assign uy1 = {8'd0, sy1};
	assign source_spatial = lane_q[3] ? spatial1_q : spatial0_q;
	assign source_valid = lane_q[3] ? valid1_q : valid0_q;
	assign o_rd_en = active_q && (state_q == ST_ISSUE) && source_valid && !response_error;
	assign o_rd_bank_b = bank_b_q;
	assign o_rd_addr = group_base_q + channel_offset_q + source_spatial;
	assign o_pair_en = o_rd_en && pair_mode_q;
	assign o_pair_addr = group_base_q + channel_offset_q + spatial1_q;
	assign response_error = active_q && expected_q && !i_rd_valid;
	assign o_word_valid = active_q && (state_q == ST_SEND);
	assign o_word_addr = {6'd0, group_q} * 10'd41 + {4'd0, pair_q};

	always@(*)begin
		case(i_input_w)
			7'd112:begin input_plane = ({9'd0,i_input_h} << 7) - ({9'd0,i_input_h} << 4); end
			7'd56:begin input_plane = ({9'd0,i_input_h} << 6) - ({9'd0,i_input_h} << 3); end
			7'd28:begin input_plane = ({9'd0,i_input_h} << 5) - ({9'd0,i_input_h} << 2); end
			7'd14:begin input_plane = ({9'd0,i_input_h} << 4) - ({9'd0,i_input_h} << 1); end
			default:begin input_plane = 16'd0; end
		endcase
		case(width_q)
			7'd112:begin row0 = (uy0 << 7) - (uy0 << 4); row1 = (uy1 << 7) - (uy1 << 4); end
			7'd56:begin row0 = (uy0 << 6) - (uy0 << 3); row1 = (uy1 << 6) - (uy1 << 3); end
			7'd28:begin row0 = (uy0 << 5) - (uy0 << 2); row1 = (uy1 << 5) - (uy1 << 2); end
			7'd14:begin row0 = (uy0 << 4) - (uy0 << 1); row1 = (uy1 << 4) - (uy1 << 1); end
			default:begin row0 = 18'd0; row1 = 18'd0; end
		endcase
	end
	always@(posedge i_clk)begin
		if(!i_rst_n)begin state_q <= ST_IDLE; active_q <= 1'b0; end
		else begin state_q <= state_d; active_q <= 1'b1; end
	end
	always@(*)begin
		state_d = state_q;
		case(state_q)
			ST_IDLE:begin
				if(job_fire)begin
					if(job_legal)begin state_d = ST_CHECK; end
					else begin state_d = ST_DONE; end
				end
			end
			ST_CHECK:begin
				if(range_error)begin state_d = ST_DONE; end
				else if(check_groups_q == 4'd1)begin state_d = ST_PREP; end
			end
			ST_PREP:begin state_d = ST_ISSUE; end
			ST_ISSUE:begin if(lane_q == (pair_mode_q ? 4'd7 : 4'd15))begin state_d = ST_DRAIN; end end
			ST_DRAIN:begin state_d = ST_SEND; end
			ST_SEND:begin
				if(i_word_ready)begin
					if((group_q + 4'd1 == groups_q) && (pair_q == (grid9_q ? 6'd40 : 6'd31)))begin state_d = ST_DONE; end
					else begin state_d = ST_PREP; end
				end
			end
			ST_DONE:begin state_d = ST_IDLE; end
			default:begin state_d = ST_IDLE; end
		endcase
		if(response_error)begin state_d = ST_DONE; end
	end
	always@(posedge i_clk)begin
		if(!i_rst_n)begin
			groups_q <= 4'd0; group_q <= 4'd0; pair_q <= 6'd0;
			check_groups_q <= 4'd0; check_end_q <= 20'd0; bank_b_q <= 1'b0;
			response_lane_q <= 4'd0; expected_q <= 1'b0;
			pair_mode_q <= 1'b0; response_pair_q <= 1'b0;
			height_q <= 7'd0; width_q <= 7'd0;
			origin_y_q <= 10'sd0; origin_x_q <= 10'sd0;
			grid9_q <= 1'b0; zero_q <= 8'd0;
			local_y_q <= 4'd0; local_x_q <= 4'd0; plane_q <= 16'd0;


			valid0_q <= 1'b0; valid1_q <= 1'b0; error_q <= 1'b0;
		end else begin
			expected_q <= o_rd_en;
			response_pair_q <= o_pair_en;
			if(o_rd_en)begin response_lane_q <= lane_q; end
			if(job_fire)begin
				groups_q <= i_input_groups; group_q <= 4'd0; pair_q <= 6'd0;
				check_groups_q <= i_input_groups; check_end_q <= {2'd0,i_base_addr}; bank_b_q <= i_bank_b;
				height_q <= i_input_h; width_q <= i_input_w;
				origin_y_q <= i_origin_y; origin_x_q <= i_origin_x;
				grid9_q <= i_grid9; zero_q <= i_input_zero;
				local_y_q <= 4'd0; local_x_q <= 4'd0;
				plane_q <= input_plane; error_q <= !job_legal;
			end
			if(state_q == ST_CHECK)begin
				check_end_q <= next_check_end;
				check_groups_q <= check_groups_q - 4'd1;
				if(range_error)begin error_q <= 1'b1; end
			end
			if(state_q == ST_PREP)begin
				// A bank仅在两个有效位置奇偶不同的情况下双读；跨行同奇偶退回单读。
				pair_mode_q <= (C_PAIR_ENABLE != 0) && !bank_b_q && (sx0[0] != sx1[0])
					&& (sy0 >= 0) && (sy0 < $signed({3'd0,height_q}))
					&& (sx0 >= 0) && (sx0 < $signed({3'd0,width_q}))
					&& (!grid9_q || local_y1 < 9)
					&& (sy1 >= 0) && (sy1 < $signed({3'd0,height_q}))
					&& (sx1 >= 0) && (sx1 < $signed({3'd0,width_q}));


				valid0_q <= (sy0 >= 10'sd0) && (sy0 < $signed({3'd0,height_q}))
					&& (sx0 >= 10'sd0) && (sx0 < $signed({3'd0,width_q}));
				valid1_q <= (!grid9_q || (local_y1 < 4'd9))
					&& (sy1 >= 10'sd0) && (sy1 < $signed({3'd0,height_q}))
					&& (sx1 >= 10'sd0) && (sx1 < $signed({3'd0,width_q}));
			end
			if(state_q == ST_ISSUE)begin



			end
			if((state_q == ST_SEND) && i_word_ready)begin
				if(pair_q == (grid9_q ? 6'd40 : 6'd31))begin
					pair_q <= 6'd0; group_q <= group_q + 4'd1;

					local_x_q <= 4'd0; local_y_q <= 4'd0;
				end else begin
					pair_q <= pair_q + 6'd1;
					local_x_q <= (local_x1 == edge_x) ? 4'd0 : local_x1 + 4'd1;
					local_y_q <= local_y1 + ((local_x1 == edge_x) ? 4'd1 : 4'd0);
				end
			end
			if(response_error)begin error_q <= 1'b1; end
		end
	end

	// 地址数据寄存器不异步清零，避免复位在DRM时钟沿附近改变ADDR。
	// 每个新job先装base，每个word在PREP先装offset/spatial/lane，再发读。
	// 控制state/valid仍异步清零；复位期间地址保持，不引入读延迟。
	always@(posedge i_clk)begin
		if(i_rst_n)begin
			if(job_fire)begin group_base_q <= i_base_addr; end
			if(state_q == ST_PREP)begin
				lane_q <= 4'd0;
				channel_offset_q <= 18'd0;
				spatial0_q <= row0 + {8'd0,sx0};
				spatial1_q <= row1 + {8'd0,sx1};
			end
			if(state_q == ST_ISSUE)begin
				lane_q <= lane_q + 4'd1;
				if(lane_q[2:0] == 3'd7)begin channel_offset_q <= 18'd0; end
				else begin channel_offset_q <= channel_offset_q + {2'd0,plane_q}; end
			end
			if((state_q == ST_SEND) && i_word_ready && (pair_q == (grid9_q ? 6'd40 : 6'd31)))begin
				group_base_q <= group_base_q + {plane_q,3'b000};
			end
		end
	end
	generate
		for(byte_index = 0; byte_index < 16; byte_index = byte_index + 1)begin:gen_word
			reg [7:0] data_q;
			always@(posedge i_clk)begin
				if(state_q == ST_PREP)begin data_q <= zero_q; end
				else if(expected_q && i_rd_valid && (response_lane_q == byte_index))begin data_q <= i_rd_data; end
				else if(expected_q && i_rd_valid && response_pair_q && (response_lane_q + 4'd8 == byte_index))begin data_q <= i_pair_data; end
			end
			assign o_word_data[byte_index * 8 +: 8] = data_q;
		end
	endgenerate
endmodule
`default_nettype wire
