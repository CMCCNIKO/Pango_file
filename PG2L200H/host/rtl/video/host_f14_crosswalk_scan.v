//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-26
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 对 185×640 打包二值 ROI 进行串行斑马线几何搜索：顺序装载 13×32 tile，检验横向游程、纵向笔画、条纹间距及候选分组。
// Module Name: host_f14_crosswalk_scan
// File Name: host/rtl/video/host_f14_crosswalk_scan.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 对 185×640 打包二值 ROI 进行串行斑马线几何搜索：顺序装载 13×32 tile，检验横向游程、纵向笔画、条纹间距及候选分组。
// 通过唯一 32 位读口请求/响应获取数据，无图像级并行比较阵列和动态除法；输出当前帧候选框及扫描拍数，时间确认交由 gate。
//
// Dependencies: host_f14_crosswalk_roi.vh；系统域 i_clk/i_rst_n；外部读口需在请求后一拍或更晚以 valid 返回。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-26
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 185x640打包二值ROI串行扫描；唯一32-bit读口，req后一拍或更晚返回valid。
// 13x32小tile顺序装载后移位；纵向笔画按候选逐行查询。
// 无动态除法，无图像级并行比较阵列。输出为未确认的当前帧几何候选。
`timescale 1ns/1ps
`default_nettype none
module host_f14_crosswalk_scan (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	output wire o_busy,
	output wire o_done,
	output wire o_req,
	output wire [11:0] o_addr,
	input wire i_read_valid,
	input wire [31:0] i_read_data,
	output wire o_candidate,
	output wire [9:0] o_left,
	output wire [8:0] o_top,
	output wire [9:0] o_right,
	output wire [8:0] o_bottom,
	output wire [31:0] o_cycles
);
	localparam [4:0] ST_IDLE=0,ST_ROW=1,ST_TILE_REQ=2,ST_TILE_WAIT=3,
		ST_SHIFT=4,ST_V_INIT=5,ST_V_REQ=6,ST_V_WAIT=7,ST_V2_REQ=8,ST_V2_WAIT=9,
		ST_V_BIT=10,ST_APPEND=11,ST_SEARCH_INIT=12,ST_RUN_REQ=13,ST_RUN_WAIT=14,
		ST_RUN_CHECK=15,ST_RUN_ACCEPT=16,ST_NEXT_START=17,ST_SCALE_END=18,
		ST_GROUP=19,ST_NEXT_ROW=20,ST_DONE=21;
	reg [4:0] state_q,state_next;
	reg [6:0] row_index_q;
	reg scale_q;
	reg [8:0] y_q;
	reg [7:0] roi_x0;
	reg [4:0] tile_row_q,word_q,bit_q;
	reg [31:0] tile [0:12];
	reg [9:0] x_q,start_x_q,end_x_q,center_x_q;
	reg in_run_q;
	reg [7:0] run_count_q,search_start_q,search_end_q;
	reg [19:0] runs [0:159] /* synthesis syn_ramstyle="lut_ram" */;
	reg [19:0] run_data_q;
	reg [8:0] v_y_q,v_limit_q,stroke_start_q;
	reg [6:0] stroke_q,stroke_limit_q;
	reg [31:0] v_low_q,v_high_q;
	reg [3:0] width_min_q,width_max_unused;
	reg [5:0] min_width_q,max_width_q;
	reg [10:0] min_spacing_q,max_spacing_q,previous_center2_q;
	reg [9:0] group_left_q,previous_right_q;
	reg [7:0] best_count_q;
	reg [9:0] best_left_q,best_right_q;
	reg [1:0] active_q;
	reg [8:0] group_first_y [0:1],group_last_y [0:1];
	reg [9:0] group_min_x [0:1],group_max_x [0:1],group_last_left [0:1],group_last_right [0:1];
	reg [6:0] group_order [0:1],group_order_q;
	reg candidate_q,done_q;
	reg [9:0] left_q,right_q;
	reg [8:0] top_q,bottom_q;
	reg [31:0] cycles_q;
	wire [3:0] projection_count;
	wire projection_bit;
	wire [9:0] run_width;
	wire [8:0] half_height,vertical_top,projection_top;
	wire [9:0] query_x;
	wire [11:0] tile_address,vertical_address;
	wire [63:0] vertical_word;
	wire [2:0] vertical_bits;
	wire vertical_on,vertical_bad;
	wire [6:0] next_stroke;
	wire [9:0] candidate_left,candidate_right,candidate_width,candidate_gap;
	wire [10:0] center2,spacing2,new_min_spacing,new_max_spacing;
	wire [5:0] new_min_width,new_max_width;
	wire invalid_group,first_run;
	wire [7:0] candidate_count;
	wire [9:0] candidate_span;
	wire [1:0] matches;
	wire selected_group;
	wire [9:0] overlap0,overlap1,width0,width1,current_width,min_width0,min_width1;
	wire [9:0] intersect_left0,intersect_left1,intersect_right0,intersect_right1;
	integer k;
	assign o_busy=state_q!=ST_IDLE;
	assign o_done=done_q;
	assign o_candidate=candidate_q;
	assign o_left=left_q;assign o_right=right_q;assign o_top=top_q;assign o_bottom=bottom_q;
	assign o_cycles=cycles_q;
	assign o_req=state_q==ST_TILE_REQ || state_q==ST_V_REQ || state_q==ST_V2_REQ;
	assign o_addr=state_q==ST_TILE_REQ ? tile_address : vertical_address;
	assign half_height=scale_q ? 9'd6 : 9'd1;
	assign projection_top=y_q-half_height;
	assign vertical_top=y_q-9'd30;
	assign query_x=center_x_q-1'b1;
	assign tile_address=((projection_top-9'd135+{4'd0,tile_row_q})<<4)+
		((projection_top-9'd135+{4'd0,tile_row_q})<<2)+{7'd0,word_q};
	assign vertical_address=(({3'd0,v_y_q}-12'd135)<<4)+(({3'd0,v_y_q}-12'd135)<<2)+
		{7'd0,query_x[9:5]}+(state_q==ST_V2_REQ ? 12'd1 : 12'd0);
	assign projection_count={3'd0,tile[0][0]}+{3'd0,tile[1][0]}+{3'd0,tile[2][0]}+
		(scale_q ? ({3'd0,tile[3][0]}+{3'd0,tile[4][0]}+{3'd0,tile[5][0]}+
		{3'd0,tile[6][0]}+{3'd0,tile[7][0]}+{3'd0,tile[8][0]}+{3'd0,tile[9][0]}+
		{3'd0,tile[10][0]}+{3'd0,tile[11][0]}+{3'd0,tile[12][0]}) : 4'd0);
	assign projection_bit=x_q>={2'd0,roi_x0} && x_q<10'd640-{2'd0,roi_x0} &&
		projection_count>=(scale_q ? 4'd3 : 4'd2);
	assign run_width=x_q-start_x_q;
	assign vertical_word={v_high_q,v_low_q};
	assign vertical_bits=vertical_word >> query_x[4:0];
	assign vertical_on=(vertical_bits[0] && vertical_bits[1]) ||
		(vertical_bits[0] && vertical_bits[2]) || (vertical_bits[1] && vertical_bits[2]);
	assign next_stroke=vertical_on ? stroke_q+1'b1 : stroke_q;
	// Early rejection is safe once an excessive stroke already intersects the band.
	assign vertical_bad=next_stroke>stroke_limit_q && stroke_start_q<=y_q+half_height &&
		(v_y_q+(vertical_on ? 9'd1 : 9'd0))>=y_q-half_height;
	assign candidate_left=run_data_q[9:0];
	assign candidate_right=run_data_q[19:10];
	assign candidate_width=candidate_right-candidate_left;
	assign candidate_gap=candidate_left-previous_right_q;
	assign center2={1'b0,candidate_left}+{1'b0,candidate_right};
	assign spacing2=center2-previous_center2_q;
	assign first_run=search_end_q==search_start_q;
	assign new_min_width=first_run || candidate_width<min_width_q ? candidate_width[5:0] : min_width_q;
	assign new_max_width=first_run || candidate_width>max_width_q ? candidate_width[5:0] : max_width_q;
	assign new_min_spacing=search_end_q==search_start_q+1 || spacing2<min_spacing_q ? spacing2 : min_spacing_q;
	assign new_max_spacing=search_end_q==search_start_q+1 || spacing2>max_spacing_q ? spacing2 : max_spacing_q;
	assign invalid_group=(!first_run && (candidate_left<previous_right_q || candidate_gap<3 || candidate_gap>55)) ||
		({2'd0,new_max_width}>{2'd0,new_min_width}+({2'd0,new_min_width}<<1)) ||
		(!first_run && (({4'd0,new_max_spacing}<<2)+{4'd0,new_max_spacing}>
		({4'd0,new_min_spacing}<<3)+{4'd0,new_min_spacing}));
	assign candidate_count=search_end_q-search_start_q+1'b1;
	assign candidate_span=candidate_right-(first_run ? candidate_left : group_left_q);
	assign current_width=best_right_q-best_left_q;
	assign width0=group_last_right[0]-group_last_left[0];
	assign width1=group_last_right[1]-group_last_left[1];
	assign min_width0=current_width<width0 ? current_width : width0;
	assign min_width1=current_width<width1 ? current_width : width1;
	assign intersect_left0=best_left_q>group_last_left[0] ? best_left_q : group_last_left[0];
	assign intersect_left1=best_left_q>group_last_left[1] ? best_left_q : group_last_left[1];
	assign intersect_right0=best_right_q<group_last_right[0] ? best_right_q : group_last_right[0];
	assign intersect_right1=best_right_q<group_last_right[1] ? best_right_q : group_last_right[1];
	assign overlap0=intersect_right0>=intersect_left0 ? intersect_right0-intersect_left0 : 10'd0;
	assign overlap1=intersect_right1>=intersect_left1 ? intersect_right1-intersect_left1 : 10'd0;
	assign matches[0]=active_q[0] && y_q>group_last_y[0] && y_q-group_last_y[0]<=4 &&
		(({3'd0,overlap0}<<2)+{3'd0,overlap0})>=(({3'd0,min_width0}<<1)+{3'd0,min_width0});
	assign matches[1]=active_q[1] && y_q>group_last_y[1] && y_q-group_last_y[1]<=4 &&
		(({3'd0,overlap1}<<2)+{3'd0,overlap1})>=(({3'd0,min_width1}<<1)+{3'd0,min_width1});
	assign selected_group=matches==3 ? group_order[1]>group_order[0] :
		matches[1] ? 1'b1 : matches[0] ? 1'b0 :
		!active_q[0] ? 1'b0 : !active_q[1] ? 1'b1 : group_last_y[1]<group_last_y[0];
	// x0=floor(140-(y-165)*100/140); row index is0..69, no divider.
	always @(*) begin
		case(row_index_q)
`include "video/host_f14_crosswalk_roi.vh"
			default: roi_x0=8'd140;
		endcase
	end
	always @(posedge i_clk) begin
		if(state_q==ST_TILE_WAIT && i_read_valid) begin tile[tile_row_q]<=i_read_data; end
		if(state_q==ST_SHIFT) begin
			for(k=0;k<13;k=k+1) begin tile[k]<=tile[k]>>1; end
		end
		if(state_q==ST_APPEND) begin runs[run_count_q]<={end_x_q,start_x_q}; end
		if(state_q==ST_RUN_REQ) begin run_data_q<=runs[search_end_q]; end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin state_q<=ST_IDLE; end
		else begin state_q<=state_next; end
	end
	always @(*) begin
		state_next=state_q;
		case(state_q)
			ST_IDLE: if(i_start) begin state_next=ST_ROW; end
			ST_ROW: state_next=ST_TILE_REQ;
			ST_TILE_REQ: state_next=ST_TILE_WAIT;
			ST_TILE_WAIT: if(i_read_valid) begin state_next=tile_row_q==(scale_q ? 12 : 2) ? ST_SHIFT : ST_TILE_REQ; end
			ST_SHIFT: begin
				if(in_run_q && !projection_bit && run_width>=3 && run_width<=45) begin state_next=ST_V_INIT; end
				else if(x_q==639) begin state_next=ST_SEARCH_INIT; end
				else if(bit_q==31) begin state_next=ST_TILE_REQ; end
			end
			ST_V_INIT: state_next=ST_V_REQ;
			ST_V_REQ: state_next=ST_V_WAIT;
			ST_V_WAIT: if(i_read_valid) begin state_next=query_x[4:0]>29 ? ST_V2_REQ : ST_V_BIT; end
			ST_V2_REQ: state_next=ST_V2_WAIT;
			ST_V2_WAIT: if(i_read_valid) begin state_next=ST_V_BIT; end
			ST_V_BIT: begin
				if(vertical_bad) begin state_next=bit_q==0 ? ST_TILE_REQ : ST_SHIFT; end
				else begin state_next=v_y_q==v_limit_q ? ST_APPEND : ST_V_REQ; end
			end
			ST_APPEND: state_next=bit_q==0 ? ST_TILE_REQ : ST_SHIFT;
			ST_SEARCH_INIT: state_next=run_count_q>=5 ? ST_RUN_REQ : ST_SCALE_END;
			ST_RUN_REQ: state_next=ST_RUN_WAIT;
			ST_RUN_WAIT: state_next=ST_RUN_CHECK;
			ST_RUN_CHECK: state_next=invalid_group ? ST_NEXT_START : ST_RUN_ACCEPT;
			ST_RUN_ACCEPT: state_next=search_end_q+1<run_count_q ? ST_RUN_REQ : ST_NEXT_START;
			ST_NEXT_START: state_next=run_count_q-search_start_q-1>=5 &&
				run_count_q-search_start_q-1>=best_count_q ? ST_RUN_REQ : ST_SCALE_END;
			ST_SCALE_END: state_next=scale_q ? ST_GROUP : ST_ROW;
			ST_GROUP: state_next=ST_NEXT_ROW;
			ST_NEXT_ROW: state_next=row_index_q==69 ? ST_DONE : ST_ROW;
			ST_DONE: state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			row_index_q<=0;scale_q<=0;y_q<=165;tile_row_q<=0;word_q<=0;bit_q<=0;
			x_q<=0;start_x_q<=0;end_x_q<=0;center_x_q<=0;in_run_q<=0;
			run_count_q<=0;search_start_q<=0;search_end_q<=0;
			v_y_q<=0;v_limit_q<=0;stroke_start_q<=0;stroke_q<=0;stroke_limit_q<=0;
			v_low_q<=0;v_high_q<=0;min_width_q<=0;max_width_q<=0;
			min_spacing_q<=0;max_spacing_q<=0;previous_center2_q<=0;group_left_q<=0;previous_right_q<=0;
			best_count_q<=0;best_left_q<=0;best_right_q<=0;active_q<=0;group_order_q<=0;
			candidate_q<=0;done_q<=0;left_q<=0;right_q<=0;top_q<=0;bottom_q<=0;cycles_q<=0;
			for(k=0;k<2;k=k+1) begin
				group_first_y[k]<=0;group_last_y[k]<=0;group_min_x[k]<=0;group_max_x[k]<=0;
				group_last_left[k]<=0;group_last_right[k]<=0;group_order[k]<=0;
			end
		end else begin
			done_q<=0;
			if(state_q!=ST_IDLE && state_q!=ST_DONE) begin cycles_q<=cycles_q+1'b1; end
			case(state_q)
				ST_IDLE: if(i_start) begin
					row_index_q<=0;y_q<=165;scale_q<=0;best_count_q<=0;active_q<=0;group_order_q<=0;
					candidate_q<=0;cycles_q<=0;left_q<=0;right_q<=0;top_q<=0;bottom_q<=0;
				end
				ST_ROW: begin
					tile_row_q<=0;word_q<=0;bit_q<=0;x_q<=0;in_run_q<=0;run_count_q<=0;
				end
				ST_TILE_WAIT: if(i_read_valid) begin
					if(tile_row_q==(scale_q ? 12 : 2)) begin tile_row_q<=0; end
					else begin tile_row_q<=tile_row_q+1'b1; end
				end
				ST_SHIFT: begin
					if(projection_bit && !in_run_q) begin start_x_q<=x_q; end
					in_run_q<=projection_bit;
					if(in_run_q && !projection_bit) begin end_x_q<=x_q; end
					x_q<=x_q+1'b1;bit_q<=bit_q+1'b1;
					if(bit_q==31) begin word_q<=word_q+1'b1; end
				end
				ST_V_INIT: begin
					center_x_q<=({1'b0,start_x_q}+{1'b0,end_x_q})>>1;
					v_y_q<=vertical_top;v_limit_q<=y_q>=289 ? 9'd319 : y_q+9'd30;
					stroke_q<=0;stroke_start_q<=vertical_top;
					stroke_limit_q<=end_x_q-start_x_q>4 ? (end_x_q-start_x_q)<<1 : 7'd8;
				end
				ST_V_WAIT: if(i_read_valid) begin v_low_q<=i_read_data;v_high_q<=0;end
				ST_V2_WAIT: if(i_read_valid) begin v_high_q<=i_read_data;end
				ST_V_BIT: begin
					if(vertical_on) begin
						stroke_q<=stroke_q+1'b1;
						if(stroke_q==0) begin stroke_start_q<=v_y_q; end
					end else begin stroke_q<=0;stroke_start_q<=v_y_q+1'b1;end
					v_y_q<=v_y_q+1'b1;
				end
				ST_APPEND: run_count_q<=run_count_q+1'b1;
				ST_SEARCH_INIT: begin search_start_q<=0;search_end_q<=0;end
				ST_RUN_ACCEPT: begin
					min_width_q<=new_min_width;max_width_q<=new_max_width;
					min_spacing_q<=new_min_spacing;max_spacing_q<=new_max_spacing;
					previous_center2_q<=center2;previous_right_q<=candidate_right;
					if(first_run) begin group_left_q<=candidate_left;end
					if(candidate_count>=5 && candidate_span>=90 &&
						(first_run ? candidate_left : group_left_q)<=360 && candidate_right>=280 &&
						(candidate_count>best_count_q || (candidate_count==best_count_q && candidate_span>current_width))) begin
						best_count_q<=candidate_count;best_left_q<=group_left_q;best_right_q<=candidate_right;
					end
					search_end_q<=search_end_q+1'b1;
				end
				ST_NEXT_START: begin search_start_q<=search_start_q+1'b1;search_end_q<=search_start_q+1'b1;end
				ST_SCALE_END: begin scale_q<=!scale_q;end
				ST_GROUP: if(best_count_q!=0) begin
					active_q[selected_group]<=1;
					group_last_y[selected_group]<=y_q;group_last_left[selected_group]<=best_left_q;group_last_right[selected_group]<=best_right_q;
					if(|matches) begin
						group_min_x[selected_group]<=best_left_q<group_min_x[selected_group] ? best_left_q : group_min_x[selected_group];
						group_max_x[selected_group]<=best_right_q>group_max_x[selected_group] ? best_right_q : group_max_x[selected_group];
						candidate_q<=1;top_q<=group_first_y[selected_group]-1'b1;bottom_q<=y_q+9'd2;
						left_q<=best_left_q<group_min_x[selected_group] ? best_left_q : group_min_x[selected_group];
						right_q<=best_right_q>group_max_x[selected_group] ? best_right_q : group_max_x[selected_group];
					end else begin
						group_first_y[selected_group]<=y_q;group_min_x[selected_group]<=best_left_q;group_max_x[selected_group]<=best_right_q;
						group_order[selected_group]<=group_order_q;group_order_q<=group_order_q+1'b1;
					end
				end
				ST_NEXT_ROW: begin
					row_index_q<=row_index_q+1'b1;y_q<=y_q+9'd2;best_count_q<=0;
				end
				ST_DONE: done_q<=1;
				default: begin end
			endcase
		end
	end
endmodule
`default_nettype wire
