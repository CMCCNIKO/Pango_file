//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 在串行车道几何筛选和 Q16 时间平滑中显式接收帧连续性标志 i_contiguous，避免不连续输入沿用错误的历史轨迹。
// Module Name: host_lane_postprocess_track, host_lane_postprocess_track_store
// File Name: lane/rtl/host_lane_postprocess_track.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 在串行车道几何筛选和 Q16 时间平滑中显式接收帧连续性标志 i_contiguous，避免不连续输入沿用错误的历史轨迹。
// 除最终四车道坐标/有效/缺口/类型表，还逐点发布空间筛选结果（lane、row、x、kept、segment），供后续分类/共识模块并行消费；同文件 track_store 保存工作及历史数据。
//
// Dependencies: 同文件 host_lane_postprocess_track_store；上游提供 i_contiguous
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 保留原文件生成来源说明；仅说明整理，不更改帧连续性、历史状态或逐点输出拍数。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 与 residual035 预览对应的串行几何/跳变/时间平滑。
// Q16 时间坐标，缺失点不保留；o_edge_table 保护被拒绝的缺口。
// Width experiment; reconstructed by tools/prepare_width_trial.py from v5. Original generator: tools/generate_postprocess.py; edit generator, not this file.
`timescale 1ns/1ps
`default_nettype none
module host_lane_postprocess_track (
 input wire i_clk, input wire i_rst_n, input wire i_start,
 input wire [839:0] i_x_table, input wire [83:0] i_valid_table,
 input wire [7:0] i_type_table,
 input wire i_contiguous,
 output wire o_spatial_valid,
 output wire [1:0] o_spatial_lane,
 output wire [4:0] o_spatial_row,
 output wire [9:0] o_spatial_x,
 output wire o_spatial_kept,
 output wire [4:0] o_spatial_segment,
 output reg o_done, output reg [839:0] o_x_table,
 output reg [83:0] o_valid_table, output reg [83:0] o_edge_table,
 output reg [7:0] o_type_table
);
 reg [8:0] state_q;
reg signed [31:0] product0;
reg signed [31:0] product1;
reg signed [31:0] product2;
reg signed [31:0] product3;
reg signed [31:0] product4;
reg signed [31:0] product5;


 reg [31:0] mul_a,mul_b,mul_acc;
 wire [31:0] mul_sum=mul_acc+(mul_b[0] ? mul_a : 32'd0);
 reg signed [31:0] mul_result;
 reg [5:0] mul_left;
 reg [8:0] mul_return_q;
 reg blend_negative;
 reg [31:0] blend_shift,blend_quotient;
 reg [3:0] blend_remainder;
 wire [4:0] blend_trial={blend_remainder,blend_shift[31]};
 wire blend_take=blend_trial>=5'd10;
 reg [5:0] blend_left;

 reg [1:0] lane;
 reg [6:0] base;
 integer i,j,n,pos,w,a,b,c,d,u,v,h0,h1,h2,span,dy0,dy1;
 integer dx0,dx1,dx2,r0,r1,peak,peak_span,best_peak,best_span;
 integer best_pos,best_w,removed,reverse_tip,tip_pass,edge_mode;
 integer run_start,run_n,segment,previous_id,sort_n,sort_i,sort_j;
 integer motion2,med2,agree,coherent,lane_reset,delta,need,lo,hi;
 reg known[0:83];
 reg [2:0] missing[0:83];
 reg [1:0] count[0:83];
 reg active[0:83];
 reg signed [31:0] value_q,change_q,median_q;
 reg signed [63:0] wide_q;
 reg valid[0:20],blocked[0:20],kept[0:20],accepted[0:20];
 reg signed [5:0] ids[0:20],sid[0:20];
 reg signed [9:0] ys[0:20];
 reg [9:0] rounded[0:20];
 integer k;
 reg [6:0] init_q;
 function integer mag;
  input integer x; begin mag=x<0 ? -x : x; end
 endfunction
 // ADS does not map signed constant division.  This exact reciprocal and
 // correction operate on magnitude; the sign is restored for truncation to 0.
 // The caller bounds |wide_q| <= 18*65536*7, safely inside signed 32 bits.
	localparam [8:0] ST_LOAD_RAW = 9'd42;
	localparam [8:0] ST_LOAD_VALID = 9'd43;
	localparam [8:0] ST_MUL_START = 9'd44;
	localparam [8:0] ST_MUL_STEP = 9'd45;
	localparam [8:0] ST_BLEND_DIV = 9'd46;
	localparam [8:0] ST_BLEND_SLOW = 9'd47;
	localparam [8:0] ST_BLEND_FINISH = 9'd48;
	localparam [8:0] ST_MATH_CAND_RESID_0 = 9'd49;
	localparam [8:0] ST_MATH_CAND_RESID_0_SAVE = 9'd50;
	localparam [8:0] ST_MATH_CAND_RESID_1 = 9'd51;
	localparam [8:0] ST_MATH_CAND_RESID_1_SAVE = 9'd52;
	localparam [8:0] ST_MATH_CAND_RESID_2 = 9'd53;
	localparam [8:0] ST_MATH_CAND_RESID_2_SAVE = 9'd54;
	localparam [8:0] ST_MATH_CAND_RESID_3 = 9'd55;
	localparam [8:0] ST_MATH_CAND_RESID_3_SAVE = 9'd56;
	localparam [8:0] ST_MATH_CAND_TEST_0 = 9'd57;
	localparam [8:0] ST_MATH_CAND_TEST_0_SAVE = 9'd58;
	localparam [8:0] ST_MATH_CAND_TEST_1 = 9'd59;
	localparam [8:0] ST_MATH_CAND_TEST_1_SAVE = 9'd60;
	localparam [8:0] ST_MATH_CAND_TEST_2 = 9'd61;
	localparam [8:0] ST_MATH_CAND_TEST_2_SAVE = 9'd62;
	localparam [8:0] ST_MATH_CAND_KEEP_0 = 9'd63;
	localparam [8:0] ST_MATH_CAND_KEEP_0_SAVE = 9'd64;
	localparam [8:0] ST_MATH_CAND_KEEP_1 = 9'd65;
	localparam [8:0] ST_MATH_CAND_KEEP_1_SAVE = 9'd66;
	localparam [8:0] ST_MATH_TIP_TEST_0 = 9'd67;
	localparam [8:0] ST_MATH_TIP_TEST_0_SAVE = 9'd68;
	localparam [8:0] ST_MATH_TIP_TEST_1 = 9'd69;
	localparam [8:0] ST_MATH_TIP_TEST_1_SAVE = 9'd70;
	localparam [8:0] ST_MATH_TIP_TEST_2 = 9'd71;
	localparam [8:0] ST_MATH_TIP_TEST_2_SAVE = 9'd72;
	localparam [8:0] ST_MATH_TIP_TEST_3 = 9'd73;
	localparam [8:0] ST_MATH_TIP_TEST_3_SAVE = 9'd74;
	localparam [8:0] ST_MATH_TIP_TEST_4 = 9'd75;
	localparam [8:0] ST_MATH_TIP_TEST_4_SAVE = 9'd76;
	localparam [8:0] ST_MATH_TIP_TEST_5 = 9'd77;
	localparam [8:0] ST_MATH_TIP_TEST_5_SAVE = 9'd78;
	localparam [8:0] ST_IDLE = 9'd0;
	localparam [8:0] ST_LIST = 9'd1;
	localparam [8:0] ST_LIST_SCAN = 9'd2;
	localparam [8:0] ST_CAND_INIT = 9'd3;
	localparam [8:0] ST_CAND_LOOP = 9'd4;
	localparam [8:0] ST_CAND_LOAD = 9'd5;
	localparam [8:0] ST_CAND_RESID = 9'd6;
	localparam [8:0] ST_CAND_TEST = 9'd7;
	localparam [8:0] ST_CAND_KEEP = 9'd8;
	localparam [8:0] ST_REMOVE = 9'd9;
	localparam [8:0] ST_TIP_INIT = 9'd10;
	localparam [8:0] ST_TIP_LOAD = 9'd11;
	localparam [8:0] ST_TIP_TEST = 9'd12;
	localparam [8:0] ST_TIP_RELIST = 9'd13;
	localparam [8:0] ST_GEOM_RUN_INIT = 9'd14;
	localparam [8:0] ST_GEOM_RUN = 9'd15;
	localparam [8:0] ST_GEOM_MARK = 9'd16;
	localparam [8:0] ST_GEOM_RUN_NEXT = 9'd17;
	localparam [8:0] ST_GATE_SORT_LOAD = 9'd18;
	localparam [8:0] ST_SORT_INIT = 9'd19;
	localparam [8:0] ST_SORT_LOOP = 9'd20;
	localparam [8:0] ST_MEDIAN = 9'd21;
	localparam [8:0] ST_GATE_INIT = 9'd22;
	localparam [8:0] ST_GATE_APPLY = 9'd23;
	localparam [8:0] ST_GATE_DECIDE = 9'd24;
	localparam [8:0] ST_GATE_CONFIRM = 9'd25;
	localparam [8:0] ST_GATE_NEXT = 9'd26;
	localparam [8:0] ST_CLEAN_RUN = 9'd27;
	localparam [8:0] ST_CLEAN_MARK = 9'd28;
	localparam [8:0] ST_CLEAN_NEXT = 9'd29;
	localparam [8:0] ST_TEMP_SORT_LOAD = 9'd30;
	localparam [8:0] ST_TEMP_MOTION = 9'd31;
	localparam [8:0] ST_TEMP_AGREE = 9'd32;
	localparam [8:0] ST_TEMP_COHERENT = 9'd33;
	localparam [8:0] ST_TEMP_APPLY_INIT = 9'd34;
	localparam [8:0] ST_TEMP_APPLY = 9'd35;
	localparam [8:0] ST_TEMP_BLEND = 9'd36;
	localparam [8:0] ST_TEMP_CLAMP = 9'd37;
	localparam [8:0] ST_TEMP_STORE = 9'd38;
	localparam [8:0] ST_TEMP_NEXT = 9'd39;
	localparam [8:0] ST_LANE_LOAD = 9'd40;
	localparam [8:0] ST_DONE = 9'd41;
	localparam [8:0] ST_RF_CAND_LOAD_0 = 9'd79;
	localparam [8:0] ST_RF_CAND_LOAD_0_WAIT = 9'd80;
	localparam [8:0] ST_RF_CAND_LOAD_1 = 9'd81;
	localparam [8:0] ST_RF_CAND_LOAD_1_WAIT = 9'd82;
	localparam [8:0] ST_RF_CAND_LOAD_2 = 9'd83;
	localparam [8:0] ST_RF_CAND_LOAD_2_WAIT = 9'd84;
	localparam [8:0] ST_RF_CAND_LOAD_3 = 9'd85;
	localparam [8:0] ST_RF_CAND_LOAD_3_WAIT = 9'd86;
	localparam [8:0] ST_RF_TIP_LOAD_0 = 9'd87;
	localparam [8:0] ST_RF_TIP_LOAD_0_WAIT = 9'd88;
	localparam [8:0] ST_RF_TIP_LOAD_1 = 9'd89;
	localparam [8:0] ST_RF_TIP_LOAD_1_WAIT = 9'd90;
	localparam [8:0] ST_RF_TIP_LOAD_2 = 9'd91;
	localparam [8:0] ST_RF_TIP_LOAD_2_WAIT = 9'd92;
	localparam [8:0] ST_RF_TIP_LOAD_3 = 9'd93;
	localparam [8:0] ST_RF_TIP_LOAD_3_WAIT = 9'd94;
	localparam [8:0] ST_RF_GATE_SORT_LOAD_0 = 9'd95;
	localparam [8:0] ST_RF_GATE_SORT_LOAD_0_WAIT = 9'd96;
	localparam [8:0] ST_RF_GATE_SORT_LOAD_1 = 9'd97;
	localparam [8:0] ST_RF_GATE_SORT_LOAD_1_WAIT = 9'd98;
	localparam [8:0] ST_RF_SORT_LOOP_0 = 9'd99;
	localparam [8:0] ST_RF_SORT_LOOP_0_WAIT = 9'd100;
	localparam [8:0] ST_RF_SORT_LOOP_1 = 9'd101;
	localparam [8:0] ST_RF_SORT_LOOP_1_WAIT = 9'd102;
	localparam [8:0] ST_RF_MEDIAN_0 = 9'd103;
	localparam [8:0] ST_RF_MEDIAN_0_WAIT = 9'd104;
	localparam [8:0] ST_RF_MEDIAN_1 = 9'd105;
	localparam [8:0] ST_RF_MEDIAN_1_WAIT = 9'd106;
	localparam [8:0] ST_RF_GATE_APPLY_0 = 9'd107;
	localparam [8:0] ST_RF_GATE_APPLY_0_WAIT = 9'd108;
	localparam [8:0] ST_RF_GATE_APPLY_1 = 9'd109;
	localparam [8:0] ST_RF_GATE_APPLY_1_WAIT = 9'd110;
	localparam [8:0] ST_RF_GATE_APPLY_2 = 9'd111;
	localparam [8:0] ST_RF_GATE_APPLY_2_WAIT = 9'd112;
	localparam [8:0] ST_RF_GATE_APPLY_3 = 9'd113;
	localparam [8:0] ST_RF_GATE_APPLY_3_WAIT = 9'd114;
	localparam [8:0] ST_RF_GATE_DECIDE_0 = 9'd115;
	localparam [8:0] ST_RF_GATE_DECIDE_0_WAIT = 9'd116;
	localparam [8:0] ST_RF_GATE_DECIDE_1 = 9'd117;
	localparam [8:0] ST_RF_GATE_DECIDE_1_WAIT = 9'd118;
	localparam [8:0] ST_RF_GATE_CONFIRM_0 = 9'd119;
	localparam [8:0] ST_RF_GATE_CONFIRM_0_WAIT = 9'd120;
	localparam [8:0] ST_RF_TEMP_SORT_LOAD_0 = 9'd121;
	localparam [8:0] ST_RF_TEMP_SORT_LOAD_0_WAIT = 9'd122;
	localparam [8:0] ST_RF_TEMP_SORT_LOAD_1 = 9'd123;
	localparam [8:0] ST_RF_TEMP_SORT_LOAD_1_WAIT = 9'd124;
	localparam [8:0] ST_RF_TEMP_AGREE_0 = 9'd125;
	localparam [8:0] ST_RF_TEMP_AGREE_0_WAIT = 9'd126;
	localparam [8:0] ST_RF_TEMP_AGREE_1 = 9'd127;
	localparam [8:0] ST_RF_TEMP_AGREE_1_WAIT = 9'd128;
	localparam [8:0] ST_RF_TEMP_APPLY_0 = 9'd129;
	localparam [8:0] ST_RF_TEMP_APPLY_0_WAIT = 9'd130;
	localparam [8:0] ST_RF_TEMP_APPLY_1 = 9'd131;
	localparam [8:0] ST_RF_TEMP_APPLY_1_WAIT = 9'd132;
	localparam [8:0] ST_RF_TEMP_CLAMP_0 = 9'd133;
	localparam [8:0] ST_RF_TEMP_CLAMP_0_WAIT = 9'd134;
	localparam [8:0] ST_RF_LOAD_VALID_0 = 9'd135;
	localparam [8:0] ST_RF_LOAD_VALID_0_WAIT = 9'd136;
	localparam [8:0] ST_RF_BLEND_FINISH_0 = 9'd137;
	localparam [8:0] ST_RF_BLEND_FINISH_0_WAIT = 9'd138;
	localparam [8:0] ST_RF_MATH_CAND_RESID_0_0 = 9'd139;
	localparam [8:0] ST_RF_MATH_CAND_RESID_0_0_WAIT = 9'd140;
	localparam [8:0] ST_RF_MATH_CAND_RESID_0_1 = 9'd141;
	localparam [8:0] ST_RF_MATH_CAND_RESID_0_1_WAIT = 9'd142;
	localparam [8:0] ST_RF_MATH_CAND_RESID_1_0 = 9'd143;
	localparam [8:0] ST_RF_MATH_CAND_RESID_1_0_WAIT = 9'd144;
	localparam [8:0] ST_RF_MATH_CAND_RESID_1_1 = 9'd145;
	localparam [8:0] ST_RF_MATH_CAND_RESID_1_1_WAIT = 9'd146;
	localparam [8:0] ST_RF_MATH_CAND_RESID_2_0 = 9'd147;
	localparam [8:0] ST_RF_MATH_CAND_RESID_2_0_WAIT = 9'd148;
	localparam [8:0] ST_RF_MATH_CAND_RESID_2_1 = 9'd149;
	localparam [8:0] ST_RF_MATH_CAND_RESID_2_1_WAIT = 9'd150;
	localparam [8:0] ST_RF_MATH_CAND_RESID_3_0 = 9'd151;
	localparam [8:0] ST_RF_MATH_CAND_RESID_3_0_WAIT = 9'd152;
	localparam [8:0] ST_RF_MATH_CAND_RESID_3_1 = 9'd153;
	localparam [8:0] ST_RF_MATH_CAND_RESID_3_1_WAIT = 9'd154;
	localparam [8:0] ST_RAM_COMMIT0 = 9'd155;
	localparam [8:0] ST_RAM_COMMIT1 = 9'd156;
reg [8:0] ram_return_q;
reg [6:0] raw_ra;
wire signed [10:0] raw_q;
reg raw_we;
reg [6:0] raw_wa;
reg signed [10:0] raw_wd;
reg signed [10:0] raw_r0;
reg signed [10:0] raw_r1;
reg signed [10:0] raw_r2;
reg signed [10:0] raw_r3;
reg raw_we0;
reg [6:0] raw_wa0;
reg signed [10:0] raw_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(11),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_raw(i_clk,i_rst_n,raw_we,raw_wa,raw_wd,raw_ra,raw_q);
reg [6:0] last_ra;
wire signed [10:0] last_q;
reg last_we;
reg [6:0] last_wa;
reg signed [10:0] last_wd;
reg signed [10:0] last_r0;
reg last_we0;
reg [6:0] last_wa0;
reg signed [10:0] last_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(11),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_last(i_clk,i_rst_n,last_we,last_wa,last_wd,last_ra,last_q);
reg [6:0] pending_ra;
wire signed [10:0] pending_q;
reg pending_we;
reg [6:0] pending_wa;
reg signed [10:0] pending_wd;
reg signed [10:0] pending_r0;
reg pending_we0;
reg [6:0] pending_wa0;
reg signed [10:0] pending_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(11),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_pending(i_clk,i_rst_n,pending_we,pending_wa,pending_wd,pending_ra,pending_q);
reg [6:0] low_ra;
wire signed [10:0] low_q;
reg low_we;
reg [6:0] low_wa;
reg signed [10:0] low_wd;
reg signed [10:0] low_r0;
reg low_we0;
reg [6:0] low_wa0;
reg signed [10:0] low_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(11),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_low(i_clk,i_rst_n,low_we,low_wa,low_wd,low_ra,low_q);
reg [6:0] high_ra;
wire signed [10:0] high_q;
reg high_we;
reg [6:0] high_wa;
reg signed [10:0] high_wd;
reg signed [10:0] high_r0;
reg high_we0;
reg [6:0] high_wa0;
reg signed [10:0] high_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(11),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_high(i_clk,i_rst_n,high_we,high_wa,high_wd,high_ra,high_q);
reg [6:0] temporal_ra;
wire signed [31:0] temporal_q;
reg temporal_we;
reg [6:0] temporal_wa;
reg signed [31:0] temporal_wd;
reg signed [31:0] temporal_r0;
reg temporal_we0;
reg [6:0] temporal_wa0;
reg signed [31:0] temporal_wd0;
host_lane_postprocess_track_store #(.C_WIDTH(32),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(1)) u_ram_temporal(i_clk,i_rst_n,temporal_we,temporal_wa,temporal_wd,temporal_ra,temporal_q);
reg [4:0] ordered_ra;
wire signed [31:0] ordered_q;
reg ordered_we;
reg [4:0] ordered_wa;
reg signed [31:0] ordered_wd;
reg signed [31:0] ordered_r0;
reg signed [31:0] ordered_r1;
reg ordered_we0;
reg [4:0] ordered_wa0;
reg signed [31:0] ordered_wd0;
reg ordered_we1;
reg [4:0] ordered_wa1;
reg signed [31:0] ordered_wd1;
host_lane_postprocess_track_store #(.C_WIDTH(32),.C_DEPTH(21),.C_ADDR(5),.C_CLEAR(0)) u_ram_ordered(i_clk,i_rst_n,ordered_we,ordered_wa,ordered_wd,ordered_ra,ordered_q);
 // TrackGate：小历史RAM+逐组扫描；仅内侧线重捕获，拒绝点仍是缺口。
 localparam [8:0] ST_RQ_INIT=157,ST_RQ_SELECT=158,ST_RQ_SCAN=159,ST_RQ_CHECK=160,
 ST_RQ_MARK=161,ST_RQ_CLEAN=162,ST_RQ_ADDR=163,ST_RQ_WAIT=164,ST_RQ_COMMIT=165,ST_RQ_NEXT=166;
 reg [14:0] rq_previous[0:83] /* synthesis syn_ramstyle="lut_ram" */;
 reg [83:0] rq_known;
 reg [3:0] rq_painted;
 reg [20:0] rq_eligible,rq_mask;
 reg [4:0] rq_previous_sid[0:20] /* synthesis syn_ramstyle="lut_ram" */;
 reg [4:0] rq_select,rq_scan,rq_count,rq_first,rq_last;
 reg [4:0] rq_current_sid,rq_prior_sid;
 reg rq_phase,rq_contiguous;
 wire [14:0] rq_old=rq_previous[base+i];
 wire signed [11:0] rq_delta=$signed(raw_r0)-$signed({1'b0,rq_old[9:0]});
 wire signed [11:0] rq_last_delta=$signed(raw_r0)-$signed(last_r0);
 assign o_spatial_valid=state_q==ST_GATE_SORT_LOAD;
 assign o_spatial_lane=lane;
 assign o_spatial_row=i;
 assign o_spatial_x=raw_r0[9:0];
 assign o_spatial_kept=kept[i];
 assign o_spatial_segment=sid[i][4:0];
 always @(posedge i_clk) if(state_q==ST_GATE_SORT_LOAD) begin
  rq_previous[base+i]<={sid[i][4:0],raw_r0[9:0]};
  rq_previous_sid[i]<=rq_old[14:10];
 end
always @(*) begin
raw_ra=0; raw_we=0; raw_wa=0; raw_wd=0;
last_ra=0; last_we=0; last_wa=0; last_wd=0;
pending_ra=0; pending_we=0; pending_wa=0; pending_wd=0;
low_ra=0; low_we=0; low_wa=0; low_wd=0;
high_ra=0; high_we=0; high_wa=0; high_wd=0;
temporal_ra=0; temporal_we=0; temporal_wa=0; temporal_wd=0;
ordered_ra=0; ordered_we=0; ordered_wa=0; ordered_wd=0;
case(state_q)
ST_RQ_ADDR: raw_ra=base+i;
ST_RF_CAND_LOAD_0: raw_ra = base+b;
ST_RF_CAND_LOAD_1: raw_ra = base+a;
ST_RF_CAND_LOAD_2: raw_ra = base+d;
ST_RF_CAND_LOAD_3: raw_ra = base+c;
ST_RF_TIP_LOAD_0: raw_ra = base+b;
ST_RF_TIP_LOAD_1: raw_ra = base+a;
ST_RF_TIP_LOAD_2: raw_ra = base+c;
ST_RF_TIP_LOAD_3: raw_ra = base+d;
ST_RF_GATE_SORT_LOAD_0: raw_ra = base+i;
ST_RF_GATE_SORT_LOAD_1: last_ra = base+i;
ST_RF_SORT_LOOP_0: ordered_ra = sort_j;
ST_RF_SORT_LOOP_1: ordered_ra = sort_j+1;
ST_RF_MEDIAN_0: ordered_ra = sort_n>>1;
ST_RF_MEDIAN_1: ordered_ra = (sort_n>>1)-1;
ST_RF_GATE_APPLY_0: raw_ra = base+i;
ST_RF_GATE_APPLY_1: last_ra = base+i;
ST_RF_GATE_APPLY_2: low_ra = base+i;
ST_RF_GATE_APPLY_3: high_ra = base+i;
ST_RF_GATE_DECIDE_0: raw_ra = base+i;
ST_RF_GATE_DECIDE_1: pending_ra = base+i;
ST_RF_GATE_CONFIRM_0: raw_ra = base+i;
ST_RF_TEMP_SORT_LOAD_0: raw_ra = base+i;
ST_RF_TEMP_SORT_LOAD_1: temporal_ra = base+i;
ST_RF_TEMP_AGREE_0: raw_ra = base+i;
ST_RF_TEMP_AGREE_1: temporal_ra = base+i;
ST_RF_TEMP_APPLY_0: raw_ra = base+i;
ST_RF_TEMP_APPLY_1: temporal_ra = base+i;
ST_RF_TEMP_CLAMP_0: raw_ra = base+i;
ST_RF_LOAD_VALID_0: raw_ra = base+i;
ST_RF_BLEND_FINISH_0: temporal_ra = base+i;
ST_RF_MATH_CAND_RESID_0_0: raw_ra = base+b;
ST_RF_MATH_CAND_RESID_0_1: raw_ra = base+a;
ST_RF_MATH_CAND_RESID_1_0: raw_ra = base+d;
ST_RF_MATH_CAND_RESID_1_1: raw_ra = base+a;
ST_RF_MATH_CAND_RESID_2_0: raw_ra = base+c;
ST_RF_MATH_CAND_RESID_2_1: raw_ra = base+a;
ST_RF_MATH_CAND_RESID_3_0: raw_ra = base+d;
ST_RF_MATH_CAND_RESID_3_1: raw_ra = base+a;
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin raw_we=raw_we0; raw_wa=raw_wa0; raw_wd=raw_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin last_we=last_we0; last_wa=last_wa0; last_wd=last_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin pending_we=pending_we0; pending_wa=pending_wa0; pending_wd=pending_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin low_we=low_we0; low_wa=low_wa0; low_wd=low_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin high_we=high_we0; high_wa=high_wa0; high_wd=high_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin temporal_we=temporal_we0; temporal_wa=temporal_wa0; temporal_wd=temporal_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin ordered_we=ordered_we0; ordered_wa=ordered_wa0; ordered_wd=ordered_wd0; end
ST_RAM_COMMIT1: begin ordered_we=ordered_we1; ordered_wa=ordered_wa1; ordered_wd=ordered_wd1; end
default: begin end
endcase
end
	always @(posedge i_clk or negedge i_rst_n) begin
			if (!i_rst_n) begin
raw_we0<=0; last_we0<=0; pending_we0<=0; low_we0<=0; high_we0<=0; temporal_we0<=0; ordered_we0<=0; ordered_we1<=0;
            state_q <= ST_IDLE; lane<=0; base<=0; o_done<=0; rq_known<=0;rq_painted<=0;rq_eligible<=0;rq_mask<=0;rq_phase<=0;rq_contiguous<=0;rq_select<=0;rq_scan<=0;rq_count<=0;rq_first<=0;rq_last<=0;rq_current_sid<=0;rq_prior_sid<=0;
            o_x_table<=0; o_valid_table<=0; o_edge_table<=0; o_type_table<=0;
            for(k=0;k<84;k=k+1) begin
                
                known[k]<=0;missing[k]<=0;count[k]<=0;active[k]<=0;
            end
            ys[0] <= 82;
            ys[1] <= 93;
            ys[2] <= 104;
            ys[3] <= 115;
            ys[4] <= 126;
            ys[5] <= 137;
            ys[6] <= 148;
            ys[7] <= 159;
            ys[8] <= 170;
            ys[9] <= 181;
            ys[10] <= 192;
            ys[11] <= 202;
            ys[12] <= 213;
            ys[13] <= 224;
            ys[14] <= 235;
            ys[15] <= 246;
            ys[16] <= 257;
            ys[17] <= 268;
            ys[18] <= 279;
            ys[19] <= 290;
            ys[20] <= 301;
		end else begin
			o_done <= 0;
if(state_q!=ST_RAM_COMMIT0 && state_q!=ST_RAM_COMMIT1) begin raw_we0<=0; last_we0<=0; pending_we0<=0; low_we0<=0; high_we0<=0; temporal_we0<=0; ordered_we0<=0; ordered_we1<=0; end
			case(state_q)
				ST_IDLE: begin
					if(i_start) begin
					 init_q<=0;rq_contiguous<=i_contiguous;
					 o_valid_table<=0; o_edge_table<=0; o_x_table<=i_x_table;
					 o_type_table<=i_type_table; lane<=0; base<=0;
					 for(k=0;k<21;k=k+1) valid[k]<=i_valid_table[k] && i_x_table[(((32'd0+k)<<3)+((32'd0+k)<<1))+:10]<640;
					 removed<=0; edge_mode<=0;state_q<=ST_LOAD_RAW;
					end

				end
				ST_LIST: begin
					i<=0;n<=0;state_q<=ST_LIST_SCAN;

				end
				ST_LIST_SCAN: begin
					if(valid[i]) begin ids[n]<=i;n<=n+1; end
					if(i==20) state_q<=ST_CAND_INIT; else i<=i+1;

				end
				ST_CAND_INIT: begin
					pos<=1; w<=1; best_pos<=-1;best_w<=0;best_peak<=0;best_span<=1;
					if(edge_mode==1) begin for(k=0;k<21;k=k+1) blocked[k]<=0;end
					if(n<4 && edge_mode==0) begin reverse_tip<=0;tip_pass<=0;state_q<=ST_TIP_INIT;end
					else state_q<=ST_CAND_LOOP;

				end
				ST_CAND_LOOP: begin
					if(pos>=n-w) begin
					 if(w==1) begin w<=2;pos<=1;end
					 else if(edge_mode==1) state_q<=ST_GEOM_RUN_INIT;
					 else if(best_pos>=0) state_q<=ST_REMOVE;
					 else begin reverse_tip<=0;tip_pass<=0;state_q<=ST_TIP_INIT;end
					end else begin
					 a<=ids[pos-1];b<=ids[pos];c<=ids[pos+w-1];d<=ids[pos+w];state_q<=ST_RF_CAND_LOAD_0;
					end

				end
				ST_CAND_LOAD: begin
state_q<=ST_RF_CAND_LOAD_0;
					h0<=ys[b]-ys[a];h1<=ys[d]-ys[c];span<=ys[d]-ys[a];
					dx0<=raw_r0-raw_r1;dx1<=raw_r2-raw_r3;
					if(b-a>2 || d-c>2 || (w==2 && c-b>2)) begin pos<=pos+1;state_q<=ST_CAND_LOOP;end
					else state_q<=ST_RF_MATH_CAND_RESID_0_0;

				end
				ST_CAND_RESID: begin
					r0<=product0-product1;
					r1<=product2-product3;
					state_q<=ST_MATH_CAND_TEST_0;

				end
				ST_CAND_TEST: begin
					if((r0>0 && r1>0 || r0<0 && r1<0) &&
					 ((dx0>0 && dx1<0)||(dx0<0 && dx1>0)) &&
					 ((mag(product0-product1)<<2)+mag(product0-product1))>=product2 &&
					 (mag(r0)>=((span<<3)-span) || mag(r1)>=((span<<3)-span)) && mag(r0)>=4*span && mag(r1)>=4*span) begin
					 peak<=mag(r0)>mag(r1) ? mag(r0) : mag(r1);state_q<=ST_MATH_CAND_KEEP_0;
					end else begin pos<=pos+1;state_q<=ST_CAND_LOOP;end

				end
				ST_CAND_KEEP: begin
					if(edge_mode==1) begin blocked[a]<=1;blocked[c]<=1;end
					else if(removed+w<=4 && (best_pos<0 || product0>product1)) begin
					 best_pos<=pos;best_w<=w;best_peak<=peak;best_span<=span;
					end
					pos<=pos+1;state_q<=ST_CAND_LOOP;

				end
				ST_REMOVE: begin
					valid[ids[best_pos]]<=0;
					if(best_w==2) valid[ids[best_pos+1]]<=0;
					removed<=removed+best_w;state_q<=ST_LIST;

				end
				ST_TIP_INIT: begin
					if(n<4 || removed>=4 || tip_pass>=2) begin
					 if(reverse_tip==0) begin reverse_tip<=1;tip_pass<=0;end
					 else begin edge_mode<=1;state_q<=ST_CAND_INIT;end
					end else begin
					 a<=reverse_tip ? ids[n-4] : ids[3];b<=reverse_tip ? ids[n-3] : ids[2];
					 c<=reverse_tip ? ids[n-2] : ids[1];d<=reverse_tip ? ids[n-1] : ids[0];state_q<=ST_RF_TIP_LOAD_0;
					end

				end
				ST_TIP_LOAD: begin
state_q<=ST_RF_TIP_LOAD_0;
					// Reverse the top endpoint ordering so all spans stay positive.
					h0<=mag(ys[b]-ys[a]);h1<=mag(ys[c]-ys[b]);h2<=mag(ys[d]-ys[c]);
					dx0<=reverse_tip ? raw_r0-raw_r1 : raw_r1-raw_r0;
					dx1<=reverse_tip ? raw_r2-raw_r0 : raw_r0-raw_r2;
					dx2<=reverse_tip ? raw_r3-raw_r2 : raw_r2-raw_r3;
					state_q<=ST_MATH_TIP_TEST_0;

				end
				ST_TIP_TEST: begin
					if(mag(b-a)<=2 && mag(c-b)<=2 && mag(d-c)<=2 &&
					 ((mag(product0-product1)<<2)+mag(product0-product1))<=product2 &&
					 ((dx1>0 && dx2<0)||(dx1<0 && dx2>0)) &&
					 ((mag(product3-product4)<<2)+mag(product3-product4))>=product5 && mag(product4-product3)>=10*h1) begin
					 valid[d]<=0;removed<=removed+1;tip_pass<=tip_pass+1;i<=0;j<=0;state_q<=ST_TIP_RELIST;
					end else begin tip_pass<=2;state_q<=ST_TIP_INIT;end

				end
				ST_TIP_RELIST: begin
					if(valid[i]) begin ids[j]<=i;j<=j+1;end
					if(i==20) begin n<=j+(valid[i]?1:0);state_q<=ST_TIP_INIT;end else i<=i+1;

				end
				ST_GEOM_RUN_INIT: begin
					for(k=0;k<21;k=k+1) begin kept[k]<=0;sid[k]<=-1;end
					pos<=0;run_start<=0;segment<=0;state_q<=ST_GEOM_RUN;

				end
				ST_GEOM_RUN: begin
					if(pos==n || (pos>run_start && (ids[pos]-ids[pos-1]>2 || blocked[ids[pos-1]]))) begin
					 if(pos-run_start>=3) begin i<=run_start;state_q<=ST_GEOM_MARK;end
					 else state_q<=ST_GEOM_RUN_NEXT;
					end else pos<=pos+1;

				end
				ST_GEOM_MARK: begin
					kept[ids[i]]<=1;sid[ids[i]]<=segment;
					if(i+1==pos) state_q<=ST_GEOM_RUN_NEXT;else i<=i+1;

				end
				ST_GEOM_RUN_NEXT: begin
					if(pos==n) begin i<=0;sort_n<=0;state_q<=ST_RF_GATE_SORT_LOAD_0;end
					else begin run_start<=pos;pos<=pos+1;segment<=segment+1;state_q<=ST_GEOM_RUN;end

				end
				ST_GATE_SORT_LOAD: begin
rq_known[base+i]<=kept[i];
rq_eligible[i]<=rq_contiguous && rq_known[base+i] && kept[i] &&
 rq_delta>=-12'sd8 && rq_delta<=12'sd8 &&
 (!known[base+i] || (rq_last_delta>=-12'sd18 && rq_last_delta<=12'sd18));
ram_return_q<=ST_RF_GATE_SORT_LOAD_0;
					if(kept[i] && known[base+i]) begin begin ordered_we0<=1; ordered_wa0<=sort_n; ordered_wd0<=raw_r0-last_r0; end sort_n<=sort_n+1;end
					if(i==20) begin sort_i<=0;sort_j<=0;edge_mode<=0;ram_return_q<=ST_SORT_INIT;end else i<=i+1;

state_q<=ST_RAM_COMMIT0;
				end
				ST_SORT_INIT: begin
					sort_i<=1;sort_j<=0;
					if(sort_n<3) begin motion2<=0;med2<=0;coherent<=0;lane_reset<=0;state_q<=edge_mode==0 ? ST_GATE_INIT : ST_TEMP_APPLY_INIT;end
					else state_q<=ST_RF_SORT_LOOP_0;

				end
				ST_SORT_LOOP: begin
ram_return_q<=ST_RF_SORT_LOOP_0;
					if(sort_i>=sort_n) ram_return_q<=ST_RF_MEDIAN_0;
					else if(sort_j+1>=sort_n) begin sort_i<=sort_i+1;sort_j<=0;end
					else begin
					 if(ordered_r0>ordered_r1) begin begin ordered_we0<=1; ordered_wa0<=sort_j; ordered_wd0<=ordered_r1; end begin ordered_we1<=1; ordered_wa1<=sort_j+1; ordered_wd1<=ordered_r0; end end
					 sort_j<=sort_j+1;
					end

state_q<=ST_RAM_COMMIT0;
				end
				ST_MEDIAN: begin
state_q<=ST_RF_MEDIAN_0;
					median_q<=sort_n[0] ? ordered_r0 : (($signed(ordered_r1)+$signed(ordered_r0)) >>> 1);
					if(edge_mode==0) begin
					 motion2<=sort_n[0] ? ordered_r0*2 : ordered_r1+ordered_r0;state_q<=ST_GATE_INIT;
					end else state_q<=ST_TEMP_MOTION;

				end
				ST_GATE_INIT: begin
rq_phase<=0;
					i<=0;state_q<=ST_RF_GATE_APPLY_0;

				end
				ST_GATE_APPLY: begin
state_q<=ST_RF_GATE_APPLY_0;
					accepted[i]<=0;
					if(!kept[i]) begin
					 if(missing[base+i]<4) missing[base+i]<=missing[base+i]+1'b1;
					 if(missing[base+i]>=3) known[base+i]<=0;
					 count[base+i]<=0;state_q<=ST_GATE_NEXT;
					end else begin
					 missing[base+i]<=0;delta<=raw_r0-last_r0;
					 lo<=raw_r0<low_r0 ? raw_r0 : low_r0;
					 hi<=raw_r0>high_r0 ? raw_r0 : high_r0;state_q<=ST_RF_GATE_DECIDE_0;
					end

				end
				ST_GATE_DECIDE: begin
ram_return_q<=ST_RF_GATE_DECIDE_0;
					if(known[base+i] && mag(delta)<=18 && (lane!=3 || mag(delta*2-motion2)<=20)) begin
					 accepted[i]<=1;begin last_we0<=1; last_wa0<=base+i; last_wd0<=raw_r0; end known[base+i]<=1;count[base+i]<=0;ram_return_q<=ST_GATE_NEXT;
					end else begin
					 need<=known[base+i] || lane==3 ? 3 : 2;
					 if(count[base+i]!=0 && mag(raw_r0-pending_r0)<=8 && (lane!=3 || hi-lo<=12)) begin
					  j<=count[base+i]+1;begin low_we0<=1; low_wa0<=base+i; low_wd0<=lo; end begin high_we0<=1; high_wa0<=base+i; high_wd0<=hi; end 
					 end else begin j<=1;begin low_we0<=1; low_wa0<=base+i; low_wd0<=raw_r0; end begin high_we0<=1; high_wa0<=base+i; high_wd0<=raw_r0; end end
					 begin pending_we0<=1; pending_wa0<=base+i; pending_wd0<=raw_r0; end ram_return_q<=ST_RF_GATE_CONFIRM_0;
					end

state_q<=ST_RAM_COMMIT0;
				end
				ST_GATE_CONFIRM: begin
ram_return_q<=ST_RF_GATE_CONFIRM_0;
					if(j>=need) begin accepted[i]<=1;begin last_we0<=1; last_wa0<=base+i; last_wd0<=raw_r0; end known[base+i]<=1;count[base+i]<=0;end
					else count[base+i]<=j;
					ram_return_q<=ST_GATE_NEXT;

state_q<=ST_RAM_COMMIT0;
				end
				ST_GATE_NEXT: begin
					if(i==20) begin for(k=0;k<21;k=k+1) kept[k]<=0;pos<=0;run_start<=0;state_q<=ST_CLEAN_RUN;end
					else begin i<=i+1;state_q<=ST_RF_GATE_APPLY_0;end

				end
				ST_CLEAN_RUN: begin
					if(pos==n || !accepted[ids[pos]] || sid[ids[pos]]<0 ||
					 (pos>run_start && sid[ids[pos]]!=sid[ids[pos-1]])) begin
					 if(pos-run_start>=3) begin i<=run_start;state_q<=ST_CLEAN_MARK;end
					 else state_q<=ST_CLEAN_NEXT;
					end else pos<=pos+1;

				end
				ST_CLEAN_MARK: begin
					kept[ids[i]]<=1;o_valid_table[base+ids[i]]<=1;
					if(i+1<pos) o_edge_table[base+ids[i]]<=1;
					if(i+1==pos) state_q<=ST_CLEAN_NEXT;else i<=i+1;

				end
				ST_CLEAN_NEXT: begin
					if(pos==n) begin
 i<=0;
 if(lane==1 || lane==2) begin
  if(!rq_phase && !rq_painted[lane])state_q<=ST_RQ_INIT;
  else state_q<=ST_RQ_ADDR;
 end else begin sort_n<=0;state_q<=ST_RF_TEMP_SORT_LOAD_0;end
end
					else if(accepted[ids[pos]] && sid[ids[pos]]>=0) begin run_start<=pos;pos<=pos+1;state_q<=ST_CLEAN_RUN;end
					else begin run_start<=pos+1;pos<=pos+1;state_q<=ST_CLEAN_RUN;end

				end
    ST_RQ_INIT: begin
     accepted[i]<=kept[i];
     if(i==20)begin rq_select<=0;state_q<=ST_RQ_SELECT;end else i<=i+1;
    end
    ST_RQ_SELECT: begin
     rq_current_sid<=sid[rq_select][4:0];rq_prior_sid<=rq_previous_sid[rq_select];
     rq_scan<=0;rq_count<=0;rq_first<=0;rq_last<=0;rq_mask<=0;
     if(rq_eligible[rq_select])state_q<=ST_RQ_SCAN;
     else if(rq_select==20)begin i<=0;state_q<=ST_RQ_CLEAN;end
     else rq_select<=rq_select+1'b1;
    end
    ST_RQ_SCAN: begin
     if(rq_eligible[rq_scan] && sid[rq_scan][4:0]==rq_current_sid && rq_previous_sid[rq_scan]==rq_prior_sid)begin
      rq_mask[rq_scan]<=1;rq_count<=rq_count+1'b1;rq_last<=rq_scan;
      if(rq_count==0)rq_first<=rq_scan;
     end
     if(rq_scan==20)state_q<=ST_RQ_CHECK;else rq_scan<=rq_scan+1'b1;
    end
    ST_RQ_CHECK: begin
     i<=0;
     if(rq_count>=3 && ys[rq_last]-ys[rq_first]>=17)state_q<=ST_RQ_MARK;
     else if(rq_select==20)state_q<=ST_RQ_CLEAN;
     else begin rq_select<=rq_select+1'b1;state_q<=ST_RQ_SELECT;end
    end
    ST_RQ_MARK: begin
     if(rq_mask[i])accepted[i]<=1;
     if(i==20)begin
      if(rq_select==20)begin i<=0;state_q<=ST_RQ_CLEAN;end
      else begin rq_select<=rq_select+1'b1;state_q<=ST_RQ_SELECT;end
     end else i<=i+1;
    end
    ST_RQ_CLEAN: begin
     kept[i]<=0;o_valid_table[base+i]<=0;o_edge_table[base+i]<=0;
     if(i==20)begin rq_phase<=1;pos<=0;run_start<=0;state_q<=ST_CLEAN_RUN;end else i<=i+1;
    end
    ST_RQ_ADDR: state_q<=ST_RQ_WAIT;
    ST_RQ_WAIT: begin raw_r0<=raw_q;state_q<=ST_RQ_COMMIT;end
    ST_RQ_COMMIT: begin
     if(i==0)rq_painted[lane]<=0;
     if(kept[i])begin
      rq_painted[lane]<=1;known[base+i]<=1;count[base+i]<=0;
      last_we0<=1;last_wa0<=base+i;last_wd0<=raw_r0;
     end
     ram_return_q<=ST_RQ_NEXT;state_q<=ST_RAM_COMMIT0;
    end
    ST_RQ_NEXT: begin
     if(i==20)begin i<=0;sort_n<=0;state_q<=ST_RF_TEMP_SORT_LOAD_0;end
     else begin i<=i+1;state_q<=ST_RQ_ADDR;end
    end
				ST_TEMP_SORT_LOAD: begin
ram_return_q<=ST_RF_TEMP_SORT_LOAD_0;
					if(kept[i] && active[base+i]) begin begin ordered_we0<=1; ordered_wa0<=sort_n; ordered_wd0<=raw_r0*65536-temporal_r0; end sort_n<=sort_n+1;end
					if(i==20) begin edge_mode<=1;ram_return_q<=ST_SORT_INIT;end else i<=i+1;

state_q<=ST_RAM_COMMIT0;
				end
				ST_TEMP_MOTION: begin
					lane_reset<=mag(median_q)>25*65536;
					coherent<=0;agree<=0;i<=0;state_q<=ST_RF_TEMP_AGREE_0;

				end
				ST_TEMP_AGREE: begin
state_q<=ST_RF_TEMP_AGREE_0;
					if(kept[i] && active[base+i] &&
					 (raw_r0*65536-temporal_r0>0 && median_q>0 || raw_r0*65536-temporal_r0<0 && median_q<0) &&
					 mag(raw_r0*65536-temporal_r0)>=2*65536) agree<=agree+1;
					if(i==20) state_q<=ST_TEMP_COHERENT; else i<=i+1;

				end
				ST_TEMP_COHERENT: begin
					coherent<=mag(median_q)>=4*65536 && (((32'd0+agree)<<3)+((32'd0+agree)<<1))>=((sort_n<<3)-sort_n) && !lane_reset;
					state_q<=ST_TEMP_APPLY_INIT;

				end
				ST_TEMP_APPLY_INIT: begin
					i<=0;state_q<=ST_RF_TEMP_APPLY_0;

				end
				ST_TEMP_APPLY: begin
state_q<=ST_RF_TEMP_APPLY_0;
					if(!kept[i]) begin active[base+i]<=0;state_q<=ST_TEMP_NEXT;end
					else begin
					 change_q<=raw_r0*65536-temporal_r0;
					 if(!active[base+i] || lane_reset || mag(raw_r0*65536-temporal_r0)>18*65536) begin
					  value_q<=raw_r0*65536;state_q<=ST_TEMP_STORE;
					 end else begin wide_q<=coherent ? (($signed(raw_r0*64'sd65536-temporal_r0)<<3)-$signed(raw_r0*64'sd65536-temporal_r0)) : $signed(raw_r0*64'sd65536-temporal_r0);state_q<=ST_TEMP_BLEND;end
					end

				end
				ST_TEMP_BLEND: begin
blend_negative<=wide_q[31];
blend_shift<=wide_q[31] ? -wide_q[31:0] : wide_q[31:0];
blend_remainder<=0;blend_quotient<=0;blend_left<=32;
state_q<=coherent ? ST_BLEND_DIV : ST_BLEND_SLOW;

				end
				ST_TEMP_CLAMP: begin
state_q<=ST_RF_TEMP_CLAMP_0;
					if(value_q<raw_r0*65536-6*65536) value_q<=raw_r0*65536-6*65536;
					if(value_q>raw_r0*65536+6*65536) value_q<=raw_r0*65536+6*65536;
					state_q<=ST_TEMP_STORE;

				end
				ST_TEMP_STORE: begin
ram_return_q<=ST_TEMP_STORE;
					begin temporal_we0<=1; temporal_wa0<=base+i; temporal_wd0<=value_q; end active[base+i]<=1;
					o_x_table[(((32'd0+base+i)<<3)+((32'd0+base+i)<<1))+:10]<=(value_q+32768)>>>16;
					ram_return_q<=ST_TEMP_NEXT;

state_q<=ST_RAM_COMMIT0;
				end
				ST_TEMP_NEXT: begin
					if(i==20) begin
					 if(lane==3) state_q<=ST_DONE;
					 else begin lane<=lane+1'b1;base<=base+21;i<=0;state_q<=ST_LANE_LOAD;end
					end else begin i<=i+1;state_q<=ST_RF_TEMP_APPLY_0;end

				end
				ST_LANE_LOAD: begin
i<=0;state_q<=ST_RF_LOAD_VALID_0;

				end
				ST_DONE: begin
					o_done<=1;state_q<=ST_IDLE;

				end
				ST_LOAD_RAW: begin
ram_return_q<=ST_LOAD_RAW;
begin raw_we0<=1; raw_wa0<=init_q; raw_wd0<=o_x_table[(((32'd0+init_q)<<3)+((32'd0+init_q)<<1))+:10]; end if(init_q==83)ram_return_q<=ST_LIST;else init_q<=init_q+1'b1;

state_q<=ST_RAM_COMMIT0;
				end
				ST_LOAD_VALID: begin
state_q<=ST_RF_LOAD_VALID_0;
valid[i]<=i_valid_table[base+i] && raw_r0<640;
if(i==20)begin removed<=0;edge_mode<=0;state_q<=ST_LIST;end else i<=i+1;

				end
				ST_MUL_START: begin
mul_acc<=0;mul_left<=32;state_q<=ST_MUL_STEP;

				end
				ST_MUL_STEP: begin
mul_a<=mul_a<<1;mul_b<=mul_b>>1;mul_acc<=mul_sum;mul_left<=mul_left-1'b1;
if(mul_left==1 || mul_b<=1)begin mul_result<=mul_sum;state_q<=mul_return_q;end

				end
				ST_BLEND_DIV: begin
blend_shift<=blend_shift<<1;
blend_remainder<=blend_take ? blend_trial-5'd10 : blend_trial;
blend_quotient<={blend_quotient[30:0],blend_take};blend_left<=blend_left-1'b1;
if(blend_left==1)state_q<=ST_RF_BLEND_FINISH_0;

				end
				ST_BLEND_SLOW: begin
blend_quotient<=blend_shift>>2;state_q<=ST_RF_BLEND_FINISH_0;

				end
				ST_BLEND_FINISH: begin
state_q<=ST_RF_BLEND_FINISH_0;
value_q<=temporal_r0+(blend_negative ? -$signed(blend_quotient) : $signed(blend_quotient));state_q<=ST_RF_TEMP_CLAMP_0;

				end
				ST_MATH_CAND_RESID_0: begin
state_q<=ST_RF_MATH_CAND_RESID_0_0;
mul_a<=raw_r0-raw_r1;mul_b<=span;mul_return_q<=ST_MATH_CAND_RESID_0_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_RESID_0_SAVE: begin
product0<=mul_result;state_q<=ST_RF_MATH_CAND_RESID_1_0;

				end
				ST_MATH_CAND_RESID_1: begin
state_q<=ST_RF_MATH_CAND_RESID_1_0;
mul_a<=raw_r0-raw_r1;mul_b<=ys[b]-ys[a];mul_return_q<=ST_MATH_CAND_RESID_1_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_RESID_1_SAVE: begin
product1<=mul_result;state_q<=ST_RF_MATH_CAND_RESID_2_0;

				end
				ST_MATH_CAND_RESID_2: begin
state_q<=ST_RF_MATH_CAND_RESID_2_0;
mul_a<=raw_r0-raw_r1;mul_b<=span;mul_return_q<=ST_MATH_CAND_RESID_2_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_RESID_2_SAVE: begin
product2<=mul_result;state_q<=ST_RF_MATH_CAND_RESID_3_0;

				end
				ST_MATH_CAND_RESID_3: begin
state_q<=ST_RF_MATH_CAND_RESID_3_0;
mul_a<=raw_r0-raw_r1;mul_b<=ys[c]-ys[a];mul_return_q<=ST_MATH_CAND_RESID_3_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_RESID_3_SAVE: begin
product3<=mul_result;state_q<=ST_CAND_RESID;

				end
				ST_MATH_CAND_TEST_0: begin
mul_a<=dx0;mul_b<=h1;mul_return_q<=ST_MATH_CAND_TEST_0_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_TEST_0_SAVE: begin
product0<=mul_result;state_q<=ST_MATH_CAND_TEST_1;

				end
				ST_MATH_CAND_TEST_1: begin
mul_a<=dx1;mul_b<=h0;mul_return_q<=ST_MATH_CAND_TEST_1_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_TEST_1_SAVE: begin
product1<=mul_result;state_q<=ST_MATH_CAND_TEST_2;

				end
				ST_MATH_CAND_TEST_2: begin
mul_a<=(h0<<3)-h0;mul_b<=h1;mul_return_q<=ST_MATH_CAND_TEST_2_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_TEST_2_SAVE: begin
product2<=mul_result;state_q<=ST_CAND_TEST;

				end
				ST_MATH_CAND_KEEP_0: begin
mul_a<=peak;mul_b<=best_span;mul_return_q<=ST_MATH_CAND_KEEP_0_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_KEEP_0_SAVE: begin
product0<=mul_result;state_q<=ST_MATH_CAND_KEEP_1;

				end
				ST_MATH_CAND_KEEP_1: begin
mul_a<=best_peak;mul_b<=span;mul_return_q<=ST_MATH_CAND_KEEP_1_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_CAND_KEEP_1_SAVE: begin
product1<=mul_result;state_q<=ST_CAND_KEEP;

				end
				ST_MATH_TIP_TEST_0: begin
mul_a<=dx0;mul_b<=h1;mul_return_q<=ST_MATH_TIP_TEST_0_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_0_SAVE: begin
product0<=mul_result;state_q<=ST_MATH_TIP_TEST_1;

				end
				ST_MATH_TIP_TEST_1: begin
mul_a<=dx1;mul_b<=h0;mul_return_q<=ST_MATH_TIP_TEST_1_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_1_SAVE: begin
product1<=mul_result;state_q<=ST_MATH_TIP_TEST_2;

				end
				ST_MATH_TIP_TEST_2: begin
mul_a<=(h0<<1)+h0;mul_b<=h1;mul_return_q<=ST_MATH_TIP_TEST_2_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_2_SAVE: begin
product2<=mul_result;state_q<=ST_MATH_TIP_TEST_3;

				end
				ST_MATH_TIP_TEST_3: begin
mul_a<=dx1;mul_b<=h2;mul_return_q<=ST_MATH_TIP_TEST_3_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_3_SAVE: begin
product3<=mul_result;state_q<=ST_MATH_TIP_TEST_4;

				end
				ST_MATH_TIP_TEST_4: begin
mul_a<=dx2;mul_b<=h1;mul_return_q<=ST_MATH_TIP_TEST_4_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_4_SAVE: begin
product4<=mul_result;state_q<=ST_MATH_TIP_TEST_5;

				end
				ST_MATH_TIP_TEST_5: begin
mul_a<=(h1<<3)-h1;mul_b<=h2;mul_return_q<=ST_MATH_TIP_TEST_5_SAVE;state_q<=ST_MUL_START;

				end
				ST_MATH_TIP_TEST_5_SAVE: begin
product5<=mul_result;state_q<=ST_TIP_TEST;

				end
				ST_RF_CAND_LOAD_0: begin
state_q<=ST_RF_CAND_LOAD_0_WAIT;
				end
				ST_RF_CAND_LOAD_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_CAND_LOAD_1;
				end
				ST_RF_CAND_LOAD_1: begin
state_q<=ST_RF_CAND_LOAD_1_WAIT;
				end
				ST_RF_CAND_LOAD_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_RF_CAND_LOAD_2;
				end
				ST_RF_CAND_LOAD_2: begin
state_q<=ST_RF_CAND_LOAD_2_WAIT;
				end
				ST_RF_CAND_LOAD_2_WAIT: begin
raw_r2<=raw_q; state_q<=ST_RF_CAND_LOAD_3;
				end
				ST_RF_CAND_LOAD_3: begin
state_q<=ST_RF_CAND_LOAD_3_WAIT;
				end
				ST_RF_CAND_LOAD_3_WAIT: begin
raw_r3<=raw_q; state_q<=ST_CAND_LOAD;
				end
				ST_RF_TIP_LOAD_0: begin
state_q<=ST_RF_TIP_LOAD_0_WAIT;
				end
				ST_RF_TIP_LOAD_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_TIP_LOAD_1;
				end
				ST_RF_TIP_LOAD_1: begin
state_q<=ST_RF_TIP_LOAD_1_WAIT;
				end
				ST_RF_TIP_LOAD_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_RF_TIP_LOAD_2;
				end
				ST_RF_TIP_LOAD_2: begin
state_q<=ST_RF_TIP_LOAD_2_WAIT;
				end
				ST_RF_TIP_LOAD_2_WAIT: begin
raw_r2<=raw_q; state_q<=ST_RF_TIP_LOAD_3;
				end
				ST_RF_TIP_LOAD_3: begin
state_q<=ST_RF_TIP_LOAD_3_WAIT;
				end
				ST_RF_TIP_LOAD_3_WAIT: begin
raw_r3<=raw_q; state_q<=ST_TIP_LOAD;
				end
				ST_RF_GATE_SORT_LOAD_0: begin
state_q<=ST_RF_GATE_SORT_LOAD_0_WAIT;
				end
				ST_RF_GATE_SORT_LOAD_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_GATE_SORT_LOAD_1;
				end
				ST_RF_GATE_SORT_LOAD_1: begin
state_q<=ST_RF_GATE_SORT_LOAD_1_WAIT;
				end
				ST_RF_GATE_SORT_LOAD_1_WAIT: begin
last_r0<=last_q; state_q<=ST_GATE_SORT_LOAD;
				end
				ST_RF_SORT_LOOP_0: begin
state_q<=ST_RF_SORT_LOOP_0_WAIT;
				end
				ST_RF_SORT_LOOP_0_WAIT: begin
ordered_r0<=ordered_q; state_q<=ST_RF_SORT_LOOP_1;
				end
				ST_RF_SORT_LOOP_1: begin
state_q<=ST_RF_SORT_LOOP_1_WAIT;
				end
				ST_RF_SORT_LOOP_1_WAIT: begin
ordered_r1<=ordered_q; state_q<=ST_SORT_LOOP;
				end
				ST_RF_MEDIAN_0: begin
state_q<=ST_RF_MEDIAN_0_WAIT;
				end
				ST_RF_MEDIAN_0_WAIT: begin
ordered_r0<=ordered_q; state_q<=ST_RF_MEDIAN_1;
				end
				ST_RF_MEDIAN_1: begin
state_q<=ST_RF_MEDIAN_1_WAIT;
				end
				ST_RF_MEDIAN_1_WAIT: begin
ordered_r1<=ordered_q; state_q<=ST_MEDIAN;
				end
				ST_RF_GATE_APPLY_0: begin
state_q<=ST_RF_GATE_APPLY_0_WAIT;
				end
				ST_RF_GATE_APPLY_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_GATE_APPLY_1;
				end
				ST_RF_GATE_APPLY_1: begin
state_q<=ST_RF_GATE_APPLY_1_WAIT;
				end
				ST_RF_GATE_APPLY_1_WAIT: begin
last_r0<=last_q; state_q<=ST_RF_GATE_APPLY_2;
				end
				ST_RF_GATE_APPLY_2: begin
state_q<=ST_RF_GATE_APPLY_2_WAIT;
				end
				ST_RF_GATE_APPLY_2_WAIT: begin
low_r0<=low_q; state_q<=ST_RF_GATE_APPLY_3;
				end
				ST_RF_GATE_APPLY_3: begin
state_q<=ST_RF_GATE_APPLY_3_WAIT;
				end
				ST_RF_GATE_APPLY_3_WAIT: begin
high_r0<=high_q; state_q<=ST_GATE_APPLY;
				end
				ST_RF_GATE_DECIDE_0: begin
state_q<=ST_RF_GATE_DECIDE_0_WAIT;
				end
				ST_RF_GATE_DECIDE_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_GATE_DECIDE_1;
				end
				ST_RF_GATE_DECIDE_1: begin
state_q<=ST_RF_GATE_DECIDE_1_WAIT;
				end
				ST_RF_GATE_DECIDE_1_WAIT: begin
pending_r0<=pending_q; state_q<=ST_GATE_DECIDE;
				end
				ST_RF_GATE_CONFIRM_0: begin
state_q<=ST_RF_GATE_CONFIRM_0_WAIT;
				end
				ST_RF_GATE_CONFIRM_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_GATE_CONFIRM;
				end
				ST_RF_TEMP_SORT_LOAD_0: begin
state_q<=ST_RF_TEMP_SORT_LOAD_0_WAIT;
				end
				ST_RF_TEMP_SORT_LOAD_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_TEMP_SORT_LOAD_1;
				end
				ST_RF_TEMP_SORT_LOAD_1: begin
state_q<=ST_RF_TEMP_SORT_LOAD_1_WAIT;
				end
				ST_RF_TEMP_SORT_LOAD_1_WAIT: begin
temporal_r0<=temporal_q; state_q<=ST_TEMP_SORT_LOAD;
				end
				ST_RF_TEMP_AGREE_0: begin
state_q<=ST_RF_TEMP_AGREE_0_WAIT;
				end
				ST_RF_TEMP_AGREE_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_TEMP_AGREE_1;
				end
				ST_RF_TEMP_AGREE_1: begin
state_q<=ST_RF_TEMP_AGREE_1_WAIT;
				end
				ST_RF_TEMP_AGREE_1_WAIT: begin
temporal_r0<=temporal_q; state_q<=ST_TEMP_AGREE;
				end
				ST_RF_TEMP_APPLY_0: begin
state_q<=ST_RF_TEMP_APPLY_0_WAIT;
				end
				ST_RF_TEMP_APPLY_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_TEMP_APPLY_1;
				end
				ST_RF_TEMP_APPLY_1: begin
state_q<=ST_RF_TEMP_APPLY_1_WAIT;
				end
				ST_RF_TEMP_APPLY_1_WAIT: begin
temporal_r0<=temporal_q; state_q<=ST_TEMP_APPLY;
				end
				ST_RF_TEMP_CLAMP_0: begin
state_q<=ST_RF_TEMP_CLAMP_0_WAIT;
				end
				ST_RF_TEMP_CLAMP_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_TEMP_CLAMP;
				end
				ST_RF_LOAD_VALID_0: begin
state_q<=ST_RF_LOAD_VALID_0_WAIT;
				end
				ST_RF_LOAD_VALID_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_LOAD_VALID;
				end
				ST_RF_BLEND_FINISH_0: begin
state_q<=ST_RF_BLEND_FINISH_0_WAIT;
				end
				ST_RF_BLEND_FINISH_0_WAIT: begin
temporal_r0<=temporal_q; state_q<=ST_BLEND_FINISH;
				end
				ST_RF_MATH_CAND_RESID_0_0: begin
state_q<=ST_RF_MATH_CAND_RESID_0_0_WAIT;
				end
				ST_RF_MATH_CAND_RESID_0_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_MATH_CAND_RESID_0_1;
				end
				ST_RF_MATH_CAND_RESID_0_1: begin
state_q<=ST_RF_MATH_CAND_RESID_0_1_WAIT;
				end
				ST_RF_MATH_CAND_RESID_0_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_MATH_CAND_RESID_0;
				end
				ST_RF_MATH_CAND_RESID_1_0: begin
state_q<=ST_RF_MATH_CAND_RESID_1_0_WAIT;
				end
				ST_RF_MATH_CAND_RESID_1_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_MATH_CAND_RESID_1_1;
				end
				ST_RF_MATH_CAND_RESID_1_1: begin
state_q<=ST_RF_MATH_CAND_RESID_1_1_WAIT;
				end
				ST_RF_MATH_CAND_RESID_1_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_MATH_CAND_RESID_1;
				end
				ST_RF_MATH_CAND_RESID_2_0: begin
state_q<=ST_RF_MATH_CAND_RESID_2_0_WAIT;
				end
				ST_RF_MATH_CAND_RESID_2_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_MATH_CAND_RESID_2_1;
				end
				ST_RF_MATH_CAND_RESID_2_1: begin
state_q<=ST_RF_MATH_CAND_RESID_2_1_WAIT;
				end
				ST_RF_MATH_CAND_RESID_2_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_MATH_CAND_RESID_2;
				end
				ST_RF_MATH_CAND_RESID_3_0: begin
state_q<=ST_RF_MATH_CAND_RESID_3_0_WAIT;
				end
				ST_RF_MATH_CAND_RESID_3_0_WAIT: begin
raw_r0<=raw_q; state_q<=ST_RF_MATH_CAND_RESID_3_1;
				end
				ST_RF_MATH_CAND_RESID_3_1: begin
state_q<=ST_RF_MATH_CAND_RESID_3_1_WAIT;
				end
				ST_RF_MATH_CAND_RESID_3_1_WAIT: begin
raw_r1<=raw_q; state_q<=ST_MATH_CAND_RESID_3;
				end
				ST_RAM_COMMIT0: begin
state_q<=ST_RAM_COMMIT1;
				end
				ST_RAM_COMMIT1: begin
state_q<=ram_return_q;
				end
				default: state_q <= ST_IDLE;
			endcase
		end
	end
endmodule

// 独立同步读、单写端口；数据阵列无复位，历史有效位屏蔽复位前内容。
module host_lane_postprocess_track_store #(
 parameter C_WIDTH = 27, parameter C_DEPTH = 84, parameter C_ADDR = 7,
 parameter C_CLEAR = 0
)(
 input wire i_clk, input wire i_rst_n, input wire i_we,
 input wire [C_ADDR-1:0] i_waddr, input wire signed [C_WIDTH-1:0] i_data,
 input wire [C_ADDR-1:0] i_raddr,
 output reg signed [C_WIDTH-1:0] o_data /* synthesis syn_ramstyle="lut_ram" */
);
 reg signed [C_WIDTH-1:0] mem[0:C_DEPTH-1];
 reg [C_DEPTH-1:0] written;
 always @(posedge i_clk) begin
  if(i_rst_n && i_we) mem[i_waddr] <= i_data;
  o_data <= C_CLEAR && !written[i_raddr] ? {C_WIDTH{1'b0}} : mem[i_raddr];
 end
 generate if(C_CLEAR) begin: gen_history_valid
  always @(posedge i_clk or negedge i_rst_n) begin
   if(!i_rst_n) written <= 0;
   else if(i_we) written[i_waddr] <= 1'b1;
  end
 end else begin: gen_no_clear
  always @(*) written = {C_DEPTH{1'b1}};
 end endgenerate
endmodule

`default_nettype wire
