//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 对筛选后的车道锚点执行两遍 Q16 空间平滑和单调 Hermite 插值，转换为逐图像行的四车道显示信息。
// Module Name: host_lane_spatial, host_lane_spatial_lut_store
// File Name: lane/rtl/host_lane_spatial.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 对筛选后的车道锚点执行两遍 Q16 空间平滑和单调 Hermite 插值，转换为逐图像行的四车道显示信息。
// 保留 edge_table 指定的缺口，固定首末端且不外推；段偏移超过 6 像素时整段退回直线，输出 52 位逐行数据。
//
// Dependencies: host_lane_signed_divider；同文件 host_lane_spatial_lut_store
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原文说明由 tools/generate_serial_lane.py 独立生成，RAM 端口经串行化；同文件参数化工作存储用于锚点、工作坐标和切线。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Q16 两次空间平滑，单调 Hermite；段偏移超过6像素则整段退回直线。
// 缺口由 edge_table 保护，首末端固定，不外推。
// Generated in isolation by tools/generate_serial_lane.py; RAM ports are serialized.
`timescale 1ns/1ps
`default_nettype none
module host_lane_spatial (
 input wire i_clk, input wire i_rst_n, input wire i_start,
 input wire [839:0] i_x_table, input wire [83:0] i_valid_table,
 input wire [83:0] i_edge_table, input wire [7:0] i_type_table,
 output reg o_done, output wire o_wr_en, output wire [8:0] o_wr_row,
 output wire [51:0] o_wr_data
);
 reg [8:0] state_q;
 reg [839:0] raw_x;
 reg [83:0] valid,edges_q;
 reg [7:0] types;
 reg [6:0] segment_start[0:83];
 reg fallback[0:83];
 reg signed [7:0] ids[0:20];
 reg signed [9:0] ys[0:20];
 integer lane,base,i,j,n,pass,a,b,c,h0,h1,h,offset,mode,which;
 integer segment,scan,row,sel0,sel1,below,above,scan_count;
 reg signed [31:0] v0,v1,v2,d0,d1,m,ma,mb;
 reg signed [63:0] mp;
 reg [63:0] mul_a,mul_b,mul_acc;
 wire [63:0] mul_sum=mul_acc+(mul_b[0] ? mul_a : 64'd0);
 reg [6:0] mul_left;
 reg [8:0] mul_return_q;
 reg [15:0] distance_adjusted;
 reg signed [63:0] total,temp,num;
 reg signed [31:0] den;
 reg [8:0] return_q;
 wire math_done;
 wire signed [63:0] math_result;
 reg signed [31:0] interp,low_x,high_x;
 reg [31:0] linear_weight;
 integer baseline,pixel,dx;
 reg [51:0] row_word;
 reg [127:0] coeff;
 integer k;
 reg [6:0] init_q;
 function integer mag;
  input integer x;begin mag=x<0 ? -x : x;end
 endfunction
 // All permitted gaps span one or two anchors.  Positive rounded numerator
 // is <=639*22+11, so a 16-bit reciprocal plus one correction is exact.
 host_lane_signed_divider u_divider(
  .i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(state_q==9'd41),
  .i_num(num),.i_den(den),.o_done(math_done),.o_result(math_result));
 assign o_wr_en=state_q==9'd38;
 assign o_wr_row=row[8:0];
 assign o_wr_data=row_word;

 reg [159:0] coeff_mem [0:127];
 initial begin
 coeff_mem[0]=160'h0010000000000000000000000000000000000000;
 coeff_mem[1]=160'h000f8d50000072b0000cf5c3fffe8f5c0001999a;
 coeff_mem[2]=160'h000e56040001a9fc00147ae1fffae14800033333;
 coeff_mem[3]=160'h000c8b44000374bc0017851ffff5eb850004cccd;
 coeff_mem[4]=160'h000a5e350005a1cb00170a3dfff0a3d700066666;
 coeff_mem[5]=160'h000800000008000000140000ffec000000080000;
 coeff_mem[6]=160'h0005a1cb000a5e35000f5c29ffe8f5c30009999a;
 coeff_mem[7]=160'h000374bc000c8b44000a147bffe87ae1000b3333;
 coeff_mem[8]=160'h0001a9fc000e560400051eb8ffeb851f000ccccd;
 coeff_mem[9]=160'h000072b0000f8d50000170a4fff30a3d000e6666;
 coeff_mem[10]=160'h0000000000100000000000000000000000100000;
 coeff_mem[32]=160'h0010000000000000000000000000000000000000;
 coeff_mem[33]=160'h000fa09a00005f66000d3920fffead7d0001745d;
 coeff_mem[34]=160'h000e9b06000164fa00156be7fffb3d5b0002e8ba;
 coeff_mem[35]=160'h000d14320002ebce00196370fff67ab600045d17;
 coeff_mem[36]=160'h000b310c0004cef40019ead8fff130a90005d174;
 coeff_mem[37]=160'h000916810006e97f0017cd39ffec2a50000745d1;
 coeff_mem[38]=160'h0006e97f000916810013d5b0ffe832c70008ba2f;
 coeff_mem[39]=160'h0004cef4000b310c000ecf57ffe61528000a2e8c;
 coeff_mem[40]=160'h0002ebce000d14320009854affe69c90000ba2e9;
 coeff_mem[41]=160'h000164fa000e9b060004c2a5ffea9419000d1746;
 coeff_mem[42]=160'h00005f66000fa09a00015283fff2c6e0000e8ba3;
 coeff_mem[43]=160'h0000000000100000000000000000000000100000;
 coeff_mem[64]=160'h0010000000000000000000000000000000000000;
 coeff_mem[65]=160'h000fe50500001afb000e8331ffff463e0000c30c;
 coeff_mem[66]=160'h000f979f00006861001a31ecfffd3e1d00018618;
 coeff_mem[67]=160'h000f1d1c0000e2e4002343ebfffa1f5900024925;
 coeff_mem[68]=160'h000e7aca000185360029f0e8fff621ab00030c31;
 coeff_mem[69]=160'h000db5f900024a07002e709efff17ccf0003cf3d;
 coeff_mem[70]=160'h000cd3f700032c090030fac7ffec687d00049249;
 coeff_mem[71]=160'h000bda13000425ed0031c71cffe71c7200055555;
 coeff_mem[72]=160'h000acd9b0005326500310d5affe1d06600061862;
 coeff_mem[73]=160'h0009b3df00064c21002f0539ffdcbc150006db6e;
 coeff_mem[74]=160'h0008922d00076dd3002be675ffd8173800079e7a;
 coeff_mem[75]=160'h00076dd30008922d0027e8c8ffd4198b00086186;
 coeff_mem[76]=160'h00064c210009b3df002343ebffd0fac700092492;
 coeff_mem[77]=160'h00053265000acd9b001e2f9affcef2a60009e79e;
 coeff_mem[78]=160'h000425ed000bda130018e38effce38e4000aaaab;
 coeff_mem[79]=160'h00032c09000cd3f700139783ffcf0539000b6db7;
 coeff_mem[80]=160'h00024a07000db5f9000e8331ffd18f62000c30c3;
 coeff_mem[81]=160'h00018536000e7aca0009de55ffd60f18000cf3cf;
 coeff_mem[82]=160'h0000e2e4000f1d1c0005e0a7ffdcbc15000db6db;
 coeff_mem[83]=160'h00006861000f979f0002c1e3ffe5ce14000e79e8;
 coeff_mem[84]=160'h00001afb000fe5050000b9c2fff17ccf000f3cf4;
 coeff_mem[85]=160'h0000000000100000000000000000000000100000;
 coeff_mem[96]=160'h0010000000000000000000000000000000000000;
 coeff_mem[97]=160'h000fe7620000189e000e9419ffff4e480000ba2f;
 coeff_mem[98]=160'h000fa09a00005f66001a723ffffd5afa0001745d;
 coeff_mem[99]=160'h000f30470000cfb90023cd39fffa58dc00022e8c;
 coeff_mem[100]=160'h000e9b06000164fa002ad7cdfff67ab60002e8ba;
 coeff_mem[101]=160'h000de57500021a8b002fc4c3fff1f34e0003a2e9;
 coeff_mem[102]=160'h000d14320002ebce0032c6e0ffecf56c00045d17;
 coeff_mem[103]=160'h000c2bda0003d426003410edffe7b3d600051746;
 coeff_mem[104]=160'h000b310c0004cef40033d5b0ffe261530005d174;
 coeff_mem[105]=160'h000a28640005d79c003247efffdd30a900068ba3;
 coeff_mem[106]=160'h000916810006e97f002f9a72ffd854a1000745d1;
 coeff_mem[107]=160'h0008000000080000002c0000ffd4000000080000;
 coeff_mem[108]=160'h0006e97f000916810027ab5fffd0658e0008ba2f;
 coeff_mem[109]=160'h0005d79c000a28640022cf57ffcdb8110009745d;
 coeff_mem[110]=160'h0004cef4000b310c001d9eadffcc2a50000a2e8c;
 coeff_mem[111]=160'h0003d426000c2bda00184c2affcbef13000ae8ba;
 coeff_mem[112]=160'h0002ebce000d143200130a94ffcd3920000ba2e9;
 coeff_mem[113]=160'h00021a8b000de575000e0cb2ffd03b3d000c5d17;
 coeff_mem[114]=160'h000164fa000e9b060009854affd52833000d1746;
 coeff_mem[115]=160'h0000cfb9000f30470005a724ffdc32c7000dd174;
 coeff_mem[116]=160'h00005f66000fa09a0002a506ffe58dc1000e8ba3;
 coeff_mem[117]=160'h0000189e000fe7620000b1b8fff16be7000f45d1;
 coeff_mem[118]=160'h0000000000100000000000000000000000100000;
end
	localparam [8:0] ST_LOAD_INIT = 9'd43;
	localparam [8:0] ST_MUL_START = 9'd44;
	localparam [8:0] ST_MUL_STEP = 9'd45;
	localparam [8:0] ST_HARMONIC_FINISH = 9'd46;
	localparam [8:0] ST_BASE_ADJUST = 9'd47;
	localparam [8:0] ST_BASE_DIV = 9'd48;
	localparam [8:0] ST_BASE_FINISH = 9'd49;
	localparam [8:0] ST_EVAL_MAIN = 9'd50;
	localparam [8:0] ST_IDLE = 9'd0;
	localparam [8:0] ST_LIST = 9'd1;
	localparam [8:0] ST_LIST_SCAN = 9'd2;
	localparam [8:0] ST_SEGMENTS = 9'd3;
	localparam [8:0] ST_SMOOTH = 9'd4;
	localparam [8:0] ST_SMOOTH_LOAD = 9'd5;
	localparam [8:0] ST_SMOOTH_MUL0 = 9'd6;
	localparam [8:0] ST_SMOOTH_MUL1 = 9'd7;
	localparam [8:0] ST_SMOOTH_DIV = 9'd8;
	localparam [8:0] ST_SMOOTH_RESULT = 9'd9;
	localparam [8:0] ST_SMOOTH_CLAMP = 9'd10;
	localparam [8:0] ST_COPY = 9'd11;
	localparam [8:0] ST_TANGENT = 9'd12;
	localparam [8:0] ST_TANGENT_LOAD = 9'd13;
	localparam [8:0] ST_SLOPE0 = 9'd14;
	localparam [8:0] ST_SLOPE1 = 9'd15;
	localparam [8:0] ST_TANGENT_CALC = 9'd16;
	localparam [8:0] ST_END_MUL1 = 9'd17;
	localparam [8:0] ST_END_DIV = 9'd18;
	localparam [8:0] ST_END_CLAMP = 9'd19;
	localparam [8:0] ST_HARMONIC_NUM = 9'd20;
	localparam [8:0] ST_HARMONIC_DEN1 = 9'd21;
	localparam [8:0] ST_HARMONIC_DIV = 9'd22;
	localparam [8:0] ST_HARMONIC_STORE = 9'd23;
	localparam [8:0] ST_ROW_INIT = 9'd24;
	localparam [8:0] ST_SCAN = 9'd25;
	localparam [8:0] ST_CHECK = 9'd26;
	localparam [8:0] ST_COEFFICIENT = 9'd27;
	localparam [8:0] ST_EVAL_INIT = 9'd28;
	localparam [8:0] ST_EVAL1 = 9'd29;
	localparam [8:0] ST_EVAL2 = 9'd30;
	localparam [8:0] ST_EVAL3 = 9'd31;
	localparam [8:0] ST_EVAL_FINISH = 9'd32;
	localparam [8:0] ST_EVAL_CLAMP = 9'd33;
	localparam [8:0] ST_EVAL_GUARD = 9'd34;
	localparam [8:0] ST_STORE_PIXEL = 9'd35;
	localparam [8:0] ST_NEXT_LANE = 9'd36;
	localparam [8:0] ST_ROW_END = 9'd37;
	localparam [8:0] ST_WRITE = 9'd38;
	localparam [8:0] ST_ROW_NEXT = 9'd39;
	localparam [8:0] ST_DONE = 9'd40;
	localparam [8:0] ST_DIV_START = 9'd41;
	localparam [8:0] ST_DIV_WAIT = 9'd42;
	localparam [8:0] ST_RF_SMOOTH_0 = 9'd51;
	localparam [8:0] ST_RF_SMOOTH_0_WAIT = 9'd52;
	localparam [8:0] ST_RF_SMOOTH_MUL0_0 = 9'd53;
	localparam [8:0] ST_RF_SMOOTH_MUL0_0_WAIT = 9'd54;
	localparam [8:0] ST_RF_SMOOTH_MUL1_0 = 9'd55;
	localparam [8:0] ST_RF_SMOOTH_MUL1_0_WAIT = 9'd56;
	localparam [8:0] ST_RF_SMOOTH_RESULT_0 = 9'd57;
	localparam [8:0] ST_RF_SMOOTH_RESULT_0_WAIT = 9'd58;
	localparam [8:0] ST_RF_COPY_0 = 9'd59;
	localparam [8:0] ST_RF_COPY_0_WAIT = 9'd60;
	localparam [8:0] ST_RF_TANGENT_LOAD_0 = 9'd61;
	localparam [8:0] ST_RF_TANGENT_LOAD_0_WAIT = 9'd62;
	localparam [8:0] ST_RF_TANGENT_LOAD_1 = 9'd63;
	localparam [8:0] ST_RF_TANGENT_LOAD_1_WAIT = 9'd64;
	localparam [8:0] ST_RF_TANGENT_LOAD_2 = 9'd65;
	localparam [8:0] ST_RF_TANGENT_LOAD_2_WAIT = 9'd66;
	localparam [8:0] ST_RF_CHECK_0 = 9'd67;
	localparam [8:0] ST_RF_CHECK_0_WAIT = 9'd68;
	localparam [8:0] ST_RF_CHECK_1 = 9'd69;
	localparam [8:0] ST_RF_CHECK_1_WAIT = 9'd70;
	localparam [8:0] ST_RF_EVAL1_0 = 9'd71;
	localparam [8:0] ST_RF_EVAL1_0_WAIT = 9'd72;
	localparam [8:0] ST_RF_EVAL2_0 = 9'd73;
	localparam [8:0] ST_RF_EVAL2_0_WAIT = 9'd74;
	localparam [8:0] ST_RF_EVAL3_0 = 9'd75;
	localparam [8:0] ST_RF_EVAL3_0_WAIT = 9'd76;
	localparam [8:0] ST_RF_EVAL_MAIN_0 = 9'd77;
	localparam [8:0] ST_RF_EVAL_MAIN_0_WAIT = 9'd78;
	localparam [8:0] ST_RAM_COMMIT0 = 9'd79;
reg [8:0] ram_return_q;
reg [6:0] anchor_ra;
wire signed [26:0] anchor_q;
reg anchor_we;
reg [6:0] anchor_wa;
reg signed [26:0] anchor_wd;
reg signed [26:0] anchor_r0;
reg signed [26:0] anchor_r1;
reg signed [26:0] anchor_r2;
reg anchor_we0;
reg [6:0] anchor_wa0;
reg signed [26:0] anchor_wd0;
host_lane_spatial_lut_store #(.C_WIDTH(27),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(0)) u_ram_anchor(i_clk,i_rst_n,anchor_we,anchor_wa,anchor_wd,anchor_ra,anchor_q);
reg [6:0] work_ra;
wire signed [26:0] work_q;
reg work_we;
reg [6:0] work_wa;
reg signed [26:0] work_wd;
reg signed [26:0] work_r0;
reg work_we0;
reg [6:0] work_wa0;
reg signed [26:0] work_wd0;
host_lane_spatial_lut_store #(.C_WIDTH(27),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(0)) u_ram_work(i_clk,i_rst_n,work_we,work_wa,work_wd,work_ra,work_q);
reg [6:0] tangent_ra;
wire signed [26:0] tangent_q;
reg tangent_we;
reg [6:0] tangent_wa;
reg signed [26:0] tangent_wd;
reg signed [26:0] tangent_r0;
reg tangent_we0;
reg [6:0] tangent_wa0;
reg signed [26:0] tangent_wd0;
host_lane_spatial_lut_store #(.C_WIDTH(27),.C_DEPTH(84),.C_ADDR(7),.C_CLEAR(0)) u_ram_tangent(i_clk,i_rst_n,tangent_we,tangent_wa,tangent_wd,tangent_ra,tangent_q);
always @(*) begin
anchor_ra=0; anchor_we=0; anchor_wa=0; anchor_wd=0;
work_ra=0; work_we=0; work_wa=0; work_wd=0;
tangent_ra=0; tangent_we=0; tangent_wa=0; tangent_wd=0;
case(state_q)
ST_RF_SMOOTH_0: anchor_ra = ids[i];
ST_RF_SMOOTH_MUL0_0: anchor_ra = b;
ST_RF_SMOOTH_MUL1_0: anchor_ra = c;
ST_RF_SMOOTH_RESULT_0: anchor_ra = a;
ST_RF_COPY_0: work_ra = ids[i];
ST_RF_TANGENT_LOAD_0: anchor_ra = b;
ST_RF_TANGENT_LOAD_1: anchor_ra = a;
ST_RF_TANGENT_LOAD_2: anchor_ra = c;
ST_RF_CHECK_0: anchor_ra = sel0;
ST_RF_CHECK_1: anchor_ra = sel1;
ST_RF_EVAL1_0: anchor_ra = sel1;
ST_RF_EVAL2_0: tangent_ra = sel0;
ST_RF_EVAL3_0: tangent_ra = sel1;
ST_RF_EVAL_MAIN_0: anchor_ra = sel0;
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin anchor_we=anchor_we0; anchor_wa=anchor_wa0; anchor_wd=anchor_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin work_we=work_we0; work_wa=work_wa0; work_wd=work_wd0; end
default: begin end
endcase
case(state_q)
ST_RAM_COMMIT0: begin tangent_we=tangent_we0; tangent_wa=tangent_wa0; tangent_wd=tangent_wd0; end
default: begin end
endcase
end
	always @(posedge i_clk or negedge i_rst_n) begin
			if (!i_rst_n) begin
anchor_we0<=0; work_we0<=0; tangent_we0<=0;
            state_q<=ST_IDLE;o_done<=0;raw_x<=0;valid<=0;edges_q<=0;types<=0;
            lane<=0;base<=0;i<=0;j<=0;n<=0;row<=0;row_word<=0;
            ma<=0;mb<=0;num<=0;den<=1;return_q<=0;
            ys[0]<=82;
            ys[1]<=93;
            ys[2]<=104;
            ys[3]<=115;
            ys[4]<=126;
            ys[5]<=137;
            ys[6]<=148;
            ys[7]<=159;
            ys[8]<=170;
            ys[9]<=181;
            ys[10]<=192;
            ys[11]<=202;
            ys[12]<=213;
            ys[13]<=224;
            ys[14]<=235;
            ys[15]<=246;
            ys[16]<=257;
            ys[17]<=268;
            ys[18]<=279;
            ys[19]<=290;
            ys[20]<=301;
		end else begin
			o_done <= 0;
if(state_q!=ST_RAM_COMMIT0) begin anchor_we0<=0; work_we0<=0; tangent_we0<=0; end
			case(state_q)
				ST_IDLE: begin
					if(i_start) begin
					 raw_x<=i_x_table;valid<=i_valid_table;edges_q<=i_edge_table;types<=i_type_table;
					 init_q<=0;
					 lane<=0;base<=0;state_q<=ST_LOAD_INIT;
					end

				end
				ST_LIST: begin
					i<=0;n<=0;segment<=base;state_q<=ST_LIST_SCAN;

				end
				ST_LIST_SCAN: begin
					if(valid[base+i]) begin ids[n]<=base+i;n<=n+1;end
					if(i==20) begin i<=0;state_q<=ST_SEGMENTS;end else i<=i+1;

				end
				ST_SEGMENTS: begin
					if(i<n) begin
					 if(i==0 || !edges_q[ids[i-1]]) begin segment_start[ids[i]]<=ids[i];segment<=ids[i];end
					 else segment_start[ids[i]]<=segment;
					 i<=i+1;
					end else begin pass<=0;i<=0;state_q<=ST_RF_SMOOTH_0;end

				end
				ST_SMOOTH: begin
ram_return_q<=ST_RF_SMOOTH_0;
					if(i>=n) begin i<=0;ram_return_q<=ST_RF_COPY_0;end
					else begin
					 a<=ids[i];
					 if(i==0 || i+1==n || !edges_q[ids[i-1]] || !edges_q[ids[i]]) begin begin work_we0<=1; work_wa0<=ids[i]; work_wd0<=anchor_r0; end i<=i+1;end
					 else begin b<=ids[i-1];c<=ids[i+1];ram_return_q<=ST_SMOOTH_LOAD;end
					end

state_q<=ST_RAM_COMMIT0;
				end
				ST_SMOOTH_LOAD: begin
					h0<=ys[a-base]-ys[b-base];h1<=ys[c-base]-ys[a-base];state_q<=ST_RF_SMOOTH_MUL0_0;

				end
				ST_SMOOTH_MUL0: begin
state_q<=ST_RF_SMOOTH_MUL0_0;
					ma<=anchor_r0;mb<=h1;mul_return_q<=ST_RF_SMOOTH_MUL1_0;state_q<=ST_MUL_START;

				end
				ST_SMOOTH_MUL1: begin
state_q<=ST_RF_SMOOTH_MUL1_0;
					total<=mp;ma<=anchor_r0;mb<=h0;mul_return_q<=ST_SMOOTH_DIV;state_q<=ST_MUL_START;

				end
				ST_SMOOTH_DIV: begin
					num<=total+mp;den<=h0+h1;return_q<=ST_RF_SMOOTH_RESULT_0;state_q<=ST_DIV_START;

				end
				ST_SMOOTH_RESULT: begin
state_q<=ST_RF_SMOOTH_RESULT_0;
					interp<=($signed(anchor_r0)+$signed(math_result))>>>1;
					low_x<=($signed({1'b0,raw_x[(((32'd0+a)<<3)+((32'd0+a)<<1))+:10]})-4)*65536;
					high_x<=($signed({1'b0,raw_x[(((32'd0+a)<<3)+((32'd0+a)<<1))+:10]})+4)*65536;state_q<=ST_SMOOTH_CLAMP;

				end
				ST_SMOOTH_CLAMP: begin
ram_return_q<=ST_SMOOTH_CLAMP;
					begin work_we0<=1; work_wa0<=a; work_wd0<=interp<low_x ? low_x : (interp>high_x ? high_x : interp); end 
					i<=i+1;ram_return_q<=ST_RF_SMOOTH_0;

state_q<=ST_RAM_COMMIT0;
				end
				ST_COPY: begin
ram_return_q<=ST_RF_COPY_0;
					if(i<n) begin begin anchor_we0<=1; anchor_wa0<=ids[i]; anchor_wd0<=work_r0; end i<=i+1;end
					else if(pass==0) begin pass<=1;i<=0;ram_return_q<=ST_RF_SMOOTH_0;end
					else begin i<=0;ram_return_q<=ST_TANGENT;end

state_q<=ST_RAM_COMMIT0;
				end
				ST_TANGENT: begin
					if(i>=n) begin
					 if(lane==3) begin mode<=0;row<=0;lane<=0;base<=0;state_q<=ST_ROW_INIT;end
					 else begin lane<=lane+1;base<=base+21;state_q<=ST_LIST;end
					end else begin
					 a<=ids[i];
					 if(i==0 || !edges_q[ids[i-1]]) begin which<=1;b<=ids[i+1];c<=ids[i+2];end
					 else if(i+1==n || !edges_q[ids[i]]) begin which<=2;b<=ids[i-1];c<=ids[i-2];end
					 else begin which<=0;b<=ids[i-1];c<=ids[i+1];end
					 state_q<=ST_RF_TANGENT_LOAD_0;
					end

				end
				ST_TANGENT_LOAD: begin
state_q<=ST_RF_TANGENT_LOAD_0;
					if(which==1) begin
					 v0<=anchor_r0-anchor_r1;v1<=anchor_r2-anchor_r0;h0<=ys[b-base]-ys[a-base];h1<=ys[c-base]-ys[b-base];
					end else if(which==2) begin
					 v0<=anchor_r1-anchor_r0;v1<=anchor_r0-anchor_r2;h0<=ys[a-base]-ys[b-base];h1<=ys[b-base]-ys[c-base];
					end else begin
					 v0<=anchor_r1-anchor_r0;v1<=anchor_r2-anchor_r1;h0<=ys[a-base]-ys[b-base];h1<=ys[c-base]-ys[a-base];
					end
					state_q<=ST_SLOPE0;

				end
				ST_SLOPE0: begin
					num<=v0;den<=h0;return_q<=ST_SLOPE1;state_q<=ST_DIV_START;

				end
				ST_SLOPE1: begin
					d0<=math_result;num<=v1;den<=h1;return_q<=ST_TANGENT_CALC;state_q<=ST_DIV_START;

				end
				ST_TANGENT_CALC: begin
ram_return_q<=ST_TANGENT_CALC;
					d1<=math_result;
					if(which!=0) begin ma<=d0;mb<=2*h0+h1;mul_return_q<=ST_END_MUL1;ram_return_q<=ST_MUL_START;end
					else if(d0==0 || math_result==0 || (d0<0)!=(math_result<0)) begin begin tangent_we0<=1; tangent_wa0<=a; tangent_wd0<=0; end i<=i+1;ram_return_q<=ST_TANGENT;end
					else begin ma<=d0;mb<=math_result;mul_return_q<=ST_HARMONIC_NUM;ram_return_q<=ST_MUL_START;end

state_q<=ST_RAM_COMMIT0;
				end
				ST_END_MUL1: begin
					total<=mp;ma<=d1;mb<=h0;mul_return_q<=ST_END_DIV;state_q<=ST_MUL_START;

				end
				ST_END_DIV: begin
					num<=total-mp;den<=h0+h1;return_q<=ST_END_CLAMP;state_q<=ST_DIV_START;

				end
				ST_END_CLAMP: begin
ram_return_q<=ST_END_CLAMP;
					if(math_result==0 || (math_result<0)!=(d0<0)) begin tangent_we0<=1; tangent_wa0<=a; tangent_wd0<=0; end 
					else if((d0<0)!=(d1<0) && (math_result<0 ? -math_result : math_result)>((mag(d0)<<1)+mag(d0))) begin tangent_we0<=1; tangent_wa0<=a; tangent_wd0<=((d0<<1)+d0); end 
					else begin tangent_we0<=1; tangent_wa0<=a; tangent_wd0<=math_result; end 
					i<=i+1;ram_return_q<=ST_TANGENT;

state_q<=ST_RAM_COMMIT0;
				end
				ST_HARMONIC_NUM: begin
					// Product is Q32; multiply by (((h0+h1)<<1)+(h0+h1)) before dividing.
					temp<=mp;ma<=d1;mb<=2*h1+h0;mul_return_q<=ST_HARMONIC_DEN1;state_q<=ST_MUL_START;

				end
				ST_HARMONIC_DEN1: begin
					total<=mp;ma<=d0;mb<=h1+2*h0;mul_return_q<=ST_HARMONIC_DIV;state_q<=ST_MUL_START;

				end
				ST_HARMONIC_DIV: begin
					den<=total+mp;mul_a<=temp;mul_b<=(((h0+h1)<<1)+(h0+h1));mul_acc<=0;mul_left<=64;mul_return_q<=ST_HARMONIC_FINISH;state_q<=ST_MUL_STEP;

				end
				ST_HARMONIC_STORE: begin
ram_return_q<=ST_HARMONIC_STORE;
					begin tangent_we0<=1; tangent_wa0<=a; tangent_wd0<=math_result; end i<=i+1;ram_return_q<=ST_TANGENT;

state_q<=ST_RAM_COMMIT0;
				end
				ST_ROW_INIT: begin
					row_word<={types,44'd0};scan<=0;below<=0;above<=0;scan_count<=0;state_q<=ST_SCAN;

				end
				ST_SCAN: begin
					if(valid[base+scan]) begin
					 scan_count<=scan_count+1;
					 if(ys[scan]<=row) begin sel0<=base+scan;below<=1;end
					 if(ys[scan]>=row && !above) begin sel1<=base+scan;above<=1;end
					end
					if(scan==20) state_q<=ST_RF_CHECK_0;else scan<=scan+1;

				end
				ST_CHECK: begin
state_q<=ST_RF_CHECK_0;
					if(!below || !above || scan_count<3 || (sel0!=sel1 && !edges_q[sel0])) begin
					 row_word[lane*11+:11]<=0;state_q<=ST_NEXT_LANE;
					end else if(sel0==sel1) begin pixel<=(anchor_r0+32768)>>>16;state_q<=ST_STORE_PIXEL;end
					else begin
					 h<=ys[sel1-base]-ys[sel0-base];offset<=row-ys[sel0-base];
					 low_x<=anchor_r0<anchor_r1 ? anchor_r0 : anchor_r1;
					 high_x<=anchor_r0>anchor_r1 ? anchor_r0 : anchor_r1;state_q<=ST_COEFFICIENT;
					end

				end
				ST_COEFFICIENT: begin
					{coeff,linear_weight}<=coeff_mem[(h==10 ? 0 : h==11 ? 32 : h==21 ? 64 : 96)+offset];
					dx<=$signed({1'b0,raw_x[(((32'd0+sel1)<<3)+((32'd0+sel1)<<1))+:10]})-$signed({1'b0,raw_x[(((32'd0+sel0)<<3)+((32'd0+sel0)<<1))+:10]});state_q<=ST_EVAL_INIT;

				end
				ST_EVAL_INIT: begin
					ma<=mag(dx);mb<=offset;mul_return_q<=ST_BASE_ADJUST;state_q<=ST_MUL_START;

				end
				ST_EVAL1: begin
state_q<=ST_RF_EVAL1_0;
					total<=mp;ma<=anchor_r0;mb<=mode==1 && fallback[segment_start[sel0]] ? linear_weight : $signed(coeff[95:64]);mul_return_q<=ST_RF_EVAL2_0;state_q<=ST_MUL_START;

				end
				ST_EVAL2: begin
state_q<=ST_RF_EVAL2_0;
					total<=total+mp;ma<=tangent_r0;mb<=mode==1 && fallback[segment_start[sel0]] ? 0 : $signed(coeff[63:32]);mul_return_q<=ST_RF_EVAL3_0;state_q<=ST_MUL_START;

				end
				ST_EVAL3: begin
state_q<=ST_RF_EVAL3_0;
					total<=total+mp;ma<=tangent_r0;mb<=mode==1 && fallback[segment_start[sel0]] ? 0 : $signed(coeff[31:0]);mul_return_q<=ST_EVAL_FINISH;state_q<=ST_MUL_START;

				end
				ST_EVAL_FINISH: begin
					interp<=($signed(total+mp)+64'sd524288)>>>20;state_q<=ST_EVAL_CLAMP;

				end
				ST_EVAL_CLAMP: begin
					if(interp<low_x) pixel<=(low_x+32768)>>>16;
					else if(interp>high_x) pixel<=(high_x+32768)>>>16;
					else pixel<=(interp+32768)>>>16;
					state_q<=ST_EVAL_GUARD;

				end
				ST_EVAL_GUARD: begin
					if(mode==0 && mag(pixel-baseline)>6) fallback[segment_start[sel0]]<=1;
					state_q<=ST_STORE_PIXEL;

				end
				ST_STORE_PIXEL: begin
					row_word[lane*11+:11]<={1'b1,pixel[9:0]};state_q<=ST_NEXT_LANE;

				end
				ST_NEXT_LANE: begin
					if(lane==3) state_q<=ST_ROW_END;
					else begin lane<=lane+1;base<=base+21;scan<=0;below<=0;above<=0;scan_count<=0;state_q<=ST_SCAN;end

				end
				ST_ROW_END: begin
					if(mode==1) state_q<=ST_WRITE;else state_q<=ST_ROW_NEXT;

				end
				ST_WRITE: begin
					state_q<=ST_ROW_NEXT;

				end
				ST_ROW_NEXT: begin
					lane<=0;base<=0;
					if(row==319) begin
					 if(mode==0) begin mode<=1;row<=0;state_q<=ST_ROW_INIT;end
					 else state_q<=ST_DONE;
					end else begin row<=row+1;state_q<=ST_ROW_INIT;end

				end
				ST_DONE: begin
					o_done<=1;state_q<=ST_IDLE;

				end
				ST_DIV_START: begin
					state_q<=ST_DIV_WAIT;

				end
				ST_DIV_WAIT: begin
					if(math_done) state_q<=return_q;

				end
				ST_LOAD_INIT: begin
ram_return_q<=ST_LOAD_INIT;
begin anchor_we0<=1; anchor_wa0<=init_q; anchor_wd0<={1'b0,raw_x[(((32'd0+init_q)<<3)+((32'd0+init_q)<<1))+:10],16'd0}; end begin tangent_we0<=1; tangent_wa0<=init_q; tangent_wd0<=0; end fallback[init_q]<=0;
if(init_q==83) ram_return_q<=ST_LIST;else init_q<=init_q+1'b1;

state_q<=ST_RAM_COMMIT0;
				end
				ST_MUL_START: begin
mul_a<=$signed(ma);mul_b<={32'd0,mb};mul_acc<=mb[31] ? -($signed({{32{ma[31]}},ma})<<32) : 64'd0;mul_left<=32;state_q<=ST_MUL_STEP;

				end
				ST_MUL_STEP: begin
mul_a<=mul_a<<1;mul_b<=mul_b>>1;mul_acc<=mul_sum;mul_left<=mul_left-1'b1;
if(mul_left==1 || mul_b<=1) begin mp<=mul_sum;state_q<=mul_return_q;end

				end
				ST_HARMONIC_FINISH: begin
num<=mp;return_q<=ST_HARMONIC_STORE;state_q<=ST_DIV_START;

				end
				ST_BASE_ADJUST: begin
distance_adjusted<=mp[15:0]+(h>>1);state_q<=ST_BASE_DIV;

				end
				ST_BASE_DIV: begin
if(h==10 || h==11 || h==21 || h==22) begin
num<={48'd0,distance_adjusted};den<=h;return_q<=ST_BASE_FINISH;state_q<=ST_DIV_START;
end else begin baseline<=$signed({1'b0,raw_x[(((32'd0+sel0)<<3)+((32'd0+sel0)<<1))+:10]});state_q<=ST_RF_EVAL_MAIN_0;end

				end
				ST_BASE_FINISH: begin
baseline<=$signed({1'b0,raw_x[(((32'd0+sel0)<<3)+((32'd0+sel0)<<1))+:10]})+(dx<0 ? -math_result : math_result);state_q<=ST_RF_EVAL_MAIN_0;

				end
				ST_EVAL_MAIN: begin
state_q<=ST_RF_EVAL_MAIN_0;
ma<=anchor_r0;mb<=mode==1 && fallback[segment_start[sel0]] ? 1048576-linear_weight : $signed(coeff[127:96]);mul_return_q<=ST_RF_EVAL1_0;state_q<=ST_MUL_START;

				end
				ST_RF_SMOOTH_0: begin
state_q<=ST_RF_SMOOTH_0_WAIT;
				end
				ST_RF_SMOOTH_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_SMOOTH;
				end
				ST_RF_SMOOTH_MUL0_0: begin
state_q<=ST_RF_SMOOTH_MUL0_0_WAIT;
				end
				ST_RF_SMOOTH_MUL0_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_SMOOTH_MUL0;
				end
				ST_RF_SMOOTH_MUL1_0: begin
state_q<=ST_RF_SMOOTH_MUL1_0_WAIT;
				end
				ST_RF_SMOOTH_MUL1_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_SMOOTH_MUL1;
				end
				ST_RF_SMOOTH_RESULT_0: begin
state_q<=ST_RF_SMOOTH_RESULT_0_WAIT;
				end
				ST_RF_SMOOTH_RESULT_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_SMOOTH_RESULT;
				end
				ST_RF_COPY_0: begin
state_q<=ST_RF_COPY_0_WAIT;
				end
				ST_RF_COPY_0_WAIT: begin
work_r0<=work_q; state_q<=ST_COPY;
				end
				ST_RF_TANGENT_LOAD_0: begin
state_q<=ST_RF_TANGENT_LOAD_0_WAIT;
				end
				ST_RF_TANGENT_LOAD_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_RF_TANGENT_LOAD_1;
				end
				ST_RF_TANGENT_LOAD_1: begin
state_q<=ST_RF_TANGENT_LOAD_1_WAIT;
				end
				ST_RF_TANGENT_LOAD_1_WAIT: begin
anchor_r1<=anchor_q; state_q<=ST_RF_TANGENT_LOAD_2;
				end
				ST_RF_TANGENT_LOAD_2: begin
state_q<=ST_RF_TANGENT_LOAD_2_WAIT;
				end
				ST_RF_TANGENT_LOAD_2_WAIT: begin
anchor_r2<=anchor_q; state_q<=ST_TANGENT_LOAD;
				end
				ST_RF_CHECK_0: begin
state_q<=ST_RF_CHECK_0_WAIT;
				end
				ST_RF_CHECK_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_RF_CHECK_1;
				end
				ST_RF_CHECK_1: begin
state_q<=ST_RF_CHECK_1_WAIT;
				end
				ST_RF_CHECK_1_WAIT: begin
anchor_r1<=anchor_q; state_q<=ST_CHECK;
				end
				ST_RF_EVAL1_0: begin
state_q<=ST_RF_EVAL1_0_WAIT;
				end
				ST_RF_EVAL1_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_EVAL1;
				end
				ST_RF_EVAL2_0: begin
state_q<=ST_RF_EVAL2_0_WAIT;
				end
				ST_RF_EVAL2_0_WAIT: begin
tangent_r0<=tangent_q; state_q<=ST_EVAL2;
				end
				ST_RF_EVAL3_0: begin
state_q<=ST_RF_EVAL3_0_WAIT;
				end
				ST_RF_EVAL3_0_WAIT: begin
tangent_r0<=tangent_q; state_q<=ST_EVAL3;
				end
				ST_RF_EVAL_MAIN_0: begin
state_q<=ST_RF_EVAL_MAIN_0_WAIT;
				end
				ST_RF_EVAL_MAIN_0_WAIT: begin
anchor_r0<=anchor_q; state_q<=ST_EVAL_MAIN;
				end
				ST_RAM_COMMIT0: begin
state_q<=ram_return_q;
				end
				default: state_q <= ST_IDLE;
			endcase
		end
	end
endmodule

// 独立同步读、单写端口；数据阵列无复位，历史有效位屏蔽复位前内容。
module host_lane_spatial_lut_store #(
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
