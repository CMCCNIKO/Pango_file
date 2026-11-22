//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将原始 1920×1080 RGB888 的 y=289..979 裁剪区域按面积权重积分到 224×168；先形成 RGB888 再量化为 RGB565，供 F14 输入使用。
// Module Name: host_f14_area_resize, host_f14_area_round
// File Name: host/rtl/video/host_f14_area_resize.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将原始 1920×1080 RGB888 的 y=289..979 裁剪区域按面积权重积分到 224×168；先形成 RGB888 再量化为 RGB565，供 F14 输入使用。
// 横向以 1/7 像素、纵向以 1/168 行计量面积，通过 224 项 RGB 累加行缓存分摊事务，不存储整帧，也不对输入反压；检测异常输入并输出 abort。
//
// Dependencies: f14_ram_128x512；像素域 i_clk/i_rst_n；VS、DE、enable 与源帧号；行和数据存储不强制复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target: PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 原裁剪 y=289:980，1920x691 面积积分到 224x168；先 RGB888 后 RGB565。
// 横向单位为 1/7 像素，纵向单位为 1/168 行；总面积 60*691。
// 行累加只保存 224 个 RGB24 位和，使用现有官方 128x512 BRAM IP。
// 每 8/9 个输入像素产生一次 RAM 事务，不需要整帧缓存或反压。
`timescale 1ns/1ps
`default_nettype none
module host_f14_area_resize (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_vs,
	input wire i_de,
	input wire [23:0] i_rgb,
	input wire i_enable,
	input wire [15:0] i_frame_id,
	output wire o_valid,
	output wire [23:0] o_rgb,
	output wire [15:0] o_rgb565,
	output wire [8:0] o_row,
	output wire [9:0] o_col,
	output wire [15:0] o_frame_id,
	output reg o_abort
);
	reg vs_q,de_q,bad_q;
	reg [11:0] x_q,y_q;
	reg [5:0] xr_q;
	reg [9:0] yr_q;
	reg [7:0] col_q,row_q;
	reg [13:0] hr_q,hg_q,hb_q;
	wire crop = y_q>=289 && y_q<980;
	wire [3:0] xw = xr_q<7 ? xr_q[3:0] : 4'd7;
	wire [3:0] spill = 4'd7-xw;
	wire [8:0] yw = yr_q<168 ? {1'b0,yr_q[7:0]} : 9'd168;
	wire [8:0] yspill = 9'd168-yw;
	wire hfinish = i_de && i_enable && !bad_q && crop && x_q<1920 && xr_q<=7;
	reg v1,v2,v3,emit1,emit2,emit3,first1,first2;
	reg [7:0] col1,col2,col3,row1,row2,row3;
	reg [15:0] id1,id2,id3;
	reg [13:0] r1,g1,b1;
	reg [8:0] w1,spill1;
	reg [23:0] r2,g2,b2,rs2,gs2,bs2,r3,g3,b3;
	reg [71:0] write3;
	wire [127:0] ram_data;
	wire [23:0] sr = r2+(first2 ? 24'd0 : ram_data[23:0]);
	wire [23:0] sg = g2+(first2 ? 24'd0 : ram_data[47:24]);
	wire [23:0] sb = b2+(first2 ? 24'd0 : ram_data[71:48]);
	f14_ram_128x512 u_line_sum (
		.wr_data({56'd0,write3}),.wr_addr({1'b0,col3}),.wr_en(v3),
		.wr_clk(i_clk),.wr_rst(1'b0),.rd_data(ram_data),
		.rd_addr({1'b0,col1}),.rd_clk(i_clk),.rd_clk_en(v1),.rd_rst(1'b0)
	);
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			vs_q<=0;de_q<=0;bad_q<=0;x_q<=0;y_q<=0;xr_q<=60;yr_q<=691;
			col_q<=0;row_q<=0;hr_q<=0;hg_q<=0;hb_q<=0;o_abort<=0;
			v1<=0;v2<=0;v3<=0;emit1<=0;emit2<=0;emit3<=0;first1<=0;first2<=0;
			col1<=0;col2<=0;col3<=0;row1<=0;row2<=0;row3<=0;id1<=0;id2<=0;id3<=0;
			r1<=0;g1<=0;b1<=0;w1<=0;spill1<=0;
			r2<=0;g2<=0;b2<=0;rs2<=0;gs2<=0;bs2<=0;r3<=0;g3<=0;b3<=0;write3<=0;
		end else begin
			vs_q<=i_vs;de_q<=i_de;o_abort<=0;
			v1<=hfinish;v2<=v1;v3<=v2;
			if (hfinish) begin
				r1<=hr_q+i_rgb[23:16]*xw;g1<=hg_q+i_rgb[15:8]*xw;b1<=hb_q+i_rgb[7:0]*xw;
				col1<=col_q;row1<=row_q;id1<=i_frame_id;
				w1<=yw;spill1<=yspill;emit1<=yr_q<=168;first1<=y_q==289;
			end
			if (v1) begin
				r2<=r1*w1;g2<=g1*w1;b2<=b1*w1;
				rs2<=r1*spill1;gs2<=g1*spill1;bs2<=b1*spill1;
				col2<=col1;row2<=row1;id2<=id1;emit2<=emit1;first2<=first1;
			end
			if (v2) begin
				r3<=sr;g3<=sg;b3<=sb;write3<=emit2 ? {bs2,gs2,rs2} : {sb,sg,sr};
				col3<=col2;row3<=row2;id3<=id2;emit3<=emit2;
			end
			if (i_vs && !vs_q) begin
				x_q<=0;y_q<=0;xr_q<=60;yr_q<=691;col_q<=0;row_q<=0;
				hr_q<=0;hg_q<=0;hb_q<=0;bad_q<=0;v1<=0;v2<=0;v3<=0;
				if (de_q || (y_q!=0 && y_q<980)) o_abort<=1;
			end else begin
				if (i_de) begin
					if (x_q<4095) x_q<=x_q+1'b1;
                    // 立即拒绝超宽行，不能等DE下降才发现最后一个CNN像素已发布。
                    if (x_q==1920) begin bad_q<=1;o_abort<=1;end
					if (crop && x_q<1920) begin
						if (xr_q<=7) begin
							hr_q<=i_rgb[23:16]*spill;hg_q<=i_rgb[15:8]*spill;hb_q<=i_rgb[7:0]*spill;
							xr_q<=xr_q+6'd53;col_q<=col_q+1'b1;
						end else begin
							hr_q<=hr_q+i_rgb[23:16]*4'd7;hg_q<=hg_q+i_rgb[15:8]*4'd7;hb_q<=hb_q+i_rgb[7:0]*4'd7;
							xr_q<=xr_q-6'd7;
						end
					end
				end
				if (!i_de && de_q) begin
					if (x_q!=1920) begin bad_q<=1;if(!bad_q)o_abort<=1;end
					x_q<=0;y_q<=y_q+1'b1;xr_q<=60;col_q<=0;hr_q<=0;hg_q<=0;hb_q<=0;
					if (crop) begin
						if (yr_q<=168) begin yr_q<=yr_q+10'd523;row_q<=row_q+1'b1;end
						else yr_q<=yr_q-10'd168;
					end
				end
			end
		end
	end
	// 固定分母逐位除法流水，8 拍延迟、1 拍吞吐，无动态 / 或 %。
	// 四舍六入五成双；与 INTER_AREA 的浮点累加差异单独计入报告。
	wire [7:0] rr,gg,bb;
	wire [33:0] meta_out;
	wire rounded_valid;
	assign o_valid=rounded_valid && !bad_q;
	host_f14_area_round u_r (.i_clk(i_clk),.i_rst_n(i_rst_n),.i_clear(i_vs&&!vs_q),.i_valid(v3&&emit3),.i_sum(r3),.i_meta({id3,row3,col3,2'd0}),.o_valid(rounded_valid),.o_value(rr),.o_meta(meta_out));
	host_f14_area_round u_g (.i_clk(i_clk),.i_rst_n(i_rst_n),.i_clear(i_vs&&!vs_q),.i_valid(v3&&emit3),.i_sum(g3),.i_meta(34'd0),.o_valid(),.o_value(gg),.o_meta());
	host_f14_area_round u_b (.i_clk(i_clk),.i_rst_n(i_rst_n),.i_clear(i_vs&&!vs_q),.i_valid(v3&&emit3),.i_sum(b3),.i_meta(34'd0),.o_valid(),.o_value(bb),.o_meta());
	assign o_rgb={rr,gg,bb};
	assign o_rgb565={rr[7:3],gg[7:2],bb[7:3]};
	assign o_frame_id=meta_out[33:18];
	assign o_row={1'b0,meta_out[17:10]};
	assign o_col={2'd0,meta_out[9:2]};
endmodule

module host_f14_area_round (
	input wire i_clk,input wire i_rst_n,input wire i_clear,input wire i_valid,
	input wire [23:0] i_sum,input wire [33:0] i_meta,
	output wire o_valid,output wire [7:0] o_value,output wire [33:0] o_meta
);
	wire [23:0] rems [0:8];
	wire [7:0] quot [0:8];
	wire [33:0] meta [0:8];
	wire [8:0] valid;
	assign rems[0]=i_sum;assign quot[0]=0;assign meta[0]=i_meta;assign valid[0]=i_valid;
	genvar s;
	generate for(s=0;s<8;s=s+1) begin:g_div
		localparam [23:0] DIVISOR=41460<<(7-s);
		reg [23:0] r;reg [7:0] q;reg [33:0] m;reg v;
		always @(posedge i_clk or negedge i_rst_n) begin
			if(!i_rst_n) begin r<=0;q<=0;m<=0;v<=0;end
			else begin
				v<=!i_clear&&valid[s];m<=meta[s];
				if(rems[s]>=DIVISOR) begin r<=rems[s]-DIVISOR;q<=quot[s]|(8'd1<<(7-s));end
				else begin r<=rems[s];q<=quot[s];end
			end
		end
		assign rems[s+1]=r;assign quot[s+1]=q;assign meta[s+1]=m;assign valid[s+1]=v;
	end endgenerate
	assign o_value=quot[8]+((rems[8]>20730)||(rems[8]==20730&&quot[8][0]));
	assign o_valid=valid[8];assign o_meta=meta[8];
endmodule
`default_nettype wire
