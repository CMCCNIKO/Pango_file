//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: quad、LUT、AVM 三个客户端共用 HMIC 写口的 owner 仲裁器，在提出 AW 前锁定归属，直到 AWLEN 指定的数据全部被 WREADY 接受。
// Module Name: ddr_write_owner_arbiter
// File Name: source/memory/ddr_write_owner_arbiter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// quad、LUT、AVM 三个客户端共用 HMIC 写口的 owner 仲裁器，在提出 AW 前锁定归属，直到 AWLEN 指定的数据全部被 WREADY 接受。
// HMIC 接口无独立 WVALID，数据须在 AW 提出前准备并稳定到接受；地址请求不得撤销，ready 仅返回当前 owner。
//
// Dependencies: 无存储 IP；所有客户端在 DDR 域并使用共同 DDR/button 复位。
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
// Create Date: 2026-10-13
// Module Name: ddr_write_owner_arbiter
// Project Name: PG2L100H HDMI AVM Monitor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: HMIC无WVALID写仲裁；AW提出前锁定owner至全部WREADY消费。
// Contract: source0=原quad，1=LU，2=AVM；均DDR域，共同DDR/button复位。
// 每source必须在AW提出前准备WDATA并在对应WREADY前保持；不允许撤销AW。
// 保留来源AWLEN（原quad仍0），ready仅返owner，不制造quad完成事件。
// Dependencies: 无存储IP，仅控制FF。
// Revision: 0.01
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
module ddr_write_owner_arbiter
(
	input i_clk,
	input i_rst_n,
	input [83:0] i_source_awaddr,
	input [11:0] i_source_awlen,
	input [2:0] i_source_awvalid,
	output [2:0] o_source_awready,
	input [767:0] i_source_wdata,
	input [95:0] i_source_wstrb,
	output [2:0] o_source_wready,
	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	input i_axi_awready,
	output [255:0] o_axi_wdata,
	output [31:0] o_axi_wstrb,
	input i_axi_wready,
	output o_busy,
	output [1:0] o_active_owner
);
	localparam [1:0] ST_IDLE=0, ST_AW=1, ST_W=2;
	reg [1:0] state_current;
	reg [1:0] reg_owner,reg_next;
	reg [1:0] dec_owner;
	reg [27:0] awaddr_o;
	reg [3:0] awlen_o;
	reg [4:0] cnt_beats;
	reg [2:0] source_awready_o,source_wready_o;
	reg [255:0] wdata_o;
	reg [31:0] wstrb_o;
	assign o_source_awready=source_awready_o;
	assign o_source_wready=source_wready_o;
	assign o_axi_awaddr=awaddr_o;
	assign o_axi_awlen=awlen_o;
	assign o_axi_awvalid=state_current==ST_AW;
	assign o_axi_wdata=wdata_o;
	assign o_axi_wstrb=wstrb_o;
	assign o_busy=state_current!=ST_IDLE;
	assign o_active_owner=reg_owner;
	always@(*)begin
		dec_owner=0;
		case(reg_next)
			2'd0:begin
				if(i_source_awvalid[0])begin dec_owner=0;end
				else if(i_source_awvalid[1])begin dec_owner=1;end
				else if(i_source_awvalid[2])begin dec_owner=2;end
			end
			2'd1:begin
				if(i_source_awvalid[1])begin dec_owner=1;end
				else if(i_source_awvalid[2])begin dec_owner=2;end
				else begin dec_owner=0;end
			end
			default:begin
				if(i_source_awvalid[2])begin dec_owner=2;end
				else if(i_source_awvalid[0])begin dec_owner=0;end
				else if(i_source_awvalid[1])begin dec_owner=1;end
			end
		endcase
	end
	always@(*)begin
		source_awready_o=0;source_wready_o=0;
		if(state_current==ST_AW)begin source_awready_o[reg_owner]=i_axi_awready;end
		if(state_current==ST_W)begin source_wready_o[reg_owner]=i_axi_wready;end
		case(reg_owner)
			2'd1:begin wdata_o=i_source_wdata[511:256];wstrb_o=i_source_wstrb[63:32];end
			2'd2:begin wdata_o=i_source_wdata[767:512];wstrb_o=i_source_wstrb[95:64];end
			default:begin wdata_o=i_source_wdata[255:0];wstrb_o=i_source_wstrb[31:0];end
		endcase
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin state_current<=ST_IDLE;reg_owner<=0;reg_next<=0;awaddr_o<=0;awlen_o<=0;cnt_beats<=0;end
		else begin
			case(state_current)
				ST_IDLE:begin
					if(|i_source_awvalid)begin
						reg_owner<=dec_owner;
						awaddr_o<=i_source_awaddr[dec_owner*28+:28];awlen_o<=i_source_awlen[dec_owner*4+:4];
						cnt_beats<={1'b0,i_source_awlen[dec_owner*4+:4]}+1'b1;state_current<=ST_AW;
					end
				end
				ST_AW:begin if(i_axi_awready)begin state_current<=ST_W;end end
				ST_W:begin
					// HMIC消费事件仅WREADY，不存在可撤销的WVALID输入。
					if(i_axi_wready)begin
						if(cnt_beats==1)begin state_current<=ST_IDLE;reg_next<=reg_owner==2 ? 0 : reg_owner+1'b1;end
						else begin cnt_beats<=cnt_beats-1'b1;end
					end
				end
				default:begin state_current<=ST_IDLE;end
			endcase
		end
	end
endmodule
