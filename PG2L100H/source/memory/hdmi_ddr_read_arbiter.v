//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: quad、monitor、AVM 共用 HMIC 的三客户端读仲裁，保持 AR 请求并锁定完整 burst 的返回归属。
// Module Name: hdmi_ddr_read_arbiter
// File Name: source/memory/hdmi_ddr_read_arbiter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// quad、monitor、AVM 共用 HMIC 的三客户端读仲裁，保持 AR 请求并锁定完整 burst 的返回归属。
// HMIC 无 RREADY，各客户端发起 AR 前必须预留 ARLEN+1 个响应空间；紧急显示和 AVM 渲染预算用于选择新请求。
//
// Dependencies: 三个已预留整笔返回缓冲的 DDR 读客户端、HMIC 读口。
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
// Module Name: hdmi_ddr_read_arbiter
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: quad/monitor/AVM共用HMIC；AR保持及完整burst返回归属锁定。
// Contract: 三个client提出AR前各自预留ARLEN+1返回空间。无RREADY。
//////////////////////////////////////////////////////////////////////////////////
module hdmi_ddr_read_arbiter (
	input i_clk,
	input i_rst_n,
	input [83:0] i_client_araddr,
	input [11:0] i_client_arlen,
	input [2:0] i_client_arvalid,
	input [1:0] i_display_urgent,
	input i_avm_budget_ok,
	output [2:0] o_client_arready,
	output [2:0] o_client_rvalid,
	output [255:0] o_client_rdata,
	output [27:0] o_axi_araddr,
	output [3:0] o_axi_arlen,
	output o_axi_arvalid,
	input i_axi_arready,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	output o_busy
);
	localparam [1:0] ST_IDLE=0, ST_AR=1, ST_RETURN=2;
	reg [1:0] state_current, state_next, owner, candidate, round_robin;
	reg [27:0] araddr_o;
	reg [3:0] arlen_o;
	reg [4:0] beats_left;
	reg [3:0] quad_bursts;
	reg [4:0] display_bursts;
	reg candidate_valid;
	wire [2:0] eligible_valid={i_avm_budget_ok && i_client_arvalid[2],i_client_arvalid[1:0]};
	assign o_busy=state_current!=ST_IDLE;
	assign o_axi_araddr=araddr_o;
	assign o_axi_arlen=arlen_o;
	assign o_axi_arvalid=state_current==ST_AR;
	assign o_client_arready=(state_current==ST_AR && i_axi_arready) ? (3'b001<<owner) : 3'b000;
	assign o_client_rvalid=(state_current==ST_RETURN && i_axi_rvalid) ? (3'b001<<owner) : 3'b000;
	assign o_client_rdata=i_axi_rdata;
	always @(*) begin
		candidate=0; candidate_valid=|eligible_valid;
		// quad紧急服务最多连续8个burst，monitor同时饥饿时插入一次服务。
		if(i_client_arvalid[2] && i_avm_budget_ok && display_bursts==16) candidate=2;
		else if(i_client_arvalid[0] && i_display_urgent[0] && !(quad_bursts==8 && i_client_arvalid[1])) candidate=0;
		else if(quad_bursts==8 && i_client_arvalid[1]) candidate=1;
		else if(i_client_arvalid[1] && i_display_urgent[1]) candidate=1;
		else begin
			case(round_robin)
				0: candidate=eligible_valid[0] ? 0 : (eligible_valid[1] ? 1 : 2);
				1: candidate=eligible_valid[1] ? 1 : (eligible_valid[2] ? 2 : 0);
				default: candidate=eligible_valid[2] ? 2 : (eligible_valid[0] ? 0 : 1);
			endcase
		end
		state_next=state_current;
		case(state_current)
			ST_IDLE: if(candidate_valid) state_next=ST_AR;
			ST_AR: if(i_axi_arready) state_next=ST_RETURN;
			ST_RETURN: if(i_axi_rvalid && beats_left==1) state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) state_current<=ST_IDLE;
		else state_current<=state_next;
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin owner<=0; round_robin<=0; araddr_o<=0; arlen_o<=0; beats_left<=0; quad_bursts<=0; display_bursts<=0; end
		else begin
			if(state_current==ST_IDLE && candidate_valid) begin
				owner<=candidate;
				araddr_o<=i_client_araddr[candidate*28+:28];
				arlen_o<=i_client_arlen[candidate*4+:4];
			end
			if(state_current==ST_AR && i_axi_arready) beats_left<={1'b0,arlen_o}+1'b1;
			if(state_current==ST_RETURN && i_axi_rvalid) begin
				beats_left<=beats_left-1'b1;
				if(beats_left==1) begin
					round_robin<=(owner==2) ? 0 : owner+1'b1;
					if(owner!=0) quad_bursts<=0;
					else if(quad_bursts!=8) quad_bursts<=quad_bursts+1'b1;
					if(owner==2) display_bursts<=0;
					else if(display_bursts!=16) display_bursts<=display_bursts+1'b1;
				end
			end
		end
	end
endmodule
