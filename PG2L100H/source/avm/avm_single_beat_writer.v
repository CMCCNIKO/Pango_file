//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 AVM 打包数据或 LUT beat 接入 HMIC 单拍写接口，AWLEN=0，并保持上游地址/数据到真实 WREADY 接受。
// Module Name: avm_single_beat_writer
// File Name: source/avm/avm_single_beat_writer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 AVM 打包数据或 LUT beat 接入 HMIC 单拍写接口，AWLEN=0，并保持上游地址/数据到真实 WREADY 接受。
// 只有实际 W 接受才产生上游 ready/消费；AW 提出后不可取消，只有共同 DDR reset 可清事务，像素/PLL reset 不可中断写入。
//
// Dependencies: HMIC 写口或 ddr_write_owner_arbiter；浅协议寄存器。
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
// Module Name: avm_single_beat_writer
// Project Name: PG2L100H HDMI AVM Monitor
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: AVM打包词到HMIC单beat写接口；实际W接受才消费上游。
// Contract: 上游valid/data/address保持到ready；AW提出后不取消事务。
//           仅共同DDR reset清状态，不用pixel/PLL reset中止已接受写。
// Dependencies: 浅协议FF，无新存储IP；AWLEN=0，不改变quad写模式。
// Revision: 0.01 - 首次实现。
//////////////////////////////////////////////////////////////////////////////////
module avm_single_beat_writer (
	input i_clk,
	input i_rst_n,
	input i_valid,
	input [27:0] i_address,
	input [255:0] i_data,
	output o_ready,
	output o_drained,
	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	input i_axi_awready,
	output [255:0] o_axi_wdata,
	output [31:0] o_axi_wstrb,
	output o_axi_wvalid,
	input i_axi_wready
);
	localparam [1:0] ST_IDLE=2'd0, ST_AW=2'd1, ST_W=2'd2;
	reg [1:0] state_current, state_next;
	reg [27:0] address_o;
	reg [255:0] data_o;
	assign o_ready=(state_current==ST_W) && i_axi_wready;
	assign o_drained=state_current==ST_IDLE;
	assign o_axi_awaddr=address_o;
	assign o_axi_awlen=4'd0;
	assign o_axi_awvalid=state_current==ST_AW;
	assign o_axi_wdata=data_o;
	assign o_axi_wstrb=32'hffffffff;
	assign o_axi_wvalid=state_current==ST_W;
	always@(*)begin
		state_next=state_current;
		case(state_current)
			ST_IDLE: if(i_valid) state_next=ST_AW;
			ST_AW: if(i_axi_awready) state_next=ST_W;
			ST_W: if(i_axi_wready) state_next=ST_IDLE;
			default: state_next=ST_IDLE;
		endcase
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n) state_current<=ST_IDLE;
		else state_current<=state_next;
	end
	// 捕获完整字并贯穿AW/W背压保持；完成事件不来自AW接受。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin address_o<=28'd0;data_o<=256'd0;end
		else if(state_current==ST_IDLE && i_valid)begin
			address_o<=i_address;
			data_o<=i_data;
		end
	end
endmodule
