//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-11-16
// Design Name: 四路真实显示提交新鲜度监视
// Module Name: quad_source_freshness_ctrl
// Project Name: PG2L100H ISP Weather Adaptive v026
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Description:
// DDR域内仅使用真实完整新帧进入DISPLAYING的单拍事件续期。
// 旧帧可继续显示；超时只撤销供ISP统计与去雾使用的来源有效侧带。
// 参数C_TIMEOUT_FRAMES范围1..63，默认60个输出显示边界，720p60约1秒。
// 同拍新提交优先于超时；有效纯黑与其他内容使用相同提交事件。
// Dependencies: 已在DDR域的display_switch和输出显示边界，无新增CDC/IP。
// Revision:
// Revision 0.01 - 增加4路独立6位饱和年龄及有效位，共28个寄存器位。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_source_freshness_ctrl
#(
	parameter integer C_TIMEOUT_FRAMES = 60
)
(
	input i_clk,
	input i_rstn,
	input i_frame_boundary,
	input [3:0] i_display_switch,
	output [3:0] o_source_fresh
);
	//-------------配置参数区域-------------//
	localparam [5:0] C_LAST_AGE = C_TIMEOUT_FRAMES - 1;

	//---------------输出信号---------------//
	reg [3:0] source_fresh_o;

	//-------------输出信号连线-------------//
	assign o_source_fresh = source_fresh_o;

	//--------------生成块区域--------------//
	genvar source_index;
	generate
		for(source_index = 0; source_index < 4; source_index = source_index + 1)begin:gen_source
			reg [5:0] cnt_age;

			// 无新完整显示提交时按显示帧边界计龄，不由重复读DDR续期。
			always@(posedge i_clk or negedge i_rstn)begin
				if(!i_rstn)begin
					cnt_age <= 6'd0;
					source_fresh_o[source_index] <= 1'b0;
				end else if(i_display_switch[source_index])begin
					cnt_age <= 6'd0;
					source_fresh_o[source_index] <= 1'b1;
				end else if(i_frame_boundary && source_fresh_o[source_index])begin
					if(cnt_age >= C_LAST_AGE)begin
						source_fresh_o[source_index] <= 1'b0;
					end else begin
						cnt_age <= cnt_age + 6'd1;
					end
				end
			end
		end
	endgenerate
endmodule
