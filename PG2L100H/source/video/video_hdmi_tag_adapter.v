//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将四宫格像素及同拍坐标注册一拍并组装统一像素 tag，携带 tile、内容有效、DE、黑条和算法配置。
// Module Name: video_hdmi_tag_adapter
// File Name: source/video/video_hdmi_tag_adapter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将四宫格像素及同拍坐标注册一拍并组装统一像素 tag，携带 tile、内容有效、DE、黑条和算法配置。
// 帧边界位于消隐，坐标和配置随原像素等拍进入后续去雾与算法流水。
//
// Dependencies: quad_video_content_tag 的同拍元数据及像素域配置。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Module: video_hdmi_tag_adapter
// Project: PG2L100H HDMI dehaze; PDS 2022.2-SP6.4
// Device: PG2L100H-6FBG676
// Description: 原quad像素及同拍坐标注册1拍，保留DE/黑条/配置标签。
// Dependencies: quad_video_content_tag 同拍元数据；frame_boundary在消隐。
// Revision: 1.0 初始实现。
//////////////////////////////////////////////////////////////////////////////
module video_hdmi_tag_adapter (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_vs,
	input wire i_hs,
	input wire i_de,
	input wire [23:0] i_rgb,
	input wire [10:0] i_x,
	input wire [9:0] i_y,
	input wire [1:0] i_tile,
	input wire [9:0] i_tile_x,
	input wire [8:0] i_tile_y,
	input wire i_content_valid,
	input wire i_frame_boundary,
	input wire [63:0] i_config,
	input wire [15:0] i_epoch,
	input wire [3:0] i_source_alive,
	input wire i_source_alive_valid,
	output wire [23:0] o_rgb,
	output wire [159:0] o_tag,
	output wire o_frame_boundary,
	output wire [3:0] o_source_alive,
	output wire o_source_alive_valid
);
	reg [31:0] frame_id;
	reg [23:0] rgb_o;
	reg [159:0] tag_o;
	reg boundary_o;
	reg [3:0] alive_o;
	reg alive_valid_o;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin frame_id<=0; rgb_o<=0; tag_o<=0; boundary_o<=0; alive_o<=0; alive_valid_o<=0; end
		else begin
			if(i_frame_boundary) frame_id<=frame_id+1'b1;
			rgb_o<=i_rgb;
			// SOF 使用真实的 VS 下降沿边界；不能从已注册 DE 与另一拍坐标推断。
			// quad_video_window 在消隐期间产生 i_frame_boundary，此时 i_de=0。
			tag_o<={i_vs,i_hs,i_de,i_frame_boundary,(i_de && i_x==1279),
				i_x,i_y,i_tile,i_tile_x,i_tile_y,i_content_valid,frame_id,i_epoch,i_config};
			boundary_o<=i_frame_boundary;
			alive_o<=i_source_alive; alive_valid_o<=i_source_alive_valid;
		end
	end
	assign o_rgb=rgb_o;
	assign o_tag=tag_o;
	assign o_frame_boundary=boundary_o;
	assign o_source_alive=alive_o;
	assign o_source_alive_valid=alive_valid_o;
endmodule
