//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 用 16 项带完整源帧号的近期缓存及两个推理槽副本，将斑马线场景结果与对应推理帧关联；支持图像结果早于或晚于槽申请到达。
// Module Name: host_f14_scene_slots
// File Name: host/rtl/video/host_f14_scene_slots.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 用 16 项带完整源帧号的近期缓存及两个推理槽副本，将斑马线场景结果与对应推理帧关联；支持图像结果早于或晚于槽申请到达。
// 释放后可再次申请；查询必须同时匹配槽所有权、有效位与帧号，缺少同帧场景时输出无效供调用方回退。
//
// Dependencies: 系统域 i_clk/i_rst_n；scene 写入、claim/release/query 槽协议；缓存数据通过独立有效位保护。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 16项带完整源帧号的近期缓存+两个推理槽独立副本；claim早于或晚于图像结果均可。
// 释放后可重新claim；场景结果缺失时valid=0，由调用方明确回退未保护基线。
`timescale 1ns/1ps
`default_nettype none
module host_f14_scene_slots (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_scene_valid,
	input wire [15:0] i_scene_frame,
	input wire i_scene_active,
	input wire i_claim,
	input wire i_claim_slot,
	input wire [15:0] i_claim_frame,
	input wire i_release,
	input wire i_release_slot,
	input wire i_query_slot,
	input wire [15:0] i_query_frame,
	output wire o_valid,
	output wire o_active
);
	reg [16:0] cache[0:15] /* synthesis syn_ramstyle="lut_ram" */;
	reg [15:0] cached_q;
	reg [15:0] tags[0:1];
	reg [1:0] owned_q,valid_q,active_q;
	wire [16:0] old=cache[i_claim_frame[4:1]];
	wire hit=cached_q[i_claim_frame[4:1]] && old[16:1]==i_claim_frame;
	assign o_valid=owned_q[i_query_slot] && valid_q[i_query_slot] && tags[i_query_slot]==i_query_frame;
	assign o_active=o_valid && active_q[i_query_slot];
	always @(posedge i_clk)if(i_scene_valid)cache[i_scene_frame[4:1]]<={i_scene_frame,i_scene_active};
	integer k;
	always @(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin cached_q<=0;tags[0]<=0;tags[1]<=0;owned_q<=0;valid_q<=0;active_q<=0;end
		else begin
			if(i_scene_valid)begin
				cached_q[i_scene_frame[4:1]]<=1;
				for(k=0;k<2;k=k+1)if(owned_q[k] && tags[k]==i_scene_frame)begin valid_q[k]<=1;active_q[k]<=i_scene_active;end
			end
			if(i_release)begin owned_q[i_release_slot]<=0;valid_q[i_release_slot]<=0;end
			if(i_claim)begin
				owned_q[i_claim_slot]<=1;tags[i_claim_slot]<=i_claim_frame;
				valid_q[i_claim_slot]<=hit || (i_scene_valid && i_scene_frame==i_claim_frame);
				active_q[i_claim_slot]<=i_scene_valid && i_scene_frame==i_claim_frame ? i_scene_active : hit && old[0];
			end
		end
	end
endmodule
`default_nettype wire
