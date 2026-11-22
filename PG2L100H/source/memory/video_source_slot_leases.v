//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 记录 quad、monitor、AVM 三个读者对 (source,slot,frame_epoch) 的读引用，形成每源每槽 held 屏障。
// Module Name: video_source_slot_leases
// File Name: source/memory/video_source_slot_leases.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 记录 quad、monitor、AVM 三个读者对 (source,slot,frame_epoch) 的读引用，形成每源每槽 held 屏障。
// 每个读者每源最多一个 lease，只有旧 DDR burst 和对应 FIFO 都排空才释放；写端分配 FREE 槽与 retire 时必须检查该屏障。
//
// Dependencies: 外部读者 acquire/release、quad/frame slot manager。
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
// Module Name: video_source_slot_leases
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: quad/monitor/AVM独立(source,slot,frame_epoch)读引用寄存器。
// Contract: 每读者每源最多一个lease；release只在该源旧burst和FIFO排空后。
//           writer/slot manager必须把o_slot_held作为FREE分配/retire释放屏障。
//////////////////////////////////////////////////////////////////////////////////
module video_source_slot_leases (
	input i_clk,
	input i_rst_n,
	input [11:0] i_acquire,
	input [23:0] i_acquire_slot,
	input [191:0] i_acquire_epoch,
	input [11:0] i_release,
	input [3:0] i_completed_valid,
	input [7:0] i_completed_slot,
	input [63:0] i_completed_epoch,
	input i_avm_budget_ok,
	output [11:0] o_acquire_ready,
	output [11:0] o_slot_held,
	output [11:0] o_lease_valid,
	output [23:0] o_lease_slot,
	output [191:0] o_lease_epoch
);
	reg [11:0] lease_valid;
	reg [23:0] lease_slot;
	reg [191:0] lease_epoch;
	reg [11:0] slot_held;
	integer reader_source;
	integer held_index;
	assign o_slot_held=slot_held;
	assign o_lease_valid=lease_valid;
	assign o_lease_slot=lease_slot;
	assign o_lease_epoch=lease_epoch;
	always @(*) begin
		slot_held=12'd0;
		held_index=0;
		for(reader_source=0;reader_source<12;reader_source=reader_source+1) begin
			held_index=(reader_source%4)*3+lease_slot[reader_source*2+:2];
			if(lease_valid[reader_source] && lease_slot[reader_source*2+:2]<3) slot_held[held_index]=1'b1;
		end
	end
	genvar lease_index;
	generate for(lease_index=0;lease_index<12;lease_index=lease_index+1) begin: gen_lease
		// 只能引用published完整帧，旧epoch描述符不能重新取得引用。
		assign o_acquire_ready[lease_index]=(!lease_valid[lease_index] || i_release[lease_index]) &&
			i_completed_valid[lease_index%4] &&
			(i_acquire_slot[lease_index*2+:2]==i_completed_slot[(lease_index%4)*2+:2]) &&
			(i_acquire_epoch[lease_index*16+:16]==i_completed_epoch[(lease_index%4)*16+:16]) &&
			((lease_index<8) || i_avm_budget_ok);
		always @(posedge i_clk or negedge i_rst_n) begin
			if(!i_rst_n) begin
				lease_valid[lease_index]<=0;
				lease_slot[lease_index*2+:2]<=0;
				lease_epoch[lease_index*16+:16]<=0;
			end else begin
				if(i_release[lease_index]) lease_valid[lease_index]<=0;
				if(i_acquire[lease_index] && o_acquire_ready[lease_index] && i_acquire_slot[lease_index*2+:2]<3) begin
					lease_valid[lease_index]<=1;
					lease_slot[lease_index*2+:2]<=i_acquire_slot[lease_index*2+:2];
					lease_epoch[lease_index*16+:16]<=i_acquire_epoch[lease_index*16+:16];
				end
			end
		end
	end endgenerate
endmodule
