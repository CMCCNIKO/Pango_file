//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 集成 F14 Stem、13 个 bottleneck、adapter 与 alignment，复用单个共享 MAC tile 和官方 DRM 特征存储完成骨干推理。
// Module Name: f14_backbone_bram_core
// File Name: model/rtl/f14_backbone_bram_core_overlap_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 集成 F14 Stem、13 个 bottleneck、adapter 与 alignment，复用单个共享 MAC tile 和官方 DRM 特征存储完成骨干推理。
// 通过双输入/双输出槽调度 gather、compute 和 stream 重叠，协调资产加载、几何生成、写回、探针读及 head 宽读端口；按参数支持外部输入图像/ROM和复位分区。
//
// Dependencies: stage_controller、record_fetch、asset_loader、tile_geometry、overlap_scheduler、gather、numeric_tile、streamer、feature_bank、adapter_align 等
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 各子模块在途事务须共同复位；本模块不实例化最终三检测头或 UART，不能把骨干完成当作全部结果完成。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 actual-ROM single-engine backbone core
// Module Name: f14_backbone_bram_core
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Production stage scheduling, same one shared MAC and official RAM.
// Scope: Stem+13 bottlenecks+adapter+alignment; final heads/UART are not connected.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none
module f14_backbone_bram_core #(
	parameter integer C_USE_EXTERNAL_IMAGE = 1,
	parameter integer C_PARTITION_RESET = 0,
	parameter integer C_USE_EXTERNAL_ROM = 1
) (
	input wire clk,
	input wire rst_n,
	input wire [7:0] i_reset_partition_n,
	input wire i_frame_valid,
	input wire [31:0] i_frame_id,
	output wire o_frame_accept,
	output wire o_busy,
	output wire o_backbone_done,
	output wire [31:0] o_frame_id,
	output wire [31:0] o_cycles,
	output wire [9:0] o_error_code,
	output wire o_stage_valid,
	input wire i_stage_ready,
	output wire [3:0] o_stage_index,
	input wire i_store_ready,
	output wire o_store_valid,
	output wire [31:0] o_store_addr,
	output wire [7:0] o_store_data,
	input wire i_head_wide_en,
	input wire [11:0] i_head_wide_addr,
	output wire o_head_wide_valid,
	output wire [127:0] o_head_wide_data,
	input wire i_probe_en,
	input wire i_probe_bank_b,
	input wire [17:0] i_probe_addr,
	output wire o_probe_valid,
	output wire [7:0] o_probe_data,
	output wire o_probe_error,
	output wire o_image_rd_en,
	output wire [15:0] o_image_rd_addr,
	input wire i_image_rd_valid,
	input wire [15:0] i_image_rd_data,
	output wire o_rom_rd_en,
	output wire [16:0] o_rom_rd_addr,
	input wire i_rom_rd_valid,
	input wire [31:0] i_rom_rd_data
);
	wire start,cfg_en,commit;
    wire pipe_done,pipe_error,output_error,repack_busy;
    wire gather_slot,compute_input_slot,compute_output_slot,stream_slot;
    wire [26:0] compute_meta,stream_meta;
    wire gather_write_ready = !repack_busy;
    wire [6:0] compute_y=compute_meta[6:0],compute_x=compute_meta[13:7];
    wire [3:0] compute_h=compute_meta[17:14],compute_w=compute_meta[21:18];
    wire compute_sy=compute_meta[22],compute_sx=compute_meta[23];
    wire compute_fy=compute_meta[24],compute_fx=compute_meta[25];
    wire [6:0] stream_y=stream_meta[6:0],stream_x=stream_meta[13:7];
    wire [3:0] stream_h=stream_meta[17:14],stream_w=stream_meta[21:18];
    wire stream_sy=stream_meta[22],stream_sx=stream_meta[23];
    wire output_rd_valid;
    wire [127:0] output_rd_data;
	wire controller_busy,controller_done,controller_stage_valid;
	wire [3:0] controller_stage_index;
	wire [31:0] controller_cycles;
	wire align_ready,align_done,align_error,align_busy,align_rd_en,align_wr_valid;
	wire [17:0] align_rd_addr,align_wr_addr,align_packed_addr;
    reg adapter_packed_q;
    wire wide_allowed = !o_busy && adapter_packed_q;
    // 调试端保留CHW视图；集成生产顶层将字节probe绑0，转换器被裁掉。
    wire [17:0] probe_ch = i_probe_addr / 18'd588;
    wire [17:0] probe_rem = i_probe_addr % 18'd588;
    wire [17:0] probe_y = probe_rem / 18'd28;
    wire [17:0] probe_x = probe_rem % 18'd28;
    wire [17:0] probe_packed = ((probe_y * 18'd112 + (probe_x >> 1) * 18'd8 + (probe_ch >> 3)) << 4)
        + {probe_x[0],probe_ch[2:0]};
    wire [17:0] b_read_addr;
    always@(posedge clk)begin
        if(!rst_n)begin adapter_packed_q<=0; end
        else if(o_frame_accept)begin adapter_packed_q<=0; end
        else if(align_done)begin adapter_packed_q<=1; end
    end
	wire [7:0] align_wr_data;
	wire align_owner;
	reg [31:0] total_cycles_q;
	reg frame_active_q;
	assign align_owner = controller_done || align_busy;
	assign o_busy = controller_busy || align_owner;
	assign o_backbone_done = align_done;
	assign o_cycles = total_cycles_q;
	assign o_stage_valid = controller_stage_valid || align_done;
	assign o_stage_index = align_owner ? 4'd15 : controller_stage_index;
	always@(posedge clk)begin
		if(!rst_n)begin total_cycles_q<=0;frame_active_q<=0;end
		else begin
			if(o_frame_accept)begin total_cycles_q<=0;frame_active_q<=1;end
			else if(frame_active_q && !align_done)begin total_cycles_q<=total_cycles_q+32'd1;end
			if(align_done)begin frame_active_q<=0;end
		end
	end
	wire [3:0] cfg_addr;
	wire [31:0] cfg_wr_data,cfg_geometry;
	wire [3:0] stage;
	wire [1:0] kind;
	wire [6:0] tile_y,tile_x;
	wire [3:0] valid_h,valid_w;
	wire shift_y,shift_x,far_y,far_x,stride2,exp_only;
	wire [7:0] input_h,input_w,input_zero;
	wire [3:0] input_groups;
	wire signed [9:0] input_y,input_x;
	wire source_bank_b;
	wire [15:0] output_h,output_w,output_c;
	wire [6:0] output_h7,output_w7;
	wire [9:0] output_c10;
	wire [31:0] dest_virtual;
	wire [16:0] image_base;
	wire record_request,record_ready,record_done,record_error,record_valid,record_word_ready;
	wire [4:0] record_index,record_word_index;
	wire [31:0] record_word_data;
	wire record_rom_en;
	wire [16:0] record_rom_addr;
	wire asset_request,asset_ready,asset_done,asset_error,asset_valid;
	wire [1:0] asset_kind,loaded_kind;
	wire [16:0] asset_base,asset_rom_addr;
	wire [10:0] asset_count;
	wire [9:0] loaded_addr;
	wire [511:0] loaded_data;
	wire asset_rom_en,rom_valid;
	wire [31:0] rom_data;
	reg [1:0] rom_owner_q;
	reg image_half;
	wire frame_en;
	wire [15:0] frame_addr;
	wire gather_request,stem_ready,stem_done,stem_error,stem_valid;
	wire [9:0] stem_addr;
	wire [127:0] stem_data;
	wire feat_ready,feat_done,feat_error,feat_valid,feat_rd_en,feat_rd_bank;
	wire [17:0] feat_rd_addr;
	wire feat_pair_en;
	wire [17:0] feat_pair_addr;
	wire [7:0] bank_a_pair_data;
	wire [9:0] feat_addr;
	wire [127:0] feat_data;
	wire cfg_ready,start_ready,done,error,token_valid,token_ready,token_error,wvalid;
	wire [31:0] token_uop,accepted,emitted;
	wire [9:0] error_code;
	wire [1:0] wkind;
	wire [11:0] waddr;
	wire [127:0] wdata;
	wire [15:0] wmask;
	wire stream_start,stream_ready,stream_done,stream_error;
	wire b_debug_en,b_debug_valid,d_debug_en,d_debug_valid;
	wire [11:0] b_debug_addr;
	wire [8:0] d_debug_addr;
	wire [127:0] b_debug_data,d_debug_data;
	wire store_valid,store_a,store_b,store_last,store_ready;
	wire [31:0] store_addr;
	wire [7:0] store_data;
	wire [23:0] stream_bytes;
	wire readback_en,readback_bank;
	wire [17:0] readback_addr;
	wire [7:0] bank_a_data,bank_b_data;
	wire bank_a_valid,bank_b_valid,bank_a_error,bank_b_error;
	wire tile_last,geometry_request,geometry_take;
	wire geometry_ready,geometry_valid,geometry_done,geometry_error;
	wire [9:0] controller_error;
	wire image_rd_valid_selected;
	wire [15:0] image_rd_data_selected;
	wire rom_request;
	wire [16:0] rom_request_addr;
	wire rom_valid_selected;
	wire [31:0] rom_data_selected;
	assign o_image_rd_en = (C_USE_EXTERNAL_IMAGE != 0) ? frame_en : 1'b0;
	assign o_image_rd_addr = frame_addr;
	assign image_rd_valid_selected = (C_USE_EXTERNAL_IMAGE != 0)
		? i_image_rd_valid : (rom_valid_selected && rom_owner_q == 2'd3);
	assign image_rd_data_selected = (C_USE_EXTERNAL_IMAGE != 0)
		? i_image_rd_data : (image_half ? rom_data_selected[31:16] : rom_data_selected[15:0]);
	assign rom_request = record_rom_en || asset_rom_en || ((C_USE_EXTERNAL_IMAGE == 0) && frame_en);
	assign rom_request_addr = record_rom_en ? record_rom_addr
		: (asset_rom_en ? asset_rom_addr : (image_base + {2'b00, frame_addr[15:1]}));
	assign o_rom_rd_en = (C_USE_EXTERNAL_ROM != 0) ? rom_request : 1'b0;
	assign o_rom_rd_addr = rom_request_addr;
	assign rom_valid_selected = (C_USE_EXTERNAL_ROM != 0) ? i_rom_rd_valid : rom_valid;
	assign rom_data_selected = (C_USE_EXTERNAL_ROM != 0) ? i_rom_rd_data : rom_data;
	// Reuse the exact same A/B BRAM ports after the adapter controller drains.
	f14_adapter_align u_align (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(controller_done && controller_error==0),.o_job_ready(align_ready),
		.o_job_done(align_done),.o_job_error(align_error),.o_busy(align_busy),
		.o_rd_en(align_rd_en),.o_rd_addr(align_rd_addr),.i_rd_valid(bank_a_valid),.i_rd_data(bank_a_data),
		.o_wr_valid(align_wr_valid),.i_wr_ready(i_store_ready),.o_wr_addr(align_wr_addr),.o_wr_packed_addr(align_packed_addr),.o_wr_data(align_wr_data));
	assign stage=o_stage_index;
	assign exp_only=kind!=2'd1;
	assign stride2=!exp_only&&cfg_geometry[29];
	assign output_h={9'd0,output_h7};
	assign output_w={9'd0,output_w7};
	assign output_c={6'd0,output_c10};
	assign store_ready=i_store_ready;
	assign o_store_valid=align_owner ? align_wr_valid : store_valid;
	assign o_store_addr=align_owner ? (32'h70000+{14'd0,align_wr_addr}) : store_addr;
	assign o_store_data=align_owner ? align_wr_data : store_data;
	assign readback_en=i_probe_en&&(o_stage_valid||!o_busy);
	assign readback_bank=i_probe_bank_b;
	assign readback_addr=i_probe_addr;
    assign b_read_addr = (readback_en && adapter_packed_q) ? probe_packed : (readback_en ? readback_addr : feat_rd_addr);
	assign o_probe_valid=readback_bank?bank_b_valid:bank_a_valid;
	assign o_probe_data=readback_bank?bank_b_data:bank_a_data;
	assign o_probe_error=(i_probe_en&&!readback_en)||(i_head_wide_en&&!wide_allowed)||bank_a_error||bank_b_error;
	assign o_error_code=align_error ? 10'h3d0 : controller_error;
	// 固定一拍ROM响应按发出请求时的所有者分发，不按当前FSM状态猜测。
	always@(posedge clk)begin
		if(record_rom_en)begin rom_owner_q<=2'd1;end
		else if(asset_rom_en)begin rom_owner_q<=2'd2;end
		else if((C_USE_EXTERNAL_IMAGE == 0) && frame_en)begin rom_owner_q<=2'd3;image_half<=frame_addr[0];end
	end
	f14_control_record_fetch u_record (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(record_request),.o_job_ready(record_ready),
		.i_record_index(record_index),.o_job_done(record_done),.o_job_error(record_error),.o_busy(),
		.o_rom_rd_en(record_rom_en),.o_rom_rd_addr(record_rom_addr),
		.i_rom_rd_valid(rom_valid_selected&&rom_owner_q==2'd1),.i_rom_rd_data(rom_data_selected),
		.o_word_valid(record_valid),.i_word_ready(record_word_ready),.o_word_index(record_word_index),.o_word_data(record_word_data));
	f14_backbone_stage_controller u_controller (
		.i_clk(clk),.i_rst_n(rst_n),.i_frame_valid(i_frame_valid&&!i_probe_en&&!align_owner),.i_frame_id(i_frame_id),
		.o_frame_accept(o_frame_accept),.o_busy(controller_busy),.o_backbone_done(controller_done),
		.o_frame_id(o_frame_id),.o_cycles(controller_cycles),.o_error_code(controller_error),
		.o_stage_valid(controller_stage_valid),.i_stage_ready(i_stage_ready),.o_stage_index(controller_stage_index),
		.o_record_valid(record_request),.i_record_ready(record_ready),.o_record_index(record_index),
		.i_record_done(record_done),.i_record_error(record_error),.i_control_valid(record_valid),
		.o_control_ready(record_word_ready),.i_control_index(record_word_index),.i_control_data(record_word_data),
		.o_asset_valid(asset_request),.i_asset_ready(asset_ready),.o_asset_kind(asset_kind),
		.o_asset_base(asset_base),.o_asset_count(asset_count),.i_asset_done(asset_done),.i_asset_error(asset_error),
		.o_cfg_wr_en(cfg_en),.o_cfg_wr_addr(cfg_addr),.o_cfg_wr_data(cfg_wr_data),.i_cfg_ready(cfg_ready),
		.o_cfg_commit(commit),.o_cfg_geometry(cfg_geometry),.o_geometry_valid(geometry_request),.i_geometry_ready(geometry_ready),
		.i_geometry_tile_valid(1'b0),.o_geometry_tile_ready(),.i_geometry_done(pipe_done),.i_geometry_error(geometry_error||pipe_error),
		.o_gather_valid(),.i_gather_ready(stage==0?stem_ready:feat_ready),.i_gather_done(stage==0?stem_done:feat_done),
		.i_gather_error(stem_error||feat_error||bank_a_error||bank_b_error),.o_tile_start(),.i_tile_start_ready(start_ready),
		.i_tile_done(done),.i_tile_error(error||token_error),.o_stream_valid(),.i_stream_ready(stream_ready),
		.i_stream_done(stream_done),.i_stream_error(stream_error||output_error),.o_kind(kind),.o_input_h(input_h),.o_input_w(input_w),
		.o_output_h(output_h7),.o_output_w(output_w7),.o_output_c(output_c10),.o_input_groups(input_groups),.o_input_zero(input_zero),
		.o_source_bank_b(source_bank_b),.o_destination(dest_virtual),.o_image_base(image_base));
	f14_backbone_tile_geometry u_geometry (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(geometry_request),.o_job_ready(geometry_ready),
		.i_kind(kind[1:0]),.i_stride2(stride2),.i_input_h(input_h),
		.i_input_w(input_w),.i_output_h(output_h[6:0]),.i_output_w(output_w[6:0]),
		.o_tile_valid(geometry_valid),.i_tile_ready(geometry_take),.o_tile_y(tile_y),.o_tile_x(tile_x),
		.o_valid_h(valid_h),.o_valid_w(valid_w),.o_shift_y(shift_y),.o_shift_x(shift_x),.o_far_y(far_y),.o_far_x(far_x),
		.o_input_y(input_y),.o_input_x(input_x),.o_last(tile_last),.o_job_done(geometry_done),.o_job_error(geometry_error),.o_busy());
	generate
		if (C_USE_EXTERNAL_ROM == 0) begin : g_internal_rom
			f14_runtime_asset_rom u_assets (
				.i_clk(clk),.i_rst_n(rst_n),.i_rd_en(rom_request),
				.i_rd_addr(rom_request_addr),
				.o_rd_data(rom_data),.o_rd_valid(rom_valid));
		end else begin : g_external_rom
			assign rom_data = 32'd0;
			assign rom_valid = 1'b0;
		end
	endgenerate
	f14_asset_word_loader u_loader (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(asset_request),.o_job_ready(asset_ready),
		.i_job_base(asset_base),.i_job_count(asset_count),.i_job_kind(asset_kind),
		.o_job_done(asset_done),.o_job_error(asset_error),.o_busy(),
		.o_rom_rd_en(asset_rom_en),.o_rom_rd_addr(asset_rom_addr),
		.i_rom_rd_valid(rom_valid_selected && rom_owner_q==2'd2),.i_rom_rd_data(rom_data_selected),
		.o_word_valid(asset_valid),.i_word_ready(cfg_ready),.o_word_kind(loaded_kind),.o_word_addr(loaded_addr),.o_word_data(loaded_data));
	f14_stem_rgb565_gather u_stem_gather (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(gather_request && stage==0),.o_job_ready(stem_ready),
		.i_tile_y(tile_y),.i_tile_x(tile_x),.i_tile_valid_h(valid_h),.i_tile_valid_w(valid_w),
		.o_job_done(stem_done),.o_job_error(stem_error),.o_busy(),
		.o_frame_rd_en(frame_en),.o_frame_rd_addr(frame_addr),
		.i_frame_rd_valid(image_rd_valid_selected),.i_frame_rd_data(image_rd_data_selected),
		.o_word_valid(stem_valid),.i_word_ready(gather_write_ready),.o_word_addr(stem_addr),.o_word_data(stem_data));
	f14_feature_tile_gather #(.C_PAIR_ENABLE(1)) u_feature_gather (
		.i_clk(clk),.i_rst_n(rst_n),.i_job_valid(gather_request && stage!=0),.o_job_ready(feat_ready),
		.i_base_addr(18'd0),.i_bank_b(source_bank_b),.i_input_h(input_h[6:0]),.i_input_w(input_w[6:0]),.i_input_groups(input_groups),
		.i_origin_y(input_y),.i_origin_x(input_x),.i_grid9(!exp_only),.i_input_zero(input_zero),
		.o_job_done(feat_done),.o_job_error(feat_error),.o_busy(),
		.o_rd_en(feat_rd_en),.o_rd_bank_b(feat_rd_bank),.o_rd_addr(feat_rd_addr),
		.i_rd_valid(feat_rd_bank?bank_b_valid:bank_a_valid),.i_rd_data(feat_rd_bank?bank_b_data:bank_a_data),
		.o_word_valid(feat_valid),.i_word_ready(gather_write_ready),.o_word_addr(feat_addr),.o_word_data(feat_data),
		.o_pair_en(feat_pair_en),.o_pair_addr(feat_pair_addr),.i_pair_data(bank_a_pair_data));
	npu_streaming_uop_generator u_uop (
		.clk(clk),.rst_n(rst_n),.start(start),.start_ready(),.cfg_geometry(cfg_geometry),
		.token_valid(token_valid),.token_ready(token_ready),.token_uop(token_uop),
		.busy(),.done(),.status_error(token_error),.emitted_uops());
	// 量化和打包延迟期间允许更多结果在途；容量同步覆盖最坏输出背压。
	// 保留 PQ8 的隔拍完成约束，不改变 MAC、量化或累加顺序。
	npu_streaming_numeric_tile_6_8 #(.QBF_ROW_DEPTH(128),.QBF_ROW_ADDR_WIDTH(7),.PER_CHANNEL_MULTIPLIER(1),
		.BACKEND_WORD_CREDITS(16),.BACKEND_META_DEPTH(16),.BACKEND_META_PTR_WIDTH(4),
		.BACKEND_PQ_FIFO_DEPTH(32),.BACKEND_PQ_FIFO_AW(5),.C_PARTITION_RESET(C_PARTITION_RESET)) dut (
		.clk(clk),.rst_n(C_PARTITION_RESET ? i_reset_partition_n[2] : rst_n),.tile_start(start),.tile_start_ready(start_ready),
        .i_backend_rst_n(i_reset_partition_n[3]),
		.tile_shift_y(compute_sy),.tile_shift_x(compute_sx),.tile_valid_h(compute_h),.tile_valid_w(compute_w),
		.tile_far_pad_y(compute_fy),.tile_far_pad_x(compute_fx),.shortcut_repack(cfg_geometry[18]),.tile_storage_slot(compute_input_slot),
		.cfg_wr_en(cfg_en),.cfg_ready(cfg_ready),.cfg_wr_addr(cfg_addr),.cfg_wr_data(cfg_wr_data),.cfg_commit(commit),.cfg_commit_done(),
		.a_load_en((stem_valid||feat_valid)&&gather_write_ready),.a_load_addr(stage==0?stem_addr:feat_addr),
		.a_load_data(stage==0?stem_data:feat_data),.a_load_storage_slot(gather_slot),
		.pw_load_en(asset_valid&&loaded_kind==0),.pw_load_addr(loaded_addr),.pw_load_data(loaded_data),
		.dw_load_en(asset_valid&&loaded_kind==1),.dw_load_addr(loaded_addr[8:0]),.dw_load_data(loaded_data[127:0]),
		.qbf_load_en(asset_valid&&loaded_kind==2),.qbf_load_addr(loaded_addr),.qbf_load_data(loaded_data[31:0]),
		.qscale_load_en(asset_valid&&loaded_kind==3),.qscale_load_addr(loaded_addr),.qscale_load_data(loaded_data[15:0]),
		.pw_direct_rd_en(),.pw_direct_rd_addr(),.pw_direct_rd_data(512'd0),
		.token_valid(token_valid),.token_ready(token_ready),.token_uop(token_uop),.token_prefetch_data(64'd0),
		.write_valid(wvalid),.write_ready(1'b1),.write_kind(wkind),.write_addr(waddr),.write_data(wdata),.write_byte_valid(wmask),
		.b_debug_en(1'b0),.b_debug_addr(12'd0),.b_debug_valid(),.b_debug_data(),
		.d_debug_en(1'b0),.d_debug_addr(9'd0),.d_debug_storage_slot(1'b0),.d_debug_valid(),.d_debug_data(),
		.tile_done(done),.busy(),.a_compute_write_busy(repack_busy),.status_error(error),.debug_error_code(error_code),
		.accepted_uops(accepted),.accepted_transactions(),.emitted_words(emitted),.expansion_words(),.depthwise_words(),.projection_words());
    f14_backbone_overlap_scheduler u_overlap (
        .i_clk(clk),.i_rst_n(rst_n),.i_start(geometry_request&&geometry_ready),
        .i_geometry_valid(geometry_valid),.i_geometry({tile_last,far_x,far_y,shift_x,shift_y,valid_w,valid_h,tile_x,tile_y}),
        .o_geometry_take(geometry_take),.o_gather_valid(gather_request),.i_gather_ready(stage==0?stem_ready:feat_ready),
        .i_gather_done(stage==0?stem_done:feat_done),.o_gather_slot(gather_slot),
        .o_compute_start(start),.i_compute_ready(start_ready),.i_compute_done(done),
        .o_compute_input_slot(compute_input_slot),.o_compute_output_slot(compute_output_slot),.o_compute_geometry(compute_meta),
        .o_stream_valid(stream_start),.i_stream_ready(stream_ready),.i_stream_done(stream_done),
        .o_stream_slot(stream_slot),.o_stream_geometry(stream_meta),.o_done(pipe_done),.o_error(pipe_error));
    f14_backbone_output_slots u_output_slots (
        .i_clk(clk),.i_rst_n(rst_n),.i_wr_en(wvalid && (exp_only ? wkind==2'd0 : wkind==2'd2)),
        .i_wr_slot(compute_output_slot),.i_wr_addr(waddr),.i_wr_data(wdata),
        .i_rd_en(b_debug_en||d_debug_en),.i_rd_slot(stream_slot),.i_rd_addr(b_debug_en?b_debug_addr:{3'd0,d_debug_addr}),
        .o_rd_valid(output_rd_valid),.o_rd_data(output_rd_data),.o_error(output_error));
    assign b_debug_valid=output_rd_valid;
    assign d_debug_valid=output_rd_valid;
    assign b_debug_data=output_rd_data;
    assign d_debug_data=output_rd_data;
	f14_shared_tile_streamer u_streamer (
		.clk(clk),.rst_n(rst_n),.start(stream_start),.start_ready(stream_ready),.source_b(exp_only),.stride2(stride2),
		.tile_y({9'd0,stream_y}),.tile_x({9'd0,stream_x}),.tile_shift_y(stream_sy),.tile_shift_x(stream_sx),.tile_valid_h(stream_h),.tile_valid_w(stream_w),
		.output_base_addr(dest_virtual),.output_h(output_h),.output_w(output_w),.output_c(output_c),.transaction_last(stream_meta[26]),
		.b_debug_en(b_debug_en),.b_debug_addr(b_debug_addr),.b_debug_valid(b_debug_valid),.b_debug_data(b_debug_data),
		.d_debug_en(d_debug_en),.d_debug_addr(d_debug_addr),.d_debug_valid(d_debug_valid),.d_debug_data(d_debug_data),
		.out_valid(store_valid),.out_ready(store_ready),.out_addr_byte(store_addr),.out_bank_a_valid(store_a),.out_bank_b_valid(store_b),
		.out_data(store_data),.out_last(store_last),.busy(),.done(stream_done),.protocol_error(stream_error),.emitted_bytes(stream_bytes));
	f14_global_feature_bank #(.C_DEPTH(225792),.C_PAIR_ENABLE(1)) u_a (
		.i_clk(clk),.i_rst_n(rst_n),.i_wr_en(store_valid&&store_ready&&store_a),.i_wr_addr(store_addr[17:0]-18'h30000),.i_wr_data(store_data),
		.i_rd_en(align_owner?align_rd_en:(readback_en?!readback_bank:(feat_rd_en&&!feat_rd_bank))),.i_rd_addr(align_owner?align_rd_addr:(readback_en?readback_addr:feat_rd_addr)),
		.o_rd_data(bank_a_data),.o_rd_valid(bank_a_valid),.o_access_error(bank_a_error),
		.i_pair_en(feat_pair_en && !align_owner && !readback_en),.i_pair_addr(feat_pair_addr),.o_pair_data(bank_a_pair_data));
	f14_global_feature_bank_wide u_b (
		.i_clk(clk),.i_rst_n(rst_n),.i_wr_en(align_owner?(align_wr_valid&&i_store_ready):(store_valid&&store_ready&&store_b)),.i_wr_addr(align_owner?align_packed_addr:(store_addr[17:0]-18'h30000)),.i_wr_data(align_owner?align_wr_data:store_data),
		.i_rd_en(readback_en?readback_bank:(feat_rd_en&&feat_rd_bank)),.i_rd_addr(b_read_addr),
		.i_wide_en(i_head_wide_en && wide_allowed),.i_wide_addr(i_head_wide_addr),
		.o_wide_valid(o_head_wide_valid),.o_wide_data(o_head_wide_data),
		.o_rd_data(bank_b_data),.o_rd_valid(bank_b_valid),.o_access_error(bank_b_error));
endmodule
`default_nettype wire
