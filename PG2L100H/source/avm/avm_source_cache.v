//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: AVM 64KiB 官方 data/tag RAM 缓存封装，连接 cache control 与 DDR 单 beat 填充适配器，对邻域采样提供按序像素响应。
// Module Name: avm_source_cache
// File Name: source/avm/avm_source_cache.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// AVM 64KiB 官方 data/tag RAM 缓存封装，连接 cache control 与 DDR 单 beat 填充适配器，对邻域采样提供按序像素响应。
// 读地址持续预取以匹配 RAM 一拍延迟，停顿时保持 RAM 输出侧结果；源基址及 epoch 由渲染事务冻结。
//
// Dependencies: avm_source_cache_ram、avm_source_cache_tag_ram、avm_source_cache_control、ddr_read_burst_engine。
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
// Module Name: avm_source_cache
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 用户生成64KiB data/tag官方RAM真实封装+1beat DDR填充缓存。
// Dependencies: avm_source_cache_ram/tag_ram真实IP，avm_source_cache_control，ddr_read_burst_engine。
// Revision: 2026-10-23 读地址持续预取；有效读延迟1拍，停顿时在RAM输出侧保持。
//////////////////////////////////////////////////////////////////////////////////
module avm_source_cache (
	input i_clk,
	input i_rst_n,
	input i_cancel,
	input i_invalidate,
	input i_request_valid,
	output o_request_ready,
	input [1:0] i_request_source,
	input [17:0] i_request_pixel_addr,
	input [15:0] i_request_frame_epoch,
	input [27:0] i_request_source_base,
	output o_response_valid,
	input i_response_ready,
	output [15:0] o_response_rgb565,
	output o_response_error,
	output o_initialized,
	output o_idle,
	output [27:0] o_axi_araddr,
	output [3:0] o_axi_arlen,
	output o_axi_arvalid,
	input i_axi_arready,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	output [31:0] o_hit_count,
	output [31:0] o_miss_count
);
	wire tag_read_enable,data_read_enable,tag_write_enable,data_write_enable;
	wire [10:0] tag_read_addr,tag_write_addr,data_write_addr;
	wire [14:0] data_read_addr;
	wire [31:0] tag_read_data,tag_write_data;
	wire [15:0] data_read_data;
	wire [31:0] tag_ram_data;
	wire [15:0] data_ram_data;
	wire [255:0] data_write_data;
	reg flag_tag_read_d;
	reg flag_data_read_d;
	reg [31:0] reg_tag_read_hold;
	reg [15:0] reg_data_read_hold;
	// 把accept/response_ready的门控从高扇出的DRM地址移到读结果有效选择。
	assign tag_read_data = flag_tag_read_d ? tag_ram_data : reg_tag_read_hold;
	assign data_read_data = flag_data_read_d ? data_ram_data : reg_data_read_hold;
	wire burst_request,engine_busy,burst_done,burst_failed,word_valid,word_ready;
	wire [27:0] burst_addr;
	wire [3:0] burst_len;
	wire [255:0] word_data;
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			flag_tag_read_d <= 1'b0;
			flag_data_read_d <= 1'b0;
			reg_tag_read_hold <= 32'd0;
			reg_data_read_hold <= 16'd0;
		end
		else begin
			flag_tag_read_d <= tag_read_enable;
			flag_data_read_d <= data_read_enable;
			if(flag_tag_read_d) begin reg_tag_read_hold <= tag_ram_data; end
			if(flag_data_read_d) begin reg_data_read_hold <= data_ram_data; end
		end
	end
	avm_source_cache_ram u_data_ram(.wr_data(data_write_data),.wr_addr(data_write_addr),
		.wr_en(data_write_enable),.wr_clk(i_clk),.wr_rst(!i_rst_n),
		.rd_addr(data_read_addr),.rd_data(data_ram_data),.rd_clk(i_clk),.rd_rst(!i_rst_n));
	avm_source_cache_tag_ram u_tag_ram(.wr_data(tag_write_data),.wr_addr(tag_write_addr),
		.wr_en(tag_write_enable),.wr_clk(i_clk),.wr_rst(!i_rst_n),
		.rd_addr(tag_read_addr),.rd_data(tag_ram_data),.rd_clk(i_clk),.rd_rst(!i_rst_n));
	avm_source_cache_control u_control(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_cancel(i_cancel),.i_invalidate(i_invalidate),
		.i_request_valid(i_request_valid),.o_request_ready(o_request_ready),.i_request_source(i_request_source),
		.i_request_pixel_addr(i_request_pixel_addr),.i_request_frame_epoch(i_request_frame_epoch),.i_request_source_base(i_request_source_base),
		.o_response_valid(o_response_valid),.i_response_ready(i_response_ready),.o_response_rgb565(o_response_rgb565),.o_response_error(o_response_error),
		.o_initialized(o_initialized),.o_idle(o_idle),.o_tag_read_enable(tag_read_enable),.o_tag_read_addr(tag_read_addr),.i_tag_read_data(tag_read_data),
		.o_data_read_enable(data_read_enable),.o_data_read_addr(data_read_addr),.i_data_read_data(data_read_data),
		.o_tag_write_enable(tag_write_enable),.o_tag_write_addr(tag_write_addr),.o_tag_write_data(tag_write_data),
		.o_data_write_enable(data_write_enable),.o_data_write_addr(data_write_addr),.o_data_write_data(data_write_data),
		.o_burst_request(burst_request),.o_burst_addr(burst_addr),.o_burst_len(burst_len),.i_engine_busy(engine_busy),
		.i_burst_done(burst_done),.i_burst_failed(burst_failed),.i_word_valid(word_valid),.i_word_data(word_data),.o_word_ready(word_ready),
		.o_hit_count(o_hit_count),.o_miss_count(o_miss_count));
	ddr_read_burst_engine u_engine(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_abort(1'b0),
		.i_burst_req(burst_request),.i_start_addr(burst_addr),.i_burst_len(burst_len),.i_output_capacity(16'd16),
		.o_busy(engine_busy),.o_burst_done(burst_done),.o_burst_failed(burst_failed),
		.o_axi_araddr(o_axi_araddr),.o_axi_arlen(o_axi_arlen),.o_axi_arvalid(o_axi_arvalid),
		.i_axi_arready(i_axi_arready),.i_axi_rdata(i_axi_rdata),.i_axi_rvalid(i_axi_rvalid),
		.o_word_valid(word_valid),.o_word_data(word_data),.i_word_ready(word_ready));
endmodule
