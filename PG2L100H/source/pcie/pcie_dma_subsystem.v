//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-06
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 集成独立 PCIe 物理接口、官方 PCIe_IP、参考小包 DMA、真实 RAM 与视频 DMA 子系统，完成 BAR1 各控制及遥测窗口分流。
// Module Name: pcie_dma_subsystem
// File Name: source/pcie/pcie_dma_subsystem.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 集成独立 PCIe 物理接口、官方 PCIe_IP、参考小包 DMA、真实 RAM 与视频 DMA 子系统，完成 BAR1 各控制及遥测窗口分流。
// 视频与旧 DMA 的 MWr TLP 按整包仲裁；同时连接图像控制、行为判定、HSST 车道状态和动作邮箱，不复用 DDR 参考时钟。
//
// Dependencies: pcie_image_control_bridge、pcie_vehicle_telemetry_regs、pcie_video_subsystem、pcie_video_tx_arbiter、vehicle_behavior_decision、rt_pcie_dma、rt_gs_dma_interface_conversion、vehicle_behavior_action_cdc、PCIe_IP、hsst_rst_cross_sync_v1_0、ram_128in_128out、ram_16in_16out、fiber_lane_status_cdc；PCIe 官方 rt_bridge_top 合成/仿真加密网表。
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
// Create Date: 2026-10-06
// Design Name: pcie_dma_subsystem
// Module Name: pcie_dma_subsystem
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2
// Description: 独立 PCIe 物理接口、官方 PCIe_IP、参考小包 DMA 和真实 RAM
//              的适配层。T04加入视频CSR、官方MWr组包与包级仲裁，不复用DDR ref_clk。
// Dependencies: PCIe_IP、rt_pcie_dma、rt_gs_dma_interface_conversion、
//              PNR 用 rt_bridge_top_v1_3_vpAll.vp、仿真用
//              simulation/rt_bridge_top_sim_v1_3_vpAll.vp、
//              ram_128in_128out、ram_16in_16out。
// Revision: 0.02 - 2026-10-12 BAR1 HUD控制CSR分流，不改物理IP/视频协议。
// Revision: 0.03 - 2026-10-21 共享场景事件、参考计时与行为会话复位。
// Revision: 0.04 - 2026-10-23 复用HSST状态mailbox参与左右变道，不新增PCIe跨域。
// Revision: 0.05 - 2026-10-28 新增只读车道/用户行为一致性遥测窗口。
// Revision: 0.06 - 2026-11-01 BAR1 0x6c0..0x6df动作CSR读分流及视频写隔离。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_dma_subsystem #(
	// DMA 用户地址保留 64 位；片上 staging RAM 仍在实例内部取低地址位。
	parameter integer C_ADDR_WIDTH = 64,
	parameter integer C_RD_DATA_DELAY = 1
)(
	input i_ref_clk,
	input i_image_control_clk,
	input i_image_control_rst_n,
	output o_isp_ext_valid,
	output o_isp_ext_query,
	output [519:0] o_isp_ext_config,
	input i_isp_ext_ready,
	input i_isp_ext_done,
	input [7:0] i_isp_ext_status,
	input [519:0] i_isp_ext_snapshot,
	input [15:0] i_isp_ext_epoch,
	input [1:0] i_isp_ext_source,
	output o_algo_ext_valid,
	output o_algo_ext_query,
	output [63:0] o_algo_ext_config,
	input i_algo_ext_ready,
	input i_algo_ext_done,
	input [7:0] i_algo_ext_status,
	input [63:0] i_algo_ext_snapshot,
	input [15:0] i_algo_ext_epoch,
	input [1:0] i_algo_ext_source,
	input [63:0] i_fiber_lane_snapshot,
	input i_fiber_lane_valid,
	input i_fiber_lane_stale,
	input i_control_rst_n,
	input i_scene_commit_valid,
	input [111:0] i_scene_commit_payload,
	output [175:0] o_display_snapshot,
	input i_video_pixel_clk,
	input i_video_pixel_rst_n,
	input i_video_vs,
	input i_video_hs,
	input i_video_de,
	input [23:0] i_video_rgb,
	input i_pcie_ref_clk_p,
	input i_pcie_ref_clk_n,
	input i_pcie_button_rst_n,
	input i_pcie_perst_n,
	input [3:0] i_pcie_rxp,
	input [3:0] i_pcie_rxn,
	input i_action_clk,
	input i_action_valid,
	input [15:0] i_action_sequence,
	input [2:0] i_action_type,
	input i_action_request,
	output o_action_ready,
	output o_action_online,
	output [3:0] o_pcie_txp,
	output [3:0] o_pcie_txn,
	output o_pcie_pclk,
	output o_pcie_pclk_div2,
	output o_pcie_core_rst_n,
	output o_pcie_smlh_link_up,
	output o_pcie_rdlh_link_up,
	output [4:0] o_pcie_ltssm_state,
	output o_pcie_link_up,
	output o_hud_wr_en,
	output [11:0] o_hud_wr_addr,
	output [31:0] o_hud_wr_data,
	output [3:0] o_hud_wr_strb,
	output [11:0] o_hud_rd_addr,
	input [127:0] i_hud_rd_window,
	output [31:0] o_behavior_result,
	output [31:0] o_behavior_effective
);
	localparam [31:0] C_ACTION_HOLD_MS = 32'd1001;
	localparam [31:0] C_HSST_STALE_MS = 32'd500;

	//-------------PCIe IP 时钟、复位和链路-------------//
	wire pcie_pclk;
	wire pcie_pclk_div2;
	wire pcie_ref_clk;
	wire pcie_core_rst_n;
	wire sync_button_rst_n;
	wire sync_perst_n;
	wire smlh_link_up;
	wire rdlh_link_up;
	wire [4:0] smlh_ltssm_state;

	//-------------PCIe AXIS 主通道-------------//
	wire axis_master_tvalid;
	wire axis_master_tready;
	wire [127:0] axis_master_tdata;
	wire [3:0] axis_master_tkeep;
	wire axis_master_tlast;
	wire [7:0] axis_master_tuser;
	wire axis_slave0_tready;
	wire axis_slave1_tready;
	wire axis_slave2_tready;
	wire axis_slave0_tvalid;
	wire axis_slave1_tvalid;
	wire axis_slave2_tvalid;
	wire [127:0] axis_slave0_tdata;
	wire [127:0] axis_slave1_tdata;
	wire [127:0] axis_slave2_tdata;
	wire axis_slave0_tlast;
	wire axis_slave1_tlast;
	wire axis_slave2_tlast;
	wire axis_slave0_tuser_ref;
	wire axis_slave1_tuser_ref;
	wire axis_slave2_tuser_ref;

	//-------------PCIe 配置输出-------------//
	wire [7:0] cfg_pbus_num;
	wire [4:0] cfg_pbus_dev_num;
	wire [2:0] cfg_max_rd_req_size;
	wire [2:0] cfg_max_payload_size;
	wire cfg_rcb;
	wire cfg_bus_master_en;
	wire video_mode;
	wire legacy_busy;
	wire legacy_reject_event;
	wire video_csr_wr_en;
	wire [11:0] video_csr_wr_addr;
	wire [31:0] video_csr_wr_data;
	wire [3:0] video_csr_wr_strb;
	wire [11:0] video_csr_rd_addr;
	wire [127:0] video_csr_rd_data;
	wire [127:0] video_regs_rd_data;
	wire [127:0] behavior_regs_rd_data;
	wire [127:0] fiber_lane_regs_rd_data;
	wire [127:0] telemetry_regs_rd_data;
	wire [191:0] telemetry_snapshot;
	wire telemetry_rst_n;
	wire flag_telemetry_read_window;
	wire flag_telemetry_write_window;
	wire [127:0] image_regs_rd_data;
	wire flag_image_write_window;
	wire flag_image_read_window;
	wire [20:0] action_payload_net /* synthesis syn_preserve = 1 */;
	wire action_valid_pcie;
	wire [20:0] action_payload_pcie /* synthesis syn_preserve = 1 */;
	wire [31:0] behavior_result;
	wire [31:0] behavior_effective;
	wire flag_hud_write_window;
	wire flag_hud_read_window;
	wire flag_behavior_window;
	wire flag_action_read_window;
	wire flag_action_write_window;
	wire legacy_axis_valid,legacy_axis_ready,legacy_axis_last,legacy_axis_user;
	wire [127:0] legacy_axis_data;
	wire video_axis_valid,video_axis_ready,video_axis_last,video_axis_user;
	wire [127:0] video_axis_data;

	//-------------参考 DMA/RAM 控制-------------//
	wire cpld_finish;
	wire mwr_req_cmd;
	wire mwr_tx_start;
	wire [31:0] rd_base_ram_addr;
	wire [31:0] wr_base_ram_addr;
	wire [10:0] mem_length;
	wire pio_wr_en;
	wire [15:0] pio_wr_byte_en;
	wire [63:0] pio_wr_addr;
	wire [127:0] pio_wr_data;
	wire pio_rd_en;
	wire [63:0] pio_rd_addr;
	wire [127:0] pio_rd_data;
	wire dma_wr_en;
	wire [15:0] dma_wr_byte_en;
	wire [C_ADDR_WIDTH-1:0] dma_wr_addr;
	wire [127:0] dma_wr_data;
	wire dma_rd_en;
	wire [C_ADDR_WIDTH-1:0] dma_rd_addr;
	wire [127:0] dma_rd_data;
	wire dma_ram_wr_en;
	wire [1:0] dma_ram_wr_byte_en;
	wire [C_ADDR_WIDTH-1:0] dma_ram_wr_addr;
	wire [15:0] dma_ram_wr_data;
	wire dma_ram_rd_en;
	wire [C_ADDR_WIDTH-1:0] dma_ram_rd_addr;
	wire [15:0] dma_ram_rd_data;

	// 参考工程的复位去抖模块只处理低有效输入，不复用 DDR 复位。
	hsst_rst_cross_sync_v1_0 u_button_rst_sync(
		.clk(pcie_ref_clk),
		.rstn_in(i_pcie_button_rst_n),
		.rstn_out(sync_button_rst_n)
	);
	hsst_rst_cross_sync_v1_0 u_perst_sync(
		.clk(pcie_ref_clk),
		.rstn_in(i_pcie_perst_n),
		.rstn_out(sync_perst_n)
	);

	// BAR1 0x600..0x67f为HUD；旧视频CSR窗口保持在更高地址。
	assign flag_hud_write_window = video_csr_wr_addr[11:7] == 5'h0c;
	assign flag_hud_read_window = video_csr_rd_addr[11:7] == 5'h0c;
	assign flag_image_write_window = video_csr_wr_addr[11:8] == 4'h7;
	assign flag_image_read_window = video_csr_rd_addr[11:8] == 4'h7;
	// BAR1 0x6c0..0x6df独立动作窗口，既有只读遥测与650/654诊断地址保持原义。
	assign flag_action_read_window = video_csr_rd_addr[11:5] == 7'h36;
	assign flag_action_write_window = video_csr_wr_addr[11:5] == 7'h36;
	assign flag_telemetry_read_window = video_csr_rd_addr[11:4] == 8'h69 ||
		video_csr_rd_addr[11:4] == 8'h6a || video_csr_rd_addr[11:4] == 8'h6b;
	assign flag_telemetry_write_window = video_csr_wr_addr[11:4] == 8'h69 ||
		video_csr_wr_addr[11:4] == 8'h6a || video_csr_wr_addr[11:4] == 8'h6b;
	// 与decision的既有会话复位完全相同，不延迟业务会话/动作的释放。
	assign telemetry_rst_n = i_pcie_button_rst_n && pcie_core_rst_n && i_control_rst_n && rdlh_link_up;
	assign flag_behavior_window = (video_csr_wr_addr >= 12'h650 && video_csr_wr_addr <= 12'h664) ||
		(video_csr_rd_addr >= 12'h650 && video_csr_rd_addr <= 12'h664);
	assign o_hud_wr_en = video_csr_wr_en && flag_hud_write_window &&
		!(video_csr_wr_addr >= 12'h650 && video_csr_wr_addr <= 12'h664);
	assign o_hud_wr_addr = video_csr_wr_addr;
	assign o_hud_wr_data = video_csr_wr_data;
	assign o_hud_wr_strb = video_csr_wr_strb;
	assign o_hud_rd_addr = video_csr_rd_addr;
	// BAR1 0x680..0x68f新增只读状态，不参与任何写地址解码。
	assign video_csr_rd_data = flag_image_read_window ? image_regs_rd_data :
		(video_csr_rd_addr[11:4] == 8'h68) ? fiber_lane_regs_rd_data :
		flag_telemetry_read_window ? telemetry_regs_rd_data :
		flag_action_read_window ? behavior_regs_rd_data :
		(video_csr_rd_addr >= 12'h650 && video_csr_rd_addr <= 12'h664) ?
		behavior_regs_rd_data : (flag_hud_read_window ? i_hud_rd_window : video_regs_rd_data);
	assign action_payload_net = {i_action_request,1'b0,i_action_type,i_action_sequence};
	assign o_behavior_result = behavior_result;
	assign o_behavior_effective = behavior_effective;

	// 新窗口在参考TLP bridge的既有0x400..0x7ff范围内，无需改官方IP。
	// PCIe掉链只清新增会话；control域已接收事务由唯一像素CDC继续排空。
	pcie_image_control_bridge u_image_control_bridge(
		.i_pcie_clk(pcie_pclk_div2),
		.i_pcie_rst_n(pcie_core_rst_n),
		.i_link_up(smlh_link_up && rdlh_link_up),
		.i_control_clk(i_image_control_clk),
		.i_control_rst_n(i_image_control_rst_n),
		.i_wr_en(video_csr_wr_en && flag_image_write_window),
		.i_wr_addr(video_csr_wr_addr),
		.i_wr_data(video_csr_wr_data),
		.i_wr_strb(video_csr_wr_strb),
		.i_rd_addr(video_csr_rd_addr),
		.o_rd_window_data(image_regs_rd_data),
		.o_isp_ext_valid(o_isp_ext_valid),
		.o_isp_ext_query(o_isp_ext_query),
		.o_isp_ext_config(o_isp_ext_config),
		.i_isp_ext_ready(i_isp_ext_ready),
		.i_isp_ext_done(i_isp_ext_done),
		.i_isp_ext_status(i_isp_ext_status),
		.i_isp_ext_snapshot(i_isp_ext_snapshot),
		.i_isp_ext_epoch(i_isp_ext_epoch),
		.i_isp_ext_source(i_isp_ext_source),
		.o_algo_ext_valid(o_algo_ext_valid),
		.o_algo_ext_query(o_algo_ext_query),
		.o_algo_ext_config(o_algo_ext_config),
		.i_algo_ext_ready(i_algo_ext_ready),
		.i_algo_ext_done(i_algo_ext_done),
		.i_algo_ext_status(i_algo_ext_status),
		.i_algo_ext_snapshot(i_algo_ext_snapshot),
		.i_algo_ext_epoch(i_algo_ext_epoch),
		.i_algo_ext_source(i_algo_ext_source)
	);

	// PCIe复位只flush读侧mailbox，光纤原始快照仍由独立sys域维护。
	fiber_lane_status_cdc u_fiber_lane_status_cdc(
		.i_sys_clk(i_ref_clk),
		.i_sys_rst_n(i_pcie_button_rst_n),
		.i_snapshot(i_fiber_lane_snapshot),
		.i_valid(i_fiber_lane_valid),
		.i_stale(i_fiber_lane_stale),
		.i_pcie_clk(pcie_pclk_div2),
		.i_pcie_rst_n(pcie_core_rst_n),
		.o_rd_window(fiber_lane_regs_rd_data)
	);

	vehicle_behavior_action_cdc u_vehicle_behavior_action_cdc(
		.i_src_clk(i_action_clk), .i_src_rst_n(i_pcie_button_rst_n),
		.i_src_valid(i_action_valid), .i_src_payload(action_payload_net),
		.o_src_ready(o_action_ready), .o_src_online(o_action_online),
		.i_dst_clk(pcie_pclk_div2), .i_dst_rst_n(pcie_core_rst_n && i_control_rst_n),
		.i_dst_online(rdlh_link_up && i_control_rst_n),
		.o_dst_valid(action_valid_pcie), .o_dst_payload(action_payload_pcie)
	);
	vehicle_behavior_decision #(
		.C_ACTION_HOLD_MS(C_ACTION_HOLD_MS)
	) u_vehicle_behavior_decision(
		.i_clk(pcie_pclk_div2), .i_rst_n(pcie_core_rst_n), .i_link_up(rdlh_link_up),
		.i_ref_clk(i_ref_clk), .i_ref_rst_n(i_pcie_button_rst_n),
		.i_control_rst_n(i_control_rst_n),
		.i_scene_commit_valid(i_scene_commit_valid),
		.i_scene_commit_payload(i_scene_commit_payload),
		.i_fiber_lane_sequence(fiber_lane_regs_rd_data[31:0]),
		.i_fiber_lane_payload(fiber_lane_regs_rd_data[63:32]),
		.i_fiber_lane_valid(fiber_lane_regs_rd_data[64]),
		.i_fiber_lane_stale(fiber_lane_regs_rd_data[65]),
		.i_wr_en(video_csr_wr_en), .i_wr_addr(video_csr_wr_addr),
		.i_wr_data(video_csr_wr_data), .i_wr_strb(video_csr_wr_strb),
		.i_rd_addr(video_csr_rd_addr), .o_rd_window_data(behavior_regs_rd_data),
		.i_action_valid(action_valid_pcie), .i_action_sequence(action_payload_pcie[15:0]),
		.i_action_type(action_payload_pcie[18:16]), .i_action_request(action_payload_pcie[20]),
		.o_behavior_result(behavior_result), .o_behavior_effective(behavior_effective),
		.o_display_snapshot(o_display_snapshot),
		.o_telemetry_snapshot(telemetry_snapshot)
	);

	// 仅观察已跨域状态；不新增CDC，不改变官方IP、DMA及原HUD读写窗口。
	pcie_vehicle_telemetry_regs #(
		.C_ACTION_HOLD_MS(C_ACTION_HOLD_MS),
		.C_HSST_STALE_MS(C_HSST_STALE_MS)
	) u_vehicle_telemetry_regs(
		.i_clk(pcie_pclk_div2),
		.i_rst_n(telemetry_rst_n),
		.i_snapshot(telemetry_snapshot),
		.i_rd_addr(video_csr_rd_addr),
		.o_rd_window_data(telemetry_regs_rd_data)
	);

	// T04 保留参考小包DMA与PIO；视频CSR在参考bridge解码后分流。
	rt_pcie_dma #(
		.ADDR_WIDTH(C_ADDR_WIDTH),
		.RD_DATA_DELAY(C_RD_DATA_DELAY)
	) u_rt_pcie_dma(
		.i_video_mode(video_mode),
		.o_video_csr_wr_en(video_csr_wr_en),
		.o_video_csr_wr_addr(video_csr_wr_addr),
		.o_video_csr_wr_data(video_csr_wr_data),
		.o_video_csr_wr_strb(video_csr_wr_strb),
		.o_video_csr_rd_addr(video_csr_rd_addr),
		.i_video_csr_rd_data(video_csr_rd_data),
		.o_legacy_busy(legacy_busy),
		.o_legacy_reject_event(legacy_reject_event),
		.clk(pcie_pclk_div2),
		.rst_n(pcie_core_rst_n),
		.i_cfg_pbus_num(cfg_pbus_num),
		.i_cfg_pbus_dev_num(cfg_pbus_dev_num),
		.i_cfg_max_rd_req_size(cfg_max_rd_req_size),
		.i_cfg_max_payload_size(cfg_max_payload_size),
		.i_axis_master_tvld(axis_master_tvalid),
		.o_axis_master_trdy(axis_master_tready),
		.i_axis_master_tdata(axis_master_tdata),
		.i_axis_master_tkeep(axis_master_tkeep),
		.i_axis_master_tlast(axis_master_tlast),
		.i_axis_master_tuser(axis_master_tuser),
		.i_axis_slave0_trdy(axis_slave0_tready),
		.o_axis_slave0_tvld(axis_slave0_tvalid),
		.o_axis_slave0_tdata(axis_slave0_tdata),
		.o_axis_slave0_tlast(axis_slave0_tlast),
		.o_axis_slave0_tuser(axis_slave0_tuser_ref),
		.i_axis_slave1_trdy(axis_slave1_tready),
		.o_axis_slave1_tvld(axis_slave1_tvalid),
		.o_axis_slave1_tdata(axis_slave1_tdata),
		.o_axis_slave1_tlast(axis_slave1_tlast),
		.o_axis_slave1_tuser(axis_slave1_tuser_ref),
		.i_axis_slave2_trdy(legacy_axis_ready),
		.o_axis_slave2_tvld(legacy_axis_valid),
		.o_axis_slave2_tdata(legacy_axis_data),
		.o_axis_slave2_tlast(legacy_axis_last),
		.o_axis_slave2_tuser(legacy_axis_user),
		.i_cpld_finish(cpld_finish),
		.o_mwr_req_cmd(mwr_req_cmd),
		.i_mwr_tx_start(mwr_tx_start),
		.o_rd_base_ram_addr(rd_base_ram_addr),
		.o_wr_base_ram_addr(wr_base_ram_addr),
		.o_mem_length(mem_length),
		.o_pio_wr_en(pio_wr_en),
		.o_pio_wr_byte_en(pio_wr_byte_en),
		.o_pio_wr_addr(pio_wr_addr),
		.o_pio_wr_data(pio_wr_data),
		.o_pio_rd_en(pio_rd_en),
		.o_pio_rd_addr(pio_rd_addr),
		.i_pio_rd_data(pio_rd_data),
		.o_dma_wr_en(dma_wr_en),
		.o_dma_wr_byte_en(dma_wr_byte_en),
		.o_dma_wr_addr(dma_wr_addr),
		.o_dma_wr_data(dma_wr_data),
		.o_dma_rd_en(dma_rd_en),
		.o_dma_rd_addr(dma_rd_addr),
		.i_dma_rd_data(dma_rd_data)
	);

	ram_128in_128out u_pio_ram(
		.wr_data(pio_wr_data),
		.wr_addr(pio_wr_addr[11:0]),
		.wr_en(pio_wr_en),
		.wr_byte_en(pio_wr_byte_en),
		.wr_clk(pcie_pclk_div2),
		.wr_rst(!pcie_core_rst_n),
		.rd_addr(pio_rd_addr[11:0]),
		.rd_data(pio_rd_data),
		.rd_clk(pcie_pclk_div2),
		.rd_clk_en(pio_rd_en),
		.rd_rst(!pcie_core_rst_n)
	);

	rt_gs_dma_interface_conversion #(
		.ADDR_WIDTH(C_ADDR_WIDTH),
		.RD_DATA_DELAY(C_RD_DATA_DELAY)
	) u_dma_interface_conversion(
		.clk(pcie_pclk_div2),
		.rst_n(pcie_core_rst_n),
		.i_dma_wr_en(dma_wr_en),
		.i_dma_wr_byte_en(dma_wr_byte_en),
		.i_dma_wr_addr(dma_wr_addr),
		.i_dma_wr_data(dma_wr_data),
		.o_dma_ram_wr_en(dma_ram_wr_en),
		.o_dma_ram_wr_byte_en(dma_ram_wr_byte_en),
		.o_dma_ram_wr_addr(dma_ram_wr_addr),
		.o_dma_ram_wr_data(dma_ram_wr_data),
		.i_dma_rd_en(dma_rd_en),
		.i_dma_rd_addr(dma_rd_addr),
		.o_dma_rd_data(dma_rd_data),
		.o_dma_ram_rd_en(dma_ram_rd_en),
		.o_dma_ram_rd_addr(dma_ram_rd_addr),
		.i_dma_ram_rd_data(dma_ram_rd_data),
		.o_cpld_finish(cpld_finish),
		.i_mwr_req_cmd(mwr_req_cmd),
		.o_mwr_tx_start(mwr_tx_start),
		.i_rd_base_ram_addr(rd_base_ram_addr),
		.i_wr_base_ram_addr(wr_base_ram_addr),
		.i_mem_length(mem_length)
	);

	ram_16in_16out u_dma_ram(
		.wr_data(dma_ram_wr_data),
		.wr_addr(dma_ram_wr_addr[11:0]),
		.wr_en(dma_ram_wr_en),
		.wr_byte_en(dma_ram_wr_byte_en),
		.wr_clk(pcie_pclk_div2),
		.wr_rst(!pcie_core_rst_n),
		.rd_addr(dma_ram_rd_addr[11:0]),
		.rd_data(dma_ram_rd_data),
		.rd_clk(pcie_pclk_div2),
		.rd_clk_en(dma_ram_rd_en),
		.rd_rst(!pcie_core_rst_n)
	);

	PCIe_IP u_pcie_ip(
		.pclk(pcie_pclk),
		.pclk_div2(pcie_pclk_div2),
		.ref_clk(pcie_ref_clk),
		.ref_clk_n(i_pcie_ref_clk_n),
		.ref_clk_p(i_pcie_ref_clk_p),
		.button_rst_n(sync_button_rst_n),
		.power_up_rst_n(sync_perst_n),
		.perst_n(sync_perst_n),
		.core_rst_n(pcie_core_rst_n),
		.smlh_link_up(smlh_link_up),
		.rdlh_link_up(rdlh_link_up),
		.smlh_ltssm_state(smlh_ltssm_state),
		.p_sel(1'b0), .p_strb(4'd0), .p_addr(16'd0), .p_wdata(32'd0),
		.p_ce(1'b0), .p_we(1'b0), .p_rdy(), .p_rdata(),
		.rxn(i_pcie_rxn), .rxp(i_pcie_rxp), .txn(o_pcie_txn), .txp(o_pcie_txp),
		.pcs_nearend_loop(4'd0), .pma_nearend_ploop(4'd0), .pma_nearend_sloop(4'd0),
		.axis_master_tvalid(axis_master_tvalid),
		.axis_master_tready(axis_master_tready),
		.axis_master_tdata(axis_master_tdata),
		.axis_master_tkeep(axis_master_tkeep),
		.axis_master_tlast(axis_master_tlast),
		.axis_master_tuser(axis_master_tuser),
		.axis_slave0_tready(axis_slave0_tready),
		.axis_slave0_tvalid(axis_slave0_tvalid),
		.axis_slave0_tdata(axis_slave0_tdata),
		.axis_slave0_tlast(axis_slave0_tlast),
		.axis_slave0_tuser(axis_slave0_tuser_ref),
		.axis_slave1_tready(axis_slave1_tready),
		.axis_slave1_tvalid(axis_slave1_tvalid),
		.axis_slave1_tdata(axis_slave1_tdata),
		.axis_slave1_tlast(axis_slave1_tlast),
		.axis_slave1_tuser(axis_slave1_tuser_ref),
		.axis_slave2_tready(axis_slave2_tready),
		.axis_slave2_tvalid(axis_slave2_tvalid),
		.axis_slave2_tdata(axis_slave2_tdata),
		.axis_slave2_tlast(axis_slave2_tlast),
		.axis_slave2_tuser(axis_slave2_tuser_ref),
		.pm_xtlh_block_tlp(),
		.ven_msi_req(1'b0), .ven_msi_tc(3'd0), .ven_msi_vector(5'd0),
		.ven_msi_grant(), .cfg_msi_pending(32'd0), .cfg_msi_en(),
		.cfg_send_cor_err_mux(), .cfg_send_nf_err_mux(), .cfg_send_f_err_mux(),
		.cfg_sys_err_rc(), .cfg_aer_rc_err_mux(), .radm_cpl_timeout(),
		.cfg_max_rd_req_size(cfg_max_rd_req_size), .cfg_bus_master_en(cfg_bus_master_en),
		.cfg_max_payload_size(cfg_max_payload_size), .cfg_ext_tag_en(),
		.cfg_rcb(cfg_rcb), .cfg_mem_space_en(), .cfg_pm_no_soft_rst(),
		.cfg_crs_sw_vis_en(), .cfg_no_snoop_en(), .cfg_relax_order_en(),
		.cfg_tph_req_en(), .cfg_pf_tph_st_mode(), .rbar_ctrl_update(),
		.cfg_atomic_req_en(), .cfg_pbus_num(cfg_pbus_num),
		.cfg_pbus_dev_num(cfg_pbus_dev_num), .radm_idle(), .radm_q_not_empty(),
		.radm_qoverflow(), .diag_ctrl_bus(2'd0), .dyn_debug_info_sel(4'd0),
		.cfg_link_auto_bw_mux(), .cfg_bw_mgt_mux(), .cfg_pme_mux(),
		.debug_info_mux(), .app_ras_des_sd_hold_ltssm(1'b0),
		.app_ras_des_tba_ctrl(2'd0)
	);

	assign o_pcie_pclk = pcie_pclk;
	assign o_pcie_pclk_div2 = pcie_pclk_div2;
	assign o_pcie_core_rst_n = pcie_core_rst_n;
	assign o_pcie_smlh_link_up = smlh_link_up;
	assign o_pcie_rdlh_link_up = rdlh_link_up;
	assign o_pcie_ltssm_state = smlh_ltssm_state;
	assign o_pcie_link_up = rdlh_link_up;

	pcie_video_subsystem u_video(
		.i_clk(pcie_pclk_div2), .i_rst_n(pcie_core_rst_n),
		.i_link_up(rdlh_link_up), .i_bus_master_enable(cfg_bus_master_en),
		.i_requester_id({cfg_pbus_num,cfg_pbus_dev_num,3'd0}),
		.i_cfg_max_payload_size(cfg_max_payload_size),
		.i_pixel_clk(i_video_pixel_clk), .i_pixel_rst_n(i_video_pixel_rst_n),
		.i_hdmi_vs(i_video_vs), .i_hdmi_hs(i_video_hs),
		.i_hdmi_de(i_video_de), .i_hdmi_rgb(i_video_rgb),
		.i_legacy_busy(legacy_busy), .i_legacy_reject_event(legacy_reject_event),
		.i_csr_wr_en(video_csr_wr_en && !flag_hud_write_window && !flag_image_write_window &&
			!flag_telemetry_write_window && !flag_action_write_window), .i_csr_wr_addr(video_csr_wr_addr),
		.i_csr_wr_data(video_csr_wr_data), .i_csr_wr_strb(video_csr_wr_strb),
		.i_csr_rd_addr(video_csr_rd_addr), .o_csr_rd_data(video_regs_rd_data),
		.o_video_mode(video_mode), .o_axis_valid(video_axis_valid),
		.o_axis_data(video_axis_data), .o_axis_last(video_axis_last),
		.o_axis_user(video_axis_user), .i_axis_ready(video_axis_ready)
	);
	pcie_video_tx_arbiter u_video_arbiter(
		.i_clk(pcie_pclk_div2), .i_rst_n(pcie_core_rst_n),
		.i_link_up(rdlh_link_up), .i_video_mode(video_mode),
		.i_legacy_valid(legacy_axis_valid), .i_legacy_data(legacy_axis_data),
		.i_legacy_last(legacy_axis_last), .i_legacy_user(legacy_axis_user),
		.o_legacy_ready(legacy_axis_ready),
		.i_video_valid(video_axis_valid), .i_video_data(video_axis_data),
		.i_video_last(video_axis_last), .i_video_user(video_axis_user),
		.o_video_ready(video_axis_ready), .o_axis_valid(axis_slave2_tvalid),
		.o_axis_data(axis_slave2_tdata), .o_axis_last(axis_slave2_tlast),
		.o_axis_user(axis_slave2_tuser_ref), .i_axis_ready(axis_slave2_tready),
		.o_packet_busy()
	);
endmodule

