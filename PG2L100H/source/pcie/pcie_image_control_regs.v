//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-23
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 实现 BAR1 0x700..0x7ff 的 ISP 与算法 shadow、COMMIT、QUERY、CLEAR 和完成/错误状态寄存器。
// Module Name: pcie_image_control_regs
// File Name: source/pcie/pcie_image_control_regs.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 实现 BAR1 0x700..0x7ff 的 ISP 与算法 shadow、COMMIT、QUERY、CLEAR 和完成/错误状态寄存器。
// DWORD 写入要求完整 strobe 及对齐；COMMIT 冻结载荷，BUSY 维持到 DONE，active 读回最近完成快照，读口输出 16 字节对齐窗口。
//
// Dependencies: 下游 ISP/算法请求应答邮箱；复位由上层协调。
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
// Create Date: 2026-10-23
// Design Name: PCIe ISP and algorithm control registers
// Module Name: pcie_image_control_regs
// Project Name: PG2L100H_AVM_ISP_DUAL_CONTROL
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: BAR1 0x700-0x7ff. DWORD writes require full strobe/alignment.
//              COMMIT freezes a shadow copy; VALID waits for READY and BUSY
//              remains set until DONE. Shadow writes are allowed while busy.
//              Reads return a 16-byte aligned four-DWORD window. Active reads
//              are the last completed transaction snapshot, refreshed by QUERY.
//              CLEAR bit0 clears done/error/last-error; completion is set-dominant.
// Dependencies: Downstream request/response mailboxes, coordinated reset.
// Revision: 0.01 - File Created
//           0.02 - 2026-10-28: Idle payload follows shadow; transaction freezes it.
//                  INVALID and not BUSY payload is not an architectural value.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_image_control_regs
(
	input i_clk,
	input i_rst_n,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window_data,
	output o_isp_valid,
	output o_isp_query,
	output [519:0] o_isp_config,
	input i_isp_ready,
	input i_isp_done,
	input [7:0] i_isp_status,
	input [519:0] i_isp_snapshot,
	input [15:0] i_isp_epoch,
	input [1:0] i_isp_source,
	output o_algo_valid,
	output o_algo_query,
	output [63:0] o_algo_config,
	input i_algo_ready,
	input i_algo_done,
	input [7:0] i_algo_status,
	input [63:0] i_algo_snapshot,
	input [15:0] i_algo_epoch,
	input [1:0] i_algo_source
);
	localparam [127:0] DEFAULT_TILE = {
		8'd0, 10'd256, 10'd256, 10'd256,
		8'd0, 8'd0, 8'd0, 2'd1, 3'd1, 8'd12,
		8'd32, 3'd1, 8'd4, 8'd16, 3'd0, 3'd0, 20'd0
	};
	localparam [519:0] DEFAULT_ISP = {8'h1e, {4{DEFAULT_TILE}}};
	localparam [63:0] DEFAULT_ALGO = 64'h000fe619dc010080;
	localparam [7:0] ERROR_WRITE = 8'd1;
	localparam [7:0] ERROR_BUSY = 8'd3;
	localparam [7:0] ERROR_COMMAND = 8'd4;

	reg [519:0] reg_isp_shadow;
	reg [63:0] reg_algo_shadow;
	reg [4:0] reg_read_index;
	reg [15:0] reg_isp_pending_seq;
	reg [15:0] reg_algo_pending_seq;
	reg flag_isp_busy;
	reg flag_isp_done;
	reg flag_isp_error;
	reg [7:0] reg_isp_error;
	reg [15:0] reg_isp_last_seq;
	reg [519:0] reg_isp_active;
	reg [15:0] reg_isp_epoch;
	reg [1:0] reg_isp_source;
	reg flag_algo_busy;
	reg flag_algo_done;
	reg flag_algo_error;
	reg [7:0] reg_algo_error;
	reg [15:0] reg_algo_last_seq;
	reg [63:0] reg_algo_active;
	reg [15:0] reg_algo_epoch;
	reg [1:0] reg_algo_source;
	reg isp_valid_o;
	reg isp_query_o;
	reg [519:0] isp_config_o;
	reg algo_valid_o;
	reg algo_query_o;
	reg [63:0] algo_config_o;
	reg [127:0] rd_window_data_o;
	reg [11:0] read_address;
	reg [31:0] read_word;
	reg [3:0] read_word_index;
	integer read_lane;
	genvar shadow_index;

	wire flag_window_write;
	wire flag_full_write;
	wire flag_algo_address;
	wire flag_isp_commit;
	wire flag_algo_commit;
	wire flag_command_valid;
	wire flag_isp_known_write;
	wire flag_algo_known_write;
	wire flag_isp_bad_write;
	wire flag_algo_bad_write;

	assign flag_window_write = i_wr_en && i_wr_addr[11:8] == 4'h7;
	assign flag_full_write = i_wr_strb == 4'hf && i_wr_addr[1:0] == 2'b00;
	assign flag_algo_address = i_wr_addr[11:5] == 7'h3b;
	assign flag_isp_commit = flag_window_write && i_wr_addr == 12'h750;
	assign flag_algo_commit = flag_window_write && i_wr_addr == 12'h768;
	assign flag_command_valid = i_wr_data[15:2] == 14'd0 &&
		(i_wr_data[1:0] == 2'b01 || i_wr_data[1:0] == 2'b10);
	assign flag_isp_known_write = i_wr_addr == 12'h70c ||
		(i_wr_addr >= 12'h710 && i_wr_addr <= 12'h74c) ||
		i_wr_addr == 12'h750 || i_wr_addr == 12'h75c || i_wr_addr == 12'h780;
	assign flag_algo_known_write = i_wr_addr == 12'h760 || i_wr_addr == 12'h764 ||
		i_wr_addr == 12'h768 || i_wr_addr == 12'h774;
	assign flag_isp_bad_write = flag_window_write && !flag_algo_address &&
		(!flag_full_write || !flag_isp_known_write ||
		(i_wr_addr == 12'h780 && i_wr_data > 32'd16));
	assign flag_algo_bad_write = flag_window_write && flag_algo_address &&
		(!flag_full_write || !flag_algo_known_write);

	assign o_rd_window_data = rd_window_data_o;
	assign o_isp_valid = isp_valid_o;
	assign o_isp_query = isp_query_o;
	assign o_isp_config = isp_config_o;
	assign o_algo_valid = algo_valid_o;
	assign o_algo_query = algo_query_o;
	assign o_algo_config = algo_config_o;

	// 各DWORD独立译码，忙时也允许准备下一份shadow；COMMIT载荷另行锁存。
	generate
		for(shadow_index = 0; shadow_index < 16; shadow_index = shadow_index + 1)begin:gen_isp_shadow
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					reg_isp_shadow[shadow_index * 32 +: 32] <= DEFAULT_ISP[shadow_index * 32 +: 32];
				end else if(flag_window_write && flag_full_write && i_wr_addr == 12'h710 + shadow_index * 4)begin
					reg_isp_shadow[shadow_index * 32 +: 32] <= i_wr_data;
				end
			end
		end
	endgenerate

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_isp_shadow[519:512] <= DEFAULT_ISP[519:512];
			reg_read_index <= 5'd0;
		end else if(flag_window_write && flag_full_write)begin
			if(i_wr_addr == 12'h70c)begin
				reg_isp_shadow[519:512] <= i_wr_data[7:0];
			end
			if(i_wr_addr == 12'h780 && i_wr_data <= 32'd16)begin
				reg_read_index <= i_wr_data[4:0];
			end
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_algo_shadow <= DEFAULT_ALGO;
		end else if(flag_window_write && flag_full_write)begin
			if(i_wr_addr == 12'h760)begin
				reg_algo_shadow[31:0] <= i_wr_data;
			end
			if(i_wr_addr == 12'h764)begin
				reg_algo_shadow[63:32] <= i_wr_data;
			end
		end
	end

	// 空闲时预取shadow，COMMIT同沿仍锁存最新shadow；在途期间冻结整份载荷。
	// 下游只在VALID && READY采样，CSR读取shadow/active，不读取空闲载荷。
	// 这样520位载荷的CE仅依赖本地寄存的BUSY/VALID，不再经过TLP地址译码。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			isp_config_o <= DEFAULT_ISP;
		end else if(!flag_isp_busy && !isp_valid_o)begin
			isp_config_o <= reg_isp_shadow;
		end
	end

	// ISP命令：READY只表示请求接收，DONE才释放BUSY；错误状态保持至CLEAR。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			isp_valid_o <= 1'b0;
			isp_query_o <= 1'b0;
			reg_isp_pending_seq <= 16'd0;
			flag_isp_busy <= 1'b0;
			flag_isp_done <= 1'b0;
			flag_isp_error <= 1'b0;
			reg_isp_error <= 8'd0;
			reg_isp_last_seq <= 16'd0;
			reg_isp_active <= DEFAULT_ISP;
			reg_isp_epoch <= 16'd0;
			reg_isp_source <= 2'd0;
		end else begin
			if(isp_valid_o && i_isp_ready)begin
				isp_valid_o <= 1'b0;
			end
			if(flag_window_write && flag_full_write && i_wr_addr == 12'h75c && i_wr_data[0])begin
				flag_isp_done <= 1'b0;
				flag_isp_error <= 1'b0;
				reg_isp_error <= 8'd0;
			end
			if(flag_isp_busy && i_isp_done)begin
				isp_valid_o <= 1'b0;
				flag_isp_busy <= 1'b0;
				flag_isp_done <= 1'b1;
				reg_isp_last_seq <= reg_isp_pending_seq;
				reg_isp_active <= i_isp_snapshot;
				reg_isp_epoch <= i_isp_epoch;
				reg_isp_source <= i_isp_source;
				if(i_isp_status != 8'd0)begin
					flag_isp_error <= 1'b1;
					reg_isp_error <= i_isp_status;
				end
			end
			if(flag_isp_bad_write)begin
				flag_isp_error <= 1'b1;
				reg_isp_error <= ERROR_WRITE;
			end else if(flag_isp_commit)begin
				if(!flag_command_valid)begin
					flag_isp_error <= 1'b1;
					reg_isp_error <= ERROR_COMMAND;
					reg_isp_last_seq <= i_wr_data[31:16];
				end else if(flag_isp_busy)begin
					flag_isp_error <= 1'b1;
					reg_isp_error <= ERROR_BUSY;
					reg_isp_last_seq <= i_wr_data[31:16];
				end else begin
					isp_valid_o <= 1'b1;
					isp_query_o <= i_wr_data[1];
					reg_isp_pending_seq <= i_wr_data[31:16];
					flag_isp_busy <= 1'b1;
				end
			end
		end
	end

	// 算法控制与ISP各有一个在途命令，两个通道可以并行等待帧边界。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			algo_valid_o <= 1'b0;
			algo_query_o <= 1'b0;
			algo_config_o <= DEFAULT_ALGO;
			reg_algo_pending_seq <= 16'd0;
			flag_algo_busy <= 1'b0;
			flag_algo_done <= 1'b0;
			flag_algo_error <= 1'b0;
			reg_algo_error <= 8'd0;
			reg_algo_last_seq <= 16'd0;
			reg_algo_active <= DEFAULT_ALGO;
			reg_algo_epoch <= 16'd0;
			reg_algo_source <= 2'd0;
		end else begin
			if(algo_valid_o && i_algo_ready)begin
				algo_valid_o <= 1'b0;
			end
			if(flag_window_write && flag_full_write && i_wr_addr == 12'h774 && i_wr_data[0])begin
				flag_algo_done <= 1'b0;
				flag_algo_error <= 1'b0;
				reg_algo_error <= 8'd0;
			end
			if(flag_algo_busy && i_algo_done)begin
				algo_valid_o <= 1'b0;
				flag_algo_busy <= 1'b0;
				flag_algo_done <= 1'b1;
				reg_algo_last_seq <= reg_algo_pending_seq;
				reg_algo_active <= i_algo_snapshot;
				reg_algo_epoch <= i_algo_epoch;
				reg_algo_source <= i_algo_source;
				if(i_algo_status != 8'd0)begin
					flag_algo_error <= 1'b1;
					reg_algo_error <= i_algo_status;
				end
			end
			if(flag_algo_bad_write)begin
				flag_algo_error <= 1'b1;
				reg_algo_error <= ERROR_WRITE;
			end else if(flag_algo_commit)begin
				if(!flag_command_valid)begin
					flag_algo_error <= 1'b1;
					reg_algo_error <= ERROR_COMMAND;
					reg_algo_last_seq <= i_wr_data[31:16];
				end else if(flag_algo_busy)begin
					flag_algo_error <= 1'b1;
					reg_algo_error <= ERROR_BUSY;
					reg_algo_last_seq <= i_wr_data[31:16];
				end else begin
					algo_valid_o <= 1'b1;
					algo_query_o <= i_wr_data[1];
					algo_config_o <= reg_algo_shadow;
					reg_algo_pending_seq <= i_wr_data[31:16];
					flag_algo_busy <= 1'b1;
				end
			end
		end
	end

	// 四DWORD窗口与既有BAR1 CSR读回延迟接口一致，不在这里新增寄存拍。
	always@(*)begin
		rd_window_data_o = 128'd0;
		read_address = 12'd0;
		read_word = 32'd0;
		read_word_index = 4'd0;
		for(read_lane = 0; read_lane < 4; read_lane = read_lane + 1)begin
			read_address = {i_rd_addr[11:4], read_lane[1:0], 2'b00};
			read_word_index = read_address[5:2] - 4'd4;
			read_word = 32'd0;
			case(read_address)
				12'h700:begin read_word = 32'h31435049; end
				12'h704:begin read_word = 32'd1; end
				12'h708:begin read_word = 32'h0000000f; end
				12'h70c:begin read_word = {24'd0, reg_isp_shadow[519:512]}; end
				12'h754:begin read_word = {reg_isp_last_seq, reg_isp_error, 5'd0, flag_isp_error, flag_isp_done, flag_isp_busy}; end
				12'h758:begin read_word = {14'd0, reg_isp_source, reg_isp_epoch}; end
				12'h760:begin read_word = reg_algo_shadow[31:0]; end
				12'h764:begin read_word = reg_algo_shadow[63:32]; end
				12'h76c:begin read_word = {reg_algo_last_seq, reg_algo_error, 5'd0, flag_algo_error, flag_algo_done, flag_algo_busy}; end
				12'h770:begin read_word = {14'd0, reg_algo_source, reg_algo_epoch}; end
				12'h780:begin read_word = {27'd0, reg_read_index}; end
				12'h784:begin
					if(reg_read_index == 5'd16)begin
						read_word = {24'd0, reg_isp_active[519:512]};
					end else begin
						read_word = reg_isp_active[reg_read_index[3:0] * 32 +: 32];
					end
				end
				12'h788:begin read_word = reg_algo_active[31:0]; end
				12'h78c:begin read_word = reg_algo_active[63:32]; end
				default:begin
					if(read_address >= 12'h710 && read_address <= 12'h74c)begin
						read_word = reg_isp_shadow[read_word_index * 32 +: 32];
					end
				end
			endcase
			rd_window_data_o[read_lane * 32 +: 32] = read_word;
		end
	end
endmodule
