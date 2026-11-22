//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 用户 SPI0 1-1-1 模式的 LUT 启动读取器，先执行 0x9F JEDEC 检查，再用 0x03 READ 从指定 Flash 地址读取整表。
// Module Name: avm_lut_flash_reader
// File Name: source/avm/avm_lut_flash_reader.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 用户 SPI0 1-1-1 模式的 LUT 启动读取器，先执行 0x9F JEDEC 检查，再用 0x03 READ 从指定 Flash 地址读取整表。
// 每 32 个字节低字节在前打包一个 256-bit DDR 字，WSTRB 全有效、地址按 32-bit word 单位递增 8；只读 Flash，不执行擦写。
//
// Dependencies: ddr_write_owner_arbiter 单拍写客户端、上层 SPI0 管脚/时钟包装。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 创建日期来自原 Revision: 2026-10-13 新建；原头未单列 Create Date。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target: Pango PG2L100H-6FBG676
// Module: avm_lut_flash_reader
// Description: 上电后从用户模式 SPI0 以 1-1-1 方式读取 LUT，并写入现有 DDR LUT 区。
//              事务使用 0x9f JEDEC 检查和 0x03 Read Data；不执行擦写，不使用 CRC/hash。
//              DDR 写格式与 avm_lut_udp_loader 一致：每 32 个字节组成一个 256-bit 单拍，
//              低位字节在前，WSTRB 全有效，地址以 32-bit word 为单位递增 8。
// Dependencies: 现有 ddr_write_owner_arbiter 的单拍 AXI-like 写接口。
// Revision: 2026-10-13 新建
//////////////////////////////////////////////////////////////////////////////////
module avm_lut_flash_reader #(
	parameter [27:0] C_LUT_BASE       = 28'h0300000,
	parameter integer C_FLASH_SELECT  = 0,
	parameter [23:0] C_FLASH_BASE_ADDR = 24'h800000,
	parameter [31:0] C_LUT_BYTES      = 32'd4915200,
	parameter integer C_SPI_DIV        = 4,
	parameter integer C_START_DELAY   = 32,
	parameter integer C_TIMEOUT_CYCLES = 32'd400000000,
	parameter [7:0] C_JEDEC_MFG       = 8'hef,
	parameter [7:0] C_JEDEC_TYPE      = 8'h40,
	parameter [7:0] C_JEDEC_CAPACITY  = 8'h18
)(
	input i_clk,
	input i_rst_n,
	input i_enable,

	inout io_spi0_d0,
	inout io_spi0_d1,
	inout io_spi0_d2,
	inout io_spi0_d3,
	inout io_spi1_d0,
	inout io_spi1_d1,
	inout io_spi1_d2,
	inout io_spi1_d3,
	output o_spi_clk,
	output o_spi0_cs_n,
	output o_spi1_cs_n,

	output o_lut_ready,
	output o_busy,
	output o_error,
	output o_jedec_ok,
	output [31:0] o_bytes_read,

	output [27:0] o_axi_awaddr,
	output [3:0] o_axi_awlen,
	output o_axi_awvalid,
	input i_axi_awready,
	output [255:0] o_axi_wdata,
	output [31:0] o_axi_wstrb,
	output o_axi_wvalid,
	input i_axi_wready
);
	localparam [3:0] ST_RESET = 4'd0;
	localparam [3:0] ST_JCMD  = 4'd1;
	localparam [3:0] ST_JRX   = 4'd2;
	localparam [3:0] ST_GAP   = 4'd3;
	localparam [3:0] ST_RCMD  = 4'd4;
	localparam [3:0] ST_RADDR = 4'd5;
	localparam [3:0] ST_RDATA = 4'd6;
	localparam [3:0] ST_WRITE = 4'd7;
	localparam [3:0] ST_DONE  = 4'd8;
	localparam [3:0] ST_ERROR = 4'd9;

	reg [3:0] state;
	reg [31:0] delay_count;
	reg [31:0] timeout_count;
	reg [31:0] bytes_read_o;
	reg [31:0] bytes_committed;
	reg [4:0] pack_count;
	reg [255:0] pack_data;
	reg [255:0] pack_data_next;
	reg any_non_ff;
	reg jedec_ok_o;
	reg error_o;
	reg ready_o;
	reg [7:0] jedec_mfg;
	reg [7:0] jedec_type;
	reg [7:0] jedec_capacity;

	reg spi_clk_o;
	reg spi_cs_n_o;
	reg spi_d0_oe;
	reg spi_d0_o;
	reg [31:0] spi_div_count;
	reg [7:0] tx_shift;
	reg [7:0] rx_shift;
	reg [2:0] bit_count;
	reg [1:0] addr_count;
	reg [1:0] jedec_count;
	reg [1:0] gap_count;
	reg axi_awvalid_o;
	reg axi_wvalid_o;
	reg [27:0] axi_awaddr_o;
	reg [255:0] axi_wdata_o;

	wire spi_d1_i = (C_FLASH_SELECT == 0) ? io_spi0_d1 : io_spi1_d1;
	wire [7:0] spi_rx_byte = {rx_shift[6:0], spi_d1_i};
	assign io_spi0_d0 = (C_FLASH_SELECT == 0 && spi_d0_oe) ? spi_d0_o : 1'bz;
	assign io_spi0_d1 = 1'bz;
	assign io_spi0_d2 = 1'bz;
	assign io_spi0_d3 = 1'bz;
	assign io_spi1_d0 = (C_FLASH_SELECT != 0 && spi_d0_oe) ? spi_d0_o : 1'bz;
	assign io_spi1_d1 = 1'bz;
	assign io_spi1_d2 = 1'bz;
	assign io_spi1_d3 = 1'bz;
	assign o_spi_clk = spi_clk_o;
	assign o_spi0_cs_n = (C_FLASH_SELECT == 0) ? spi_cs_n_o : 1'b1;
	assign o_spi1_cs_n = (C_FLASH_SELECT != 0) ? spi_cs_n_o : 1'b1;
	assign o_lut_ready = ready_o;
	assign o_busy = (state != ST_DONE) && (state != ST_ERROR) && (state != ST_RESET || delay_count != 0);
	assign o_error = error_o;
	assign o_jedec_ok = jedec_ok_o;
	assign o_bytes_read = bytes_read_o;
	assign o_axi_awaddr = axi_awaddr_o;
	assign o_axi_awlen = 4'd0;
	assign o_axi_awvalid = axi_awvalid_o;
	assign o_axi_wdata = axi_wdata_o;
	assign o_axi_wstrb = 32'hffffffff;
	assign o_axi_wvalid = axi_wvalid_o;

	// pack_count 的动态 part-select只用于32-byte小缓存，不形成整帧存储。
	always @* begin
		pack_data_next = pack_data;
		pack_data_next[pack_count*8 +: 8] = rx_shift;
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			state <= ST_RESET;
			delay_count <= C_START_DELAY;
			timeout_count <= 0;
			bytes_read_o <= 0;
			bytes_committed <= 0;
			pack_count <= 0;
			pack_data <= 0;
			any_non_ff <= 0;
			jedec_ok_o <= 0;
			error_o <= 0;
			ready_o <= 0;
			jedec_mfg <= 0;
			jedec_type <= 0;
			jedec_capacity <= 0;
			spi_clk_o <= 0;
			spi_cs_n_o <= 1;
			spi_d0_oe <= 0;
			spi_d0_o <= 0;
			spi_div_count <= 0;
			tx_shift <= 0;
			rx_shift <= 0;
			bit_count <= 0;
			addr_count <= 0;
			jedec_count <= 0;
			gap_count <= 0;
			axi_awvalid_o <= 0;
			axi_wvalid_o <= 0;
			axi_awaddr_o <= C_LUT_BASE;
			axi_wdata_o <= 0;
		end else begin
			// 全局超时有明确错误出口，完成态不再计时。
			if(i_enable && state != ST_DONE && state != ST_ERROR && timeout_count < C_TIMEOUT_CYCLES)
				timeout_count <= timeout_count + 1'b1;
			if(state == ST_WRITE && timeout_count >= C_TIMEOUT_CYCLES-1) begin
				error_o <= 1'b1;
			end
			if(state != ST_WRITE && state != ST_DONE && state != ST_ERROR && timeout_count >= C_TIMEOUT_CYCLES-1) begin
				state <= ST_ERROR;
				error_o <= 1'b1;
				spi_cs_n_o <= 1'b1;
				spi_clk_o <= 1'b0;
				spi_d0_oe <= 1'b0;
			end else begin
				case(state)
				ST_RESET: begin
					spi_cs_n_o <= 1'b1;
					spi_clk_o <= 1'b0;
					spi_d0_oe <= 1'b0;
					if(!i_enable) begin delay_count <= C_START_DELAY; end
					else if(C_LUT_BYTES == 0 || (C_LUT_BYTES % 32) != 0 || C_SPI_DIV < 2 || {8'd0,C_FLASH_BASE_ADDR} + C_LUT_BYTES > 32'h01000000) begin state <= ST_ERROR; error_o <= 1; end
					else if(delay_count != 0) delay_count <= delay_count - 1'b1;
					else begin
						state <= ST_JCMD;
						spi_cs_n_o <= 1'b0;
						spi_d0_oe <= 1'b1;
						spi_d0_o <= 1'b1;
						tx_shift <= 8'h9f;
						bit_count <= 3'd7;
					end
				end
				ST_JCMD: begin
					spi_d0_oe <= 1'b1;
					if(spi_div_count == C_SPI_DIV-1) begin
						spi_div_count <= 0;
						if(!spi_clk_o) begin
							spi_clk_o <= 1'b1;
						end else begin
							spi_clk_o <= 1'b0;
							if(bit_count == 0) begin
								state <= ST_JRX; bit_count <= 3'd7; rx_shift <= 0; jedec_count <= 0; spi_d0_oe <= 1'b0;
							end else begin bit_count <= bit_count - 1'b1; spi_d0_o <= tx_shift[bit_count-1'b1]; end
						end
					end else spi_div_count <= spi_div_count + 1'b1;
				end
				ST_JRX: begin
					spi_d0_oe <= 1'b0;
					if(spi_div_count == C_SPI_DIV-1) begin
						spi_div_count <= 0;
						if(!spi_clk_o) begin
							spi_clk_o <= 1'b1;
							rx_shift <= {rx_shift[6:0],spi_d1_i};
						end else begin
							spi_clk_o <= 1'b0;
							if(bit_count == 0) begin
								if(jedec_count == 0) jedec_mfg <= rx_shift;
								if(jedec_count == 1) jedec_type <= rx_shift;
								if(jedec_count == 2) begin
									jedec_capacity <= rx_shift;
									jedec_ok_o <= ((jedec_mfg == C_JEDEC_MFG) || (jedec_mfg == 8'h0b)) &&
										(jedec_type == C_JEDEC_TYPE) &&
										(rx_shift == C_JEDEC_CAPACITY);
									spi_cs_n_o <= 1'b1; gap_count <= 0; state <= ST_GAP;
								end else begin
									jedec_count <= jedec_count + 1'b1; bit_count <= 3'd7; rx_shift <= 0;
								end
							end else bit_count <= bit_count - 1'b1;
						end
					end else spi_div_count <= spi_div_count + 1'b1;
				end
				ST_GAP: begin
					spi_clk_o <= 0; spi_cs_n_o <= 1; spi_d0_oe <= 0;
					if(gap_count == 2) begin
						if(!jedec_ok_o) begin state <= ST_ERROR; error_o <= 1; end
						else begin
							state <= ST_RCMD; spi_cs_n_o <= 0; spi_d0_oe <= 1; spi_d0_o <= 0; tx_shift <= 8'h03; bit_count <= 7;
						end
					end else gap_count <= gap_count + 1'b1;
				end
				ST_RCMD: begin
					if(spi_div_count == C_SPI_DIV-1) begin
						spi_div_count <= 0;
						if(!spi_clk_o) begin spi_clk_o <= 1; end
						else begin
							spi_clk_o <= 0;
							if(bit_count == 0) begin state <= ST_RADDR; bit_count <= 7; addr_count <= 0; tx_shift <= C_FLASH_BASE_ADDR[23:16]; spi_d0_o <= C_FLASH_BASE_ADDR[23]; end
							else begin bit_count <= bit_count - 1'b1; spi_d0_o <= tx_shift[bit_count-1'b1]; end
						end
					end else spi_div_count <= spi_div_count + 1'b1;
				end
				ST_RADDR: begin
					spi_d0_oe <= 1;
					if(spi_div_count == C_SPI_DIV-1) begin
						spi_div_count <= 0;
						if(!spi_clk_o) begin spi_clk_o <= 1; end
						else begin
							spi_clk_o <= 0;
							if(bit_count == 0) begin
										if(addr_count == 0) begin addr_count <= 1; tx_shift <= C_FLASH_BASE_ADDR[15:8]; bit_count <= 7; spi_d0_o <= C_FLASH_BASE_ADDR[15]; end
										else if(addr_count == 1) begin addr_count <= 2; tx_shift <= C_FLASH_BASE_ADDR[7:0]; bit_count <= 7; spi_d0_o <= C_FLASH_BASE_ADDR[7]; end
								else begin state <= ST_RDATA; bit_count <= 7; rx_shift <= 0; spi_d0_oe <= 0; end
							end else begin bit_count <= bit_count - 1'b1; spi_d0_o <= tx_shift[bit_count-1'b1]; end
						end
					end else spi_div_count <= spi_div_count + 1'b1;
				end
				ST_RDATA: begin
					spi_d0_oe <= 0;
					if(spi_div_count == C_SPI_DIV-1) begin
						spi_div_count <= 0;
						if(!spi_clk_o) begin spi_clk_o <= 1; rx_shift <= {rx_shift[6:0],spi_d1_i}; end
						else begin
							spi_clk_o <= 0;
							if(bit_count == 0) begin
								pack_data <= pack_data_next;
														if(rx_shift != 8'hff) any_non_ff <= 1;
								bytes_read_o <= bytes_read_o + 1'b1;
								if(pack_count == 31 || bytes_read_o + 1'b1 == C_LUT_BYTES) begin
									axi_awaddr_o <= C_LUT_BASE + (bytes_committed >> 2);
									axi_wdata_o <= pack_data_next;
									axi_awvalid_o <= 1; axi_wvalid_o <= 1; state <= ST_WRITE; pack_count <= 0; pack_data <= 0;
								end else begin pack_count <= pack_count + 1'b1; bit_count <= 7; rx_shift <= 0; end
							end else bit_count <= bit_count - 1'b1;
						end
					end else spi_div_count <= spi_div_count + 1'b1;
				end
				ST_WRITE: begin
					spi_clk_o <= 0; spi_cs_n_o <= 0;
					if(axi_awvalid_o && i_axi_awready) axi_awvalid_o <= 0;
					if(axi_wvalid_o && i_axi_wready) axi_wvalid_o <= 0;
					if((!axi_awvalid_o || i_axi_awready) && (!axi_wvalid_o || i_axi_wready)) begin
						bytes_committed <= bytes_committed + 32;
						if(error_o) begin state <= ST_ERROR; spi_cs_n_o <= 1; end
						else if(bytes_read_o >= C_LUT_BYTES) begin
							spi_cs_n_o <= 1;
							if(any_non_ff) begin ready_o <= 1; state <= ST_DONE; end
							else begin error_o <= 1; state <= ST_ERROR; end
						end else begin
							state <= ST_RDATA; bit_count <= 7; rx_shift <= 0; spi_d0_oe <= 0;
						end
					end
				end
				ST_DONE: begin
					spi_cs_n_o <= 1; spi_clk_o <= 0; spi_d0_oe <= 0; axi_awvalid_o <= 0; axi_wvalid_o <= 0;
				end
				default: begin
					state <= ST_ERROR; error_o <= 1; spi_cs_n_o <= 1; spi_clk_o <= 0; spi_d0_oe <= 0;
				end
				endcase
			end
		end
	end
endmodule
