//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/01 15:38:32
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 以太网 FCS 的 8-bit 并行 CRC32 更新器，根据每拍有效输入字节推进 32-bit CRC 状态，供 MAC 收发检验/生成使用。
// Module Name: crc32_gen
// File Name: source/ethernet/crc32_gen.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 以太网 FCS 的 8-bit 并行 CRC32 更新器，根据每拍有效输入字节推进 32-bit CRC 状态，供 MAC 收发检验/生成使用。
// 复位或重新初始化恢复 CRC 种子；属于以太网协议差错检测逻辑。
//
// Dependencies: 无 IP 依赖；mac_rx 与 mac_tx 使用。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// Company/Engineer 仅标注本次用途说明整理；下方原作者和第三方版权继续有效。
// 历史协议栈/参考驱动，原始署名为空或来源未明确；保留历史说明，不把整理维护描述成原创。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/03/01 15:38:32
// Design Name: 
// Module Name: crc32_gen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module crc32_gen(
	input         clk,
    input         rstn,
	input         crc32_init,       //crc校验值初始化信号
	input         crc32_en,         //crc校验使能信号
	input         crc_read, 
    input  [7:0]  data,	 
	output [7:0]  crc_out 
);

    reg [31:0]   crc_temp;
    
    assign crc_out = crc_read ? ~{crc_temp[24], crc_temp[25], crc_temp[26], crc_temp[27],
    	                          crc_temp[28], crc_temp[29], crc_temp[30], crc_temp[31]} : 8'h00;			  
        
    
    always@(posedge clk) 
    begin        
       if(!rstn)
    	    crc_temp <= 32'hffffffff;		  
       else if(crc32_init)
    	    crc_temp <= 32'hffffffff;
       else if(crc32_en)
    	  begin
    		 crc_temp[0]  <= crc_temp[24] ^ crc_temp[30] ^ data[1] ^ data[7];
    		 crc_temp[1]  <= crc_temp[25] ^ crc_temp[31] ^ data[0] ^ data[6] ^ crc_temp[24] ^ crc_temp[30] ^ data[1] ^ data[7];
    		 crc_temp[2]  <= crc_temp[26] ^ data[5]      ^ crc_temp[25] ^ crc_temp[31]^data[0] ^ data[6]^crc_temp[24]^crc_temp[30]^data[1]^data[7];
    		 crc_temp[3]  <= crc_temp[27] ^ data[4]      ^ crc_temp[26] ^ data[5]^crc_temp[25] ^ crc_temp[31]^data[0]^data[6];
    		 crc_temp[4]  <= crc_temp[28] ^ data[3]      ^ crc_temp[27] ^ data[4]^crc_temp[26] ^ data[5]^crc_temp[24]^crc_temp[30]^data[1]^data[7];
    		 crc_temp[5]  <= crc_temp[29] ^ data[2]      ^ crc_temp[28] ^ data[3]^crc_temp[27] ^ data[4]^crc_temp[25]^crc_temp[31]^data[0]^data[6]^crc_temp[24]^crc_temp[30]^data[1]^data[7];
    		 crc_temp[6]  <= crc_temp[30] ^ data[1]      ^ crc_temp[29] ^ data[2]^crc_temp[28] ^ data[3]^crc_temp[26]^data[5]^crc_temp[25]^crc_temp[31]^data[0]^data[6];
    		 crc_temp[7]  <= crc_temp[31] ^ data[0]      ^ crc_temp[29] ^ data[2]^crc_temp[27] ^ data[4]^crc_temp[26]^data[5]^crc_temp[24]^data[7];
    		 crc_temp[8]  <= crc_temp[ 0] ^ crc_temp[28] ^ data[3] ^crc_temp[27] ^ data[4] ^ crc_temp[25]^data[6]^crc_temp[24]^data[7];
    		 crc_temp[9]  <= crc_temp[ 1] ^ crc_temp[29] ^ data[2] ^crc_temp[28] ^ data[3] ^ crc_temp[26]^data[5]^crc_temp[25]^data[6];
    		 crc_temp[10] <= crc_temp[ 2] ^ crc_temp[29] ^ data[2] ^crc_temp[27] ^ data[4] ^ crc_temp[26]^data[5]^crc_temp[24]^data[7];
    		 crc_temp[11] <= crc_temp[ 3] ^ crc_temp[28] ^ data[3] ^crc_temp[27] ^ data[4] ^ crc_temp[25]^data[6]^crc_temp[24]^data[7];
    		 crc_temp[12] <= crc_temp[ 4] ^ crc_temp[29] ^ data[2] ^crc_temp[28] ^ data[3] ^ crc_temp[26]^data[5]^crc_temp[25]^data[6]^crc_temp[24]^crc_temp[30]^data[1]^data[7];
    		 crc_temp[13] <= crc_temp[ 5] ^ crc_temp[30] ^ data[1] ^crc_temp[29] ^ data[2] ^ crc_temp[27]^data[4]^crc_temp[26]^data[5]^crc_temp[25]^crc_temp[31]^data[0]^data[6];
    		 crc_temp[14] <= crc_temp[ 6] ^ crc_temp[31] ^ data[0] ^crc_temp[30] ^ data[1] ^ crc_temp[28]^data[3]^crc_temp[27]^data[4]^crc_temp[26]^data[5];
    		 crc_temp[15] <= crc_temp[ 7] ^ crc_temp[31] ^ data[0] ^crc_temp[29] ^ data[2] ^ crc_temp[28]^data[3]^crc_temp[27]^data[4];
    		 crc_temp[16] <= crc_temp[ 8] ^ crc_temp[29] ^ data[2] ^crc_temp[28] ^ data[3] ^ crc_temp[24]^data[7];
    		 crc_temp[17] <= crc_temp[ 9] ^ crc_temp[30] ^ data[1] ^crc_temp[29] ^ data[2] ^ crc_temp[25]^data[6];
    		 crc_temp[18] <= crc_temp[10] ^ crc_temp[31] ^ data[0] ^crc_temp[30] ^ data[1] ^ crc_temp[26]^data[5];
    		 crc_temp[19] <= crc_temp[11] ^ crc_temp[31] ^ data[0] ^crc_temp[27] ^ data[4];
    		 crc_temp[20] <= crc_temp[12] ^ crc_temp[28] ^ data[3];                    
    		 crc_temp[21] <= crc_temp[13] ^ crc_temp[29] ^ data[2];                    
    		 crc_temp[22] <= crc_temp[14] ^ crc_temp[24] ^ data[7];                    
    		 crc_temp[23] <= crc_temp[15] ^ crc_temp[25] ^ data[6] ^crc_temp[24] ^ crc_temp[30] ^ data[1] ^ data[7];
    		 crc_temp[24] <= crc_temp[16] ^ crc_temp[26] ^ data[5] ^crc_temp[25] ^ crc_temp[31] ^ data[0] ^ data[6];
    		 crc_temp[25] <= crc_temp[17] ^ crc_temp[27] ^ data[4] ^crc_temp[26] ^ data[5];
    		 crc_temp[26] <= crc_temp[18] ^ crc_temp[28] ^ data[3] ^crc_temp[27] ^ data[4] ^ crc_temp[24] ^ crc_temp[30]^data[1]^data[7];
    		 crc_temp[27] <= crc_temp[19] ^ crc_temp[29] ^ data[2] ^crc_temp[28] ^ data[3] ^ crc_temp[25] ^ crc_temp[31]^data[0]^data[6];
    		 crc_temp[28] <= crc_temp[20] ^ crc_temp[30] ^ data[1] ^crc_temp[29] ^ data[2] ^ crc_temp[26] ^ data[5];
    		 crc_temp[29] <= crc_temp[21] ^ crc_temp[31] ^ data[0] ^crc_temp[30] ^ data[1] ^ crc_temp[27] ^ data[4];
    		 crc_temp[30] <= crc_temp[22] ^ crc_temp[31] ^ data[0] ^crc_temp[28] ^ data[3];
    		 crc_temp[31] <= crc_temp[23] ^ crc_temp[29] ^ data[2];
    	  end
    	else if(crc_read)
    	    crc_temp <= {crc_temp[23:0], 8'hff};
    end
		 
endmodule
