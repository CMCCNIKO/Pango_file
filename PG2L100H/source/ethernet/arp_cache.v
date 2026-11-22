//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/01 22:01:53
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 单条 ARP 地址缓存，在收到 arp_found 时登记源 IPv4/MAC 对，按目标 IPv4 返回对应目的 MAC。
// Module Name: arp_cache
// File Name: source/ethernet/arp_cache.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 单条 ARP 地址缓存，在收到 arp_found 时登记源 IPv4/MAC 对，按目标 IPv4 返回对应目的 MAC。
// 未匹配或缓存 MAC 为广播地址时拉高 mac_not_exist，并输出广播 MAC 供 ARP 探测使用。
//
// Dependencies: arp_rx 的有效地址发现事件；无 RAM IP。
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
// Create Date: 2022/03/01 22:01:53
// Design Name: 
// Module Name: arp_cache
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

module arp_cache(
    input                clk ,
    input                rst_n ,
    
    input                arp_found,
    input      [31:0]    arp_rec_source_ip_addr,
    input      [47:0]    arp_rec_source_mac_addr,
               
    input      [31:0]    dest_ip_addr,
    output reg [47:0]    dest_mac_addr,
    
    output reg           mac_not_exist
) ;
           
    reg [79:0]  arp_cache ;
    
    //init arp cache
    always @(posedge clk or negedge rst_n)
    begin
        if (~rst_n)
            arp_cache  <= 80'h00_00_00_00_ff_ff_ff_ff_ff_ff ;
        else if (arp_found)
            arp_cache  <= {arp_rec_source_ip_addr, arp_rec_source_mac_addr} ;
        else
            arp_cache  <= arp_cache ;
    end
      
    always @(posedge clk or negedge rst_n)
    begin
        if (~rst_n)
            dest_mac_addr  <= 48'hff_ff_ff_ff_ff_ff ;
        else if (dest_ip_addr == arp_cache[79:48])
            dest_mac_addr  <= arp_cache[47:0] ;
        else
            dest_mac_addr  <= 48'hff_ff_ff_ff_ff_ff ;
    end
      
    always @(posedge clk or negedge rst_n)
    begin
        if (~rst_n)
            mac_not_exist  <= 1'b0 ;
        else if (dest_ip_addr != arp_cache[79:48])
            mac_not_exist  <= 1'b1 ;
        else if (dest_ip_addr == arp_cache[79:48] && arp_cache[47:0] == 48'hff_ff_ff_ff_ff_ff)
            mac_not_exist  <= 1'b1 ;
        else
            mac_not_exist  <= 1'b0 ;
    end
  
endmodule
