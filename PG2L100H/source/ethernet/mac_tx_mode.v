//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/07 11:21:37
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: ARP 与 IP 到单个 MAC 发送器的事务仲裁/数据复用层，IDLE 下 ARP 请求优先。
// Module Name: mac_tx_mode
// File Name: source/ethernet/mac_tx_mode.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// ARP 与 IP 到单个 MAC 发送器的事务仲裁/数据复用层，IDLE 下 ARP 请求优先。
// 锁定被选中的协议，转发 request、ready、data、end 和 ack，直到 mac_send_end 或超时才释放。
//
// Dependencies: arp_tx、ip_tx、mac_tx。
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
// Create Date: 2022/03/07 11:21:37
// Design Name: 
// Module Name: mac_tx_mode
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


module mac_tx_mode(
    input               clk ,
    input               rstn,
    input               mac_send_end,
    
    input               arp_tx_req,
    input               arp_tx_ready ,
    input      [7:0]    arp_tx_data,
    input               arp_tx_end,
    output reg          arp_tx_ack,
    
    input               ip_tx_req,
    input               ip_tx_ready,
    input      [7:0]    ip_tx_data,
    input               ip_tx_end,
    output reg          ip_tx_ack,
    
    input               mac_tx_ack,
    output reg          mac_tx_req,         
    output reg          mac_tx_ready,
    output reg [7:0]    mac_tx_data,
    output reg          mac_tx_end
);
     
    reg [15:0]    timeout ;
    
    parameter IDLE       = 5'b00001 ;
    parameter ARP_WAIT   = 5'b00010 ;
    parameter ARP        = 5'b00100 ;
    parameter IP_WAIT    = 5'b01000 ;
    parameter IP         = 5'b10000 ;
    
    
    reg [4:0]    state  ;
    reg [4:0]    state_n ;
    
    always @(posedge clk)
    begin
        if (~rstn)
          state  <=  IDLE  ;
        else
          state  <= state_n ;
    end
      
    always @(*)
    begin
        case(state)
            IDLE        :
            begin
                if (arp_tx_req)
                    state_n = ARP_WAIT ;
                else if (ip_tx_req)
                    state_n = IP_WAIT  ;
                else
                    state_n = IDLE ;
            end
            ARP_WAIT  :
            begin
                if (mac_tx_ack)
                    state_n = ARP ;
                else
                    state_n = ARP_WAIT ;
            end        
            ARP         :
            begin
                if (mac_send_end)
                    state_n = IDLE ;
                else if (timeout == 16'hffff)
                    state_n = IDLE ;
                else
                    state_n = ARP ;
            end
            IP_WAIT  :
            begin
                if (mac_tx_ack)
                    state_n = IP ;
                else
                    state_n = IP_WAIT ;
            end    
            IP          :
            begin
                if (mac_send_end)
                    state_n = IDLE ;
                else if (timeout == 16'hffff)
                    state_n = IDLE ;
                else
                    state_n = IP ;
            end
            default     :
                state_n = IDLE ;
      endcase
    end

    always @(posedge clk)
    begin
        if (~rstn)
            timeout <= 16'd0 ;
        else if (state == ARP || state == IP)
            timeout <= timeout + 1'b1 ;
        else
            timeout <= 16'd0 ;
    end
      
    always @(posedge clk)
    begin
        if (~rstn)
            arp_tx_ack <= 1'b0 ;
        else if (state == ARP)
            arp_tx_ack <= 1'b1 ;
        else
            arp_tx_ack <= 1'b0 ;
    end  
     
    always @(posedge clk)
    begin
        if (~rstn)
            ip_tx_ack <= 1'b0 ;
        else if (state == IP)
            ip_tx_ack <= 1'b1 ;
        else
            ip_tx_ack <= 1'b0 ;
    end 
    
    always @(posedge clk)
    begin
        if (~rstn)
            mac_tx_req <= 1'b0 ;
        else if (state == ARP_WAIT || state == IP_WAIT)
            mac_tx_req <= 1'b1 ;
        else
            mac_tx_req <= 1'b0 ;
    end   
    
    always @(posedge clk)
    begin
        if (~rstn)
        begin
            mac_tx_ready      <= 1'b0  ;
            mac_tx_data       <= 8'h00 ;
            mac_tx_end        <= 1'b0  ;
        end
        else if (state == ARP)
        begin
            mac_tx_ready      <= arp_tx_ready ;
            mac_tx_data       <= arp_tx_data ;
            mac_tx_end        <= arp_tx_end ;
        end
        else if (state == IP)
        begin
            mac_tx_ready      <= ip_tx_ready ;
            mac_tx_data       <= ip_tx_data ;
            mac_tx_end        <= ip_tx_end ;
        end
        else
        begin
            mac_tx_ready      <= 1'b0  ;
            mac_tx_data       <= 8'h00 ;
            mac_tx_end        <= 1'b0  ;
        end
    end
endmodule
