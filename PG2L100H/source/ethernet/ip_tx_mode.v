//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/16 21:40:47
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: UDP 与 ICMP 到共享 IPv4 发送器的事务仲裁器，IDLE 下 UDP 请求优先，输出相应协议号及载荷长度。
// Module Name: ip_tx_mode
// File Name: source/ethernet/ip_tx_mode.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// UDP 与 ICMP 到共享 IPv4 发送器的事务仲裁器，IDLE 下 UDP 请求优先，输出相应协议号及载荷长度。
// 当前协议的数据、ready、ack 保持在同一 owner，直到整帧发送结束或超时。
//
// Dependencies: udp_tx、icmp、ip_tx。
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
// Create Date: 2022/03/16 21:40:47
// Design Name: 
// Module Name: ip_tx_mode
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


module ip_tx_mode(
    input                    clk ,
    input                    rstn,
    input                    mac_send_end,

    input                    udp_tx_req,
    input                    udp_tx_ready ,
    input      [7:0]         udp_tx_data,
    input      [15:0]        udp_send_data_length,
    output reg               udp_tx_ack,
    
    input                    icmp_tx_req,
    input                    icmp_tx_ready,
    input      [7:0]         icmp_tx_data,
    input      [15:0]        icmp_send_data_length,
    output reg               icmp_tx_ack,
    
    input                    ip_tx_ack,
    output reg               ip_tx_req,
    output reg               ip_tx_ready,
    output reg [7:0]         ip_tx_data,
    output reg [7:0]         ip_send_type,
    output reg [15:0]        ip_send_data_length
);
       
    localparam ip_udp_type  = 8'h11 ;
    localparam ip_icmp_type = 8'h01 ;
    
    reg [15:0]    timeout ;
    
    parameter IDLE      = 5'b00001 ;
    parameter UDP_WAIT  = 5'b00010 ;
    parameter UDP       = 5'b00100 ;
    parameter ICMP_WAIT = 5'b01000 ;
    parameter ICMP      = 5'b10000 ;
    
    
    reg [4:0]    state  ;
    reg [4:0]    next_state ;
    
    always @(posedge clk)
    begin
        if (~rstn)
            state  <=  IDLE  ;
        else
            state  <= next_state ;
    end
      
    always @(*)
    begin
        case(state)
          IDLE        :
          begin
              if (udp_tx_req)
                  next_state = UDP_WAIT ;
              else if (icmp_tx_req)
                  next_state = ICMP_WAIT  ;
              else
                  next_state = IDLE ;
          end
          UDP_WAIT    :
          begin
    		  if (ip_tx_ack)
    		      next_state = UDP ;
    		  else
    		      next_state = UDP_WAIT ;
          end		
          UDP         :
          begin
              if (mac_send_end)
                  next_state = IDLE ;
              else if (timeout == 16'hffff)
                  next_state = IDLE ;
              else
                  next_state = UDP ;
          end
    	  ICMP_WAIT    :
          begin
    		  if (ip_tx_ack)
    		      next_state = ICMP ;
    		  else
    		      next_state = ICMP_WAIT ;
          end	
          ICMP        :
          begin
              if (mac_send_end)
                  next_state = IDLE ;
              else if (timeout == 16'hffff)
                  next_state = IDLE ;
              else
                  next_state = ICMP ;
          end
          default     :
              next_state = IDLE ;
      endcase
    end

    always @(posedge clk)
    begin
        if (~rstn)
            timeout <= 16'd0 ;
        else if (state == UDP || state == ICMP)
            timeout <= timeout + 1'b1 ;
        else
            timeout <= 16'd0 ;
    end
      
    always @(posedge clk)
    begin
        if (~rstn)
            ip_send_data_length <= 16'd0 ;
        else if (state == ICMP_WAIT || state == ICMP)
            ip_send_data_length <= icmp_send_data_length ;
        else
            ip_send_data_length <= udp_send_data_length + 16'd20 ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            ip_tx_req  <= 1'b0  ;
        else if (state == UDP_WAIT || state == ICMP_WAIT)
    	    ip_tx_req  <= 1'b1  ;
        else 
    	    ip_tx_req  <= 1'b0  ;
    end
      
    always @(posedge clk)
    begin
        if (~rstn)
            udp_tx_ack  <= 1'b0  ;
        else if (state == UDP)
    	    udp_tx_ack  <= ip_tx_ack  ;
        else 
    	    udp_tx_ack  <= 1'b0  ;
    end

    always @(posedge clk)
    begin
        if (~rstn)
            icmp_tx_ack  <= 1'b0  ;
        else if (state == ICMP)
    	    icmp_tx_ack  <= ip_tx_ack  ;
        else 
    	    icmp_tx_ack  <= 1'b0  ;
    end
      
    always @(posedge clk)
    begin
        if (~rstn)
        begin
            ip_tx_ready       <= 1'b0 ;
            ip_tx_data        <= 8'h00 ;
            ip_send_type      <= ip_udp_type ;
        end
        else if (state == UDP)
        begin
            ip_tx_ready       <= udp_tx_ready ;
            ip_tx_data        <= udp_tx_data ;
            ip_send_type      <= ip_udp_type ;
            
        end
        else if (state == ICMP)
        begin
            ip_tx_ready       <= icmp_tx_ready ;
            ip_tx_data        <= icmp_tx_data ;
            ip_send_type      <= ip_icmp_type ;
            
        end
        else
        begin
            ip_tx_ready       <= 1'b0 ;
            ip_tx_data        <= 8'h00 ;
            ip_send_type      <= ip_udp_type ;
        end
    end
  
endmodule
