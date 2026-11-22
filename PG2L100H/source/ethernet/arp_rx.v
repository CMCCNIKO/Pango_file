//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/01 22:01:53
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 流式解析 Ethernet/IPv4 ARP 报文，检查固定 6-byte MAC/4-byte IP 格式、操作码与目标地址。
// Module Name: arp_rx
// File Name: source/ethernet/arp_rx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 流式解析 Ethernet/IPv4 ARP 报文，检查固定 6-byte MAC/4-byte IP 格式、操作码与目标地址。
// 只在该物理帧最终 MAC 判决通过后提交 ARP cache 或请求 ARP 应答，首次判决锁存后不被相邻帧覆盖。
//
// Dependencies: mac_rx/arp_mac_top 提供的 ARP 字节与 frame verdict。
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
// Module Name: arp_rx
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


module arp_rx(
    input                  clk,   
    input                  rstn,    
    
    input       [31:0]     local_ip_addr,
    input       [47:0]     local_mac_addr,
    //mac rx connect
    input       [ 7:0]     arp_rx_data,          //arp received data                        
    input                  i_frame_verdict_valid,
    input                  i_frame_verdict_error,
    input                  arp_rx_req,           //arp rx request from mac                  
    output reg             arp_rx_end,           //arp rx end                               
        
    // 协议解析反馈端口                                                                                    
    input                  arp_reply_ack,        //arp reply ack from arp reply module      
    output reg             arp_reply_req,        //arp reply request to arp reply module    
                                                                                            
    output reg [31:0]      arp_rec_sour_ip_addr, //arp received source ip address           
    output reg [47:0]      arp_rec_sour_mac_addr,//arp received mac address                 
    output reg             arp_found             //found destination mac address            
) ;
    
    localparam ARP_REQUEST_CODE = 16'h0001 ;
    localparam ARP_REPLY_CODE   = 16'h0002 ;
    
    reg [31:0]              arp_rec_dest_ip_addr ;
    reg [47:0]              arp_rec_dest_mac_addr ;
    reg [15:0]              arp_rec_op ;
    reg [7:0]               arp_rx_cnt ;
    
    parameter IDLE          = 4'b0001 ;
    parameter ARP_REC_DATA  = 4'b0010 ;
    parameter ARP_WAIT      = 4'b0100 ;
    parameter ARP_END       = 4'b1000 ;
    
    reg [3:0]    state  ;
    reg [3:0]    state_n ;

    reg flag_header_error;
    // Only Ethernet/IPv4 ARP with the fixed 6-byte/4-byte address format is valid.
    always @(posedge clk) begin
        if (!rstn || state == IDLE)
            flag_header_error <= 1'b0;
        else if (state == ARP_REC_DATA) begin
            case (arp_rx_cnt)
                8'd0: if (arp_rx_data != 8'h00) flag_header_error <= 1'b1;
                8'd1: if (arp_rx_data != 8'h01) flag_header_error <= 1'b1;
                8'd2: if (arp_rx_data != 8'h08) flag_header_error <= 1'b1;
                8'd3: if (arp_rx_data != 8'h00) flag_header_error <= 1'b1;
                8'd4: if (arp_rx_data != 8'd6) flag_header_error <= 1'b1;
                8'd5: if (arp_rx_data != 8'd4) flag_header_error <= 1'b1;
                default: ;
            endcase
        end
    end

    reg flag_frame_verdict;
    reg flag_frame_error;
    // ARP cache/reply commits require the accepted frame's complete MAC verdict.
    // Lock the first verdict so later neighboring frames cannot overwrite it.
    always @(posedge clk) begin
        if (!rstn || state == IDLE) begin
            flag_frame_verdict <= 1'b0;
            flag_frame_error <= 1'b0;
        end else if (!flag_frame_verdict && i_frame_verdict_valid) begin
            flag_frame_verdict <= 1'b1;
            flag_frame_error <= i_frame_verdict_error;
        end
    end
    
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
            IDLE         : 
            begin
                if (arp_rx_req)
                    state_n = ARP_REC_DATA ;
                else
                    state_n = IDLE ;
            end
            ARP_REC_DATA : 
            begin
                if (arp_rx_cnt == 45)
                    state_n = ARP_WAIT     ;
                else
                    state_n = ARP_REC_DATA ;
            end    
            ARP_WAIT     : 
            begin
                if (arp_rx_cnt == 99)
                    state_n = ARP_END     ;
                else
                    state_n = ARP_WAIT ;
            end              
            ARP_END      : state_n = IDLE ; 
            default      : state_n = IDLE ;
        endcase
    end

    always @(posedge clk)
    begin
        if (~rstn)
            arp_rx_end <= 1'b0 ;
        // One cycle before the first FCS reaches MAC count1.
        else if (state == ARP_REC_DATA && arp_rx_cnt == 43)
            arp_rx_end <= 1'b1 ;
        else
            arp_rx_end <= 1'b0 ;
    end
    
    //received arp request 
    always @(posedge clk)
    begin
        if (~rstn)
            arp_reply_req  <= 1'b0 ;
        else if (arp_rx_req)
            arp_reply_req  <= 1'b0 ;
        else if (arp_reply_ack)
            arp_reply_req  <= 1'b0 ;
        else if (state == ARP_END)
        begin 
            if (flag_frame_verdict && !flag_frame_error && !flag_header_error && arp_rec_op == ARP_REQUEST_CODE && (arp_rec_dest_ip_addr == local_ip_addr))          
                arp_reply_req <= 1'b1 ;
        end
    end
    
    //received arp reply
    always @(posedge clk)
    begin
      if (~rstn)
        arp_found  <= 1'b0 ;
      else if (state == ARP_END)
      begin 
          if (flag_frame_verdict && !flag_frame_error && !flag_header_error && arp_rec_op == ARP_REPLY_CODE && (arp_rec_dest_ip_addr == local_ip_addr) && (arp_rec_dest_mac_addr == local_mac_addr))          
              arp_found <= 1'b1 ;
          else
              arp_found  <= 1'b0 ;
      end
      else
          arp_found  <= 1'b0 ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rx_cnt  <= 8'd0 ;
        else if (state == ARP_REC_DATA || state == ARP_WAIT)
            arp_rx_cnt <= arp_rx_cnt + 1'b1 ;
        else
            arp_rx_cnt <= 8'd0 ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rec_op <= 16'd0 ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd6)
            arp_rec_op[15:8] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd7)
            arp_rec_op[7:0] <= arp_rx_data ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rec_sour_mac_addr <= 48'd0 ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd8)
            arp_rec_sour_mac_addr[47:40] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd9)
            arp_rec_sour_mac_addr[39:32] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd10)
            arp_rec_sour_mac_addr[31:24] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd11)
            arp_rec_sour_mac_addr[23:16] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd12)
            arp_rec_sour_mac_addr[15:8] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd13)
            arp_rec_sour_mac_addr[7:0] <= arp_rx_data ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rec_sour_ip_addr <= 32'd0 ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd14)
            arp_rec_sour_ip_addr[31:24] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd15)
            arp_rec_sour_ip_addr[23:16] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd16)
            arp_rec_sour_ip_addr[15:8] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd17)
            arp_rec_sour_ip_addr[7:0] <= arp_rx_data ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rec_dest_mac_addr <= 48'd0 ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd18)
            arp_rec_dest_mac_addr[47:40] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd19)
            arp_rec_dest_mac_addr[39:32] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd20)
            arp_rec_dest_mac_addr[31:24] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd21)
            arp_rec_dest_mac_addr[23:16] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd22)
            arp_rec_dest_mac_addr[15:8] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd23)
            arp_rec_dest_mac_addr[7:0] <= arp_rx_data ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            arp_rec_dest_ip_addr <= 32'd0 ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd24)
            arp_rec_dest_ip_addr[31:24] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd25)
            arp_rec_dest_ip_addr[23:16] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd26)
            arp_rec_dest_ip_addr[15:8] <= arp_rx_data ;
        else if (state == ARP_REC_DATA && arp_rx_cnt == 8'd27)
            arp_rec_dest_ip_addr[7:0] <= arp_rx_data ;
    end
    
endmodule
