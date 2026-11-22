//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/01 17:01:40
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: UDP 发送器，从应用数据握手生成源/目的端口、UDP 长度与头字段，再把应用字节送入 IPv4 发送链。
// Module Name: udp_tx
// File Name: source/ethernet/udp_tx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// UDP 发送器，从应用数据握手生成源/目的端口、UDP 长度与头字段，再把应用字节送入 IPv4 发送链。
// 使用官方 udp_shift_register 保存字节时序，按请求/ready/ack 与应用及 IP 层衔接一次报文。
//
// Dependencies: udp_shift_register 官方 IP、IPv4 发送接口。
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
// Create Date: 2022/03/01 17:01:40
// Design Name: 
// Module Name: udp_tx
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


module udp_tx #(
    parameter LOCAL_PORT_NUM = 16'hf000    //源端口号
) (
    input wire         udp_send_clk,      //时钟信号                                                                                                                                                                                                                                                                                                                                                                
    input wire         rstn,              //复位信号，低电平有效                                                                                                                                                                                                                                                                                                                                                    
                                                                                                                                                                                                                                                                                                                                                                                                                    
    //from software app                                                                                                                                                                                                                                                                                                                                                                                             
    input wire         app_data_in_valid, //本模块从外部所接收的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                        
    input wire [7:0]   app_data_in,       //本模块从外部所接收的数据输出                                                                                                                                                                                                                                                                                                                                            
    input wire [15:0]  app_data_length,   //本模块从外部所接收的当前数据包的长度（不含udp、ip、mac 首部），单位：字节                                                                                                                                                                                                                                                                                               
    input wire [15:0]  udp_dest_port,     //本模块从外部所接收的数据包的源端口号                                                                                                                                                                                                                                                                                                                                    
    input wire         app_data_request,  //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                                                                                                                                                    
    output wire        udp_send_ready,    //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    output wire        udp_send_ack,      //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
                                                                                                                                                                                                                                                                                                                                                                                                                    
    input wire         ip_send_ready,     //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    input wire         ip_send_ack,       //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    //to IP_send                                                                                                                                                                                                                                                                                                                                                                                                    
    output wire        udp_send_request,  //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
    output reg         udp_data_out_valid,//发送的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                                      
    output reg [7:0]   udp_data_out,      //发送的数据输出                                                                                                                                                                                                                                                                                                                                                          
    output reg [15:0]  udp_packet_length  //当前数据包的长度（不含udp、ip、mac 首部），单位：字节                                                                                                                                                                                                                                                                                                                   
);

    localparam  IDLE = 2'd0;
    localparam  WAIT_ACK = 2'd1;
    localparam  SEND_UDP_HEADER = 2'd2;
    localparam  SEND_UDP_PACKET = 2'd3;
    
    reg  [1:0]  state;
    reg  [1:0]  state_n;
    reg [3:0]   cnt; 
    wire [7:0]  shift_data_out;
    reg  [15:0] trans_data_cnt;

    localparam  CHECKSUM = 16'h0000;        //假如UDP包不使用校验和功能，校验和部分需全部置0
    
    assign udp_send_ready = state != IDLE;//ip_send_ready;
    assign udp_send_request = app_data_request;
    assign udp_send_ack = ip_send_ready;

    udp_shift_register udp_shift_register    (           //8个级联的移位寄存器组
          .din  (   app_data_in           ),// input [7:0]             
          .clk  (   udp_send_clk          ),// input                   
          .rst  (   ~rstn                 ),// input                   
          .dout (   shift_data_out        ) // output [7:0]            
//        .D    (  app_data_in                             ), // input [7 : 0] d
//        .CLK  (  udp_send_clk                            ), // input clk
//        .CE   (  app_data_in_valid | udp_data_out_valid  ), // input ce
//        .Q    (  shift_data_out                          ) // output [7 : 0] q
    );
  
    always @(posedge udp_send_clk) 
    begin
        if(~rstn)
            state <= IDLE;
        else
            state <= state_n;
    end
    
    always @(*) 
    begin
        case(state)
            IDLE:
            begin
                if(udp_send_request)// & ip_send_ready)
                    state_n = WAIT_ACK;
                else
                    state_n = IDLE;
            end
            WAIT_ACK:
            begin
                if(ip_send_ready)
                    state_n = SEND_UDP_HEADER;
                else
                    state_n = WAIT_ACK;
            end
            SEND_UDP_HEADER:
            begin
                if(cnt == 4'd7)
                    state_n = SEND_UDP_PACKET;
                else
                    state_n = SEND_UDP_HEADER;
            end
            SEND_UDP_PACKET:
            begin
                if (trans_data_cnt != (udp_packet_length - 16'd8)) 
                    state_n = SEND_UDP_PACKET;
                else 
                    state_n = IDLE;
            end
            default: state_n = IDLE;
        endcase  
    end
    always @(posedge udp_send_clk) 
    begin
        if(~rstn) 
            udp_packet_length <= 16'h0008;       //最小长度为包头
        else
            udp_packet_length <= app_data_length + 16'h0008;       //UDP报文长度
    end

    always @(posedge udp_send_clk) 
    begin
        if(~rstn) 
        begin
             cnt <= 4'h0;
             udp_data_out_valid <= 1'b0;
             udp_data_out <= 8'd0;
             trans_data_cnt <= 16'd0;
        end
        else 
        begin
            if(state == SEND_UDP_HEADER)
            begin
                case (cnt) 
                    4'd0: 
                    begin
                        if(app_data_in_valid) begin
                            udp_data_out <= LOCAL_PORT_NUM[15:8];
                            udp_data_out_valid <= 1'b1;
                            cnt <= cnt + 1'b1;
                        end
                        else
                            cnt <= 4'd0;
                    end
                    4'd1: 
                    begin
                        udp_data_out <= LOCAL_PORT_NUM[7:0];
                        cnt <= cnt + 1'b1;
                    end
                    4'd2: 
                    begin
                        udp_data_out <= udp_dest_port[15:8];
                        cnt <= cnt + 1'b1;
                    end
                    4'd3: 
                    begin
                        udp_data_out <= udp_dest_port[7:0];
                        cnt <= cnt + 1'b1;
                    end
                    4'd4: 
                    begin
                        udp_data_out <= udp_packet_length[15:8];
                        cnt <= cnt + 1'b1;
                    end
                    4'd5: 
                    begin
                        udp_data_out <= udp_packet_length[7:0];
                        cnt <= cnt + 1'b1;
                    end
                    4'd6: 
                    begin    
                        udp_data_out <= CHECKSUM[15:8];
                        cnt <= cnt + 1'b1;
                    end
                    4'd7:
                    begin
                        udp_data_out <= CHECKSUM[7:0];
                        cnt <= 4'h0;
                    end
                    default: cnt <= 4'h0;
                endcase
            end
            else if(state == SEND_UDP_PACKET)
            begin
                if (trans_data_cnt != (udp_packet_length - 16'd8)) 
                begin
                    udp_data_out_valid <= 1'b1;
                    udp_data_out <= shift_data_out;
                    trans_data_cnt <= trans_data_cnt + 1'b1;
                end
                else 
                begin
                    trans_data_cnt <= 16'd0;
                    udp_data_out_valid <= 1'b0;
                    udp_data_out <= 8'd0;
                    cnt <= 4'h0;
                end
            end
        end    
    end
    
endmodule
