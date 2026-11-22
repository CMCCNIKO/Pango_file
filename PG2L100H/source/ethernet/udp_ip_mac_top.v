//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/16 23:40:57
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: UDP/IPv4/ARP/MAC 全协议栈组合顶层，连接 GMII 收发、目标 MAC 查询和应用 UDP 字节流。
// Module Name: udp_ip_mac_top
// File Name: source/ethernet/udp_ip_mac_top.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// UDP/IPv4/ARP/MAC 全协议栈组合顶层，连接 GMII 收发、目标 MAC 查询和应用 UDP 字节流。
// 视频流保持原 start/data/end 时序；控制提交额外等待同一物理帧最终 MAC 判决，兼容 payload end 与 verdict 的任意先后或同拍。
//
// Dependencies: arp_mac_top、ip_layer、udp_layer。
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
// Create Date: 2022/03/16 23:40:57
// Design Name: 
// Module Name: udp_ip_mac_top
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


module udp_ip_mac_top #(
    parameter       LOCAL_MAC = 48'h3C_2B_1A_09_4D_5E,
    parameter       LOCAL_IP  = 32'hC0_A8_01_6E,
    parameter       LOCL_PORT = 16'hF0F0,
    parameter CONTROL_PORT = 16'd5002,
    parameter       DEST_MAC  = 48'h04_D9_F5_89_43_66,
    parameter       DEST_IP   = 32'hC0_A8_01_69,
    parameter       DEST_PORT = 16'hA0A0 
)(
    input           rgmii_clk,
    input           rstn,
    
    input           app_data_in_valid,
    input   [7:0]   app_data_in,      
    input   [15:0]  app_data_length,    
    input           app_data_request, 
                                      
    output          udp_send_ack,
    output          udp_send_done,   
    
    input           arp_req,
    output          arp_found,
    output          mac_not_exist,
    output          mac_send_end,

    output  [7:0]   udp_rec_rdata ,      //udp ram read data   
    output  [15:0]  udp_rec_data_length,     //udp data length     
    output          udp_rec_data_valid,      //udp data valid      
    output          udp_rec_data_start,      //first valid payload byte
    output          udp_rec_data_end,
    output udp_rec_is_control,
    output udp_rec_error,
    
    output          mac_data_valid,
    output  [7:0]   mac_tx_data,   
                                   
    input           rx_en,         
    input           rx_error,
    input   [7:0]   mac_rx_datain
);
    
    wire             udp_tx_req;         
    wire             udp_tx_ready;       
    wire     [7:0]   udp_tx_data;         
    wire     [15:0]  udp_send_data_length;
    wire             udp_tx_ack;         
                     
    wire             upper_data_req;  
    wire             udp_rx_req; 
    wire             ip_addr_check_error;
    wire [7:0]       net_protocol;      
    wire [31:0]      ip_rec_source_addr;
    wire [31:0]      ip_rec_dest_addr; 
    wire             ip_checksum_error;
    wire [15:0]      upper_layer_data_length;
 
    wire             mac_data_req;
    wire             mac_rec_err;
    wire mac_frame_verdict_valid;
    wire mac_frame_verdict_error;
    wire raw_udp_end;
    wire raw_udp_error;
    wire raw_udp_control;
    reg flag_frame_verdict;
    reg flag_frame_error;
    reg flag_control_open;
    reg flag_control_end_pending;
    reg flag_control_error;
    reg control_end_o;
    reg control_error_o;
    reg [2:0] rx_valid_pipeline;
    wire rx_frame_start = rx_en && !rx_valid_pipeline[0];
    wire flag_control_end = raw_udp_end && raw_udp_control;
    wire flag_verdict_available = flag_frame_verdict || mac_frame_verdict_valid;
    wire flag_verdict_error = flag_frame_error ||
                             (mac_frame_verdict_valid && mac_frame_verdict_error);
    // Video byte/start/valid/end timing remains the legacy streaming contract.
    // Only control commit waits for the same physical frame's MAC verdict.
    assign udp_rec_data_end = (raw_udp_end && !raw_udp_control) || control_end_o;
    assign udp_rec_is_control = raw_udp_control || control_end_o;
    assign udp_rec_error = raw_udp_error || (control_end_o && control_error_o);
    always @(posedge rgmii_clk) begin
        if (!rstn) begin
            flag_frame_verdict <= 1'b0;
            flag_frame_error <= 1'b0;
            flag_control_open <= 1'b0;
            flag_control_end_pending <= 1'b0;
            flag_control_error <= 1'b0;
            control_end_o <= 1'b0;
            control_error_o <= 1'b0;
        end else begin
            control_end_o <= 1'b0;
            control_error_o <= 1'b0;
            if (rx_frame_start) begin
                // A new physical frame cannot inherit an earlier verdict.
                // Abnormal unfinished control reception is terminated once.
                flag_frame_verdict <= 1'b0;
                flag_frame_error <= 1'b0;
                flag_control_open <= 1'b0;
                flag_control_end_pending <= 1'b0;
                flag_control_error <= 1'b0;
                if (flag_control_open || flag_control_end_pending) begin
                    control_end_o <= 1'b1;
                    control_error_o <= 1'b1;
                end
            end else begin
                if (mac_frame_verdict_valid) begin
                    flag_frame_verdict <= 1'b1;
                    flag_frame_error <= mac_frame_verdict_error;
                end
                if (udp_rec_data_start && raw_udp_control) begin
                    flag_control_open <= 1'b1;
                    flag_control_error <= raw_udp_error;
                end else if (flag_control_open && raw_udp_error) begin
                    flag_control_error <= 1'b1;
                end
                if (flag_control_end) begin
                    flag_control_end_pending <= 1'b1;
                    flag_control_error <= flag_control_error || raw_udp_error;
                end
                // Both orders and a same-cycle end/verdict are supported.
                if ((flag_control_end_pending || flag_control_end) && flag_verdict_available) begin
                    control_end_o <= 1'b1;
                    control_error_o <= flag_control_error || raw_udp_error || flag_verdict_error;
                    flag_control_end_pending <= 1'b0;
                    flag_control_open <= 1'b0;
                    flag_control_error <= 1'b0;
                end
            end
        end
    end
    wire  [7:0]      mac_rx_dataout;   

    // RX_DV follows the same three registers as mac_rx_dataout.  The pulse
    // marks the final physical byte, including FCS, without adding payload latency.
    wire rx_frame_end = rx_valid_pipeline[2] && !rx_valid_pipeline[1];
    always @(posedge rgmii_clk) begin
        if (!rstn) rx_valid_pipeline <= 3'd0;
        else rx_valid_pipeline <= {rx_valid_pipeline[1:0],rx_en};
    end

    // UDP ownership is granted before the shared IP payload request.
    wire [47:0] arp_dest_mac_addr;
    reg udp_owned;
    reg udp_started;
    reg [47:0] udp_dest_mac_locked;
    wire udp_data_request = upper_data_req && udp_owned;
    assign udp_send_done = mac_send_end && udp_owned && udp_started;
    always @(posedge rgmii_clk) begin
        if (!rstn) begin
            udp_owned <= 1'b0;
            udp_started <= 1'b0;
            udp_dest_mac_locked <= 48'hff_ff_ff_ff_ff_ff;
        end else if (mac_send_end) begin
            udp_owned <= 1'b0;
            udp_started <= 1'b0;
        end else begin
            if (udp_tx_ack) begin
                udp_owned <= 1'b1;
                if (!udp_owned) udp_dest_mac_locked <= arp_dest_mac_addr;
            end
            if (udp_data_request) udp_started <= 1'b1;
        end
    end

    udp_layer #(
        .LOCAL_PORT_NUM          (  LOCL_PORT                ),
        .CONTROL_PORT(CONTROL_PORT)
    )udp_layer (                                             
        .udp_send_clk            (  rgmii_clk                ),//input              udp_send_clk,               //时钟信号                                                                                                                                                                                                                                                                                                                                                                
        .rstn                    (  rstn                     ),//input              rstn,                       //复位信号，低电平有效                                                                                                                                                                                                                                                                                                                                                    
                                                                                                                                                                                                                                                                                                                                                                                                                            
        //from software app                                                                                                                                                                                                                                                                                                                                                                                                   
        .app_data_in_valid       (  app_data_in_valid        ),//input              app_data_in_valid,          //本模块从外部所接收的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                        
        .app_data_in             (  app_data_in              ),//input      [7:0]   app_data_in,                //本模块从外部所接收的数据输出                                                                                                                                                                                                                                                                                                                                            
        .app_data_length         (  app_data_length          ),//input      [15:0]  app_data_length,            //本模块从外部所接收的当前数据包的长度（不含udp、ip、mac 首部），单位：字节                                                                                                                                                                                                                                                                                               
        .udp_dest_port           (  DEST_PORT                ),//input      [15:0]  udp_dest_port,              //本模块从外部所接收的数据包的源端口号                                                                                                                                                                                                                                                                                                                                    
        .app_data_request        (  app_data_request         ),//input              app_data_request,           //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                                                                                                                                                             
        .udp_send_ack            (  udp_send_ack             ),//output             udp_send_ack,               //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
                                                                                                                                                                                                                                                                                                                                                                                                   
        .udp_send_ready          (  udp_tx_ready             ),//output          udp_send_ready,             //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                                                                                    
        .ip_send_ready           (  udp_data_request           ),//input           ip_send_ready,              //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
        .ip_send_ack             (  udp_tx_ack               ),//input           ip_send_ack,                //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
        //to IP_send                                                                                                                                                                                                                                                                                                                                                                                                          
        .udp_send_request        (  udp_tx_req               ),//output          udp_send_request,           //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
        .udp_data_out_valid      (                           ),//output          udp_data_out_valid,         //发送的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                                      
        .udp_data_out            (  udp_tx_data              ),//output  [7:0]   udp_data_out,               //发送的数据输出                                                                                                                                                                                                                                                                                                                                                          
        .udp_packet_length       (  udp_send_data_length     ),//output  [15:0]  udp_packet_length,          //当前数据包的长度（不含udp、ip、mac 首部），单位：字节  
                                                             
        .udp_rx_data             (  mac_rx_dataout           ),//input      [7:0]   udp_rx_data,
        .udp_rx_req              (  udp_rx_req               ),//input              udp_rx_req,
        .i_frame_end             (  rx_frame_end             ),
        .i_ip_source_addr        (  ip_rec_source_addr       ),
        .i_ip_dest_addr          (  ip_rec_dest_addr         ),
        .i_ip_payload_length     (  upper_layer_data_length  ),
                                                             
        .ip_checksum_error       (  ip_checksum_error        ),//input              ip_checksum_error,
        .ip_addr_check_error     (  ip_addr_check_error      ),//input              ip_addr_check_error,
        
        .udp_rec_rdata           (  udp_rec_rdata            ),//output     [7:0]   udp_rec_rdata ,      //udp ram read data
        .udp_rec_data_length     (  udp_rec_data_length      ),//output     [15:0]  udp_rec_data_length,     //udp data length
        .udp_rec_data_valid      (  udp_rec_data_valid       ),//output             udp_rec_data_valid       //udp data valid
        .udp_rec_data_start      (  udp_rec_data_start       ),//output             udp_rec_data_start       //first payload byte
        .udp_rec_data_end        (  raw_udp_end              ),
        .udp_rec_is_control(raw_udp_control),
        .udp_rec_error(raw_udp_error)
    );

    wire          ip_tx_ready;
    wire [7:0]    ip_tx_data ;
    wire          ip_tx_end  ;
    wire          ip_tx_ack  ; 
    wire          mac_tx_req;
    wire          ip_rx_end  ;
    wire          ip_rx_req  ;       
    wire  [47:0]  mac_rx_dest_mac_addr;
    wire  [47:0]  mac_rx_sour_mac_addr;

    ip_layer#(
        .LOCAL_IP                (  LOCAL_IP                 ),//parameter    LOCAL_IP  = 32'hC0_A8_00_02,
        .LOCAL_MAC               (  LOCAL_MAC                ) //parameter    LOCAL_MAC = 48'h3C_2B_1A_09_4D_5E
    ) ip_layer (                                              
        .clk                     (  rgmii_clk                ),//input              clk, 
        .rstn                    (  rstn                     ),//input              rstn,
        //MAC TX                                             
        .mac_tx_ack              (  ip_tx_ack                ),//input              mac_tx_ack,  
        .mac_send_end            (  mac_send_end             ),//input              mac_send_end,
        .mac_data_req            (  mac_data_req             ),//input              mac_data_req,
        .ip_tx_req               (  ip_tx_req               ),//output             mac_tx_req,                                                     
        .ip_tx_ready             (  ip_tx_ready              ),//output             ip_tx_ready, 
        .ip_tx_data              (  ip_tx_data               ),//output [7:0]       ip_tx_data,  
        .ip_tx_end               (  ip_tx_end                ),//output             ip_tx_end,    
        //MAC RX                                             
        .ip_rx_data              (  mac_rx_dataout           ),//input  [7:0]       ip_rx_data,          
        .ip_rx_req               (  ip_rx_req                ),//input              ip_rx_req,           
        .i_frame_end             (  rx_frame_end             ),
        .mac_rx_dest_mac_addr    (  mac_rx_dest_mac_addr     ),//input  [47:0]      mac_rx_dest_mac_addr,
        .ip_rx_end               (  ip_rx_end                ),//output             ip_rx_end,  
                                                             
        .i_frame_verdict_valid   (  mac_frame_verdict_valid   ),
        .i_frame_verdict_error   (  mac_frame_verdict_error   ),
        .mac_rec_err             (  mac_rec_err              ),//input              mac_rec_err,  
        //UPPER LAYER TX                                     
        .dest_mac_addr           (  udp_owned ? udp_dest_mac_locked : arp_dest_mac_addr ),// Fixed UDP transaction MAC snapshot; ICMP unchanged.
        .dest_ip_addr            (  DEST_IP                  ),//input  [31:0]      dest_ip_addr,
                                                             
        .udp_tx_req              (  udp_tx_req               ),//input              udp_tx_req,          
        .udp_tx_ready            (  udp_tx_ready             ),//input              udp_tx_ready ,       
        .udp_tx_data             (  udp_tx_data              ),//input      [7:0]   udp_tx_data,         
        .udp_send_data_length    (  udp_send_data_length     ),//input      [15:0]  udp_send_data_length,
        .udp_tx_ack              (  udp_tx_ack               ),//output             udp_tx_ack,     
                                                             
        .upper_data_req          (  upper_data_req           ),//output             upper_data_req,    
                                                     
        //UPPER LAYER RX                                     
        .udp_rx_req              (  udp_rx_req               ),//output             udp_rx_req,
        .ip_addr_check_error     (  ip_addr_check_error      ),//output             ip_addr_check_error,                                                       
        .net_protocol            (  net_protocol             ),//output [7:0]       net_protocol,           
        .ip_rec_source_addr      (  ip_rec_source_addr       ),//output [31:0]      ip_rec_source_addr,     
        .ip_rec_dest_addr        (  ip_rec_dest_addr         ),//output [31:0]      ip_rec_dest_addr,   
        .upper_layer_data_length (  upper_layer_data_length  ),//output [15:0]      upper_layer_data_length,      
        .ip_checksum_error       (  ip_checksum_error        ) //output             ip_checksum_error       
    );                                                       
                                                             
    arp_mac_top#(                                            
        .LOCAL_MAC_ADDR          (  LOCAL_MAC                ),//parameter LOCAL_MAC_ADDR = 48'h3C_2B_1A_09_4D_5E,
        .LOCAL_IP_ADDR           (  LOCAL_IP                 ) //parameter LOCAL_IP_ADDR  = 24'hC0_A8_00_02,
    ) arp_mac_top_U1 (                                       
        .clk                     (  rgmii_clk                ),//input           clk,
        .rstn                    (  rstn                     ),//input           rstn,
                                                             
        .checksum_err            (  ip_checksum_error        ),//input           checksum_err,
                                                             
        .ip_tx_req               (  ip_tx_req                ),//input           ip_tx_req     ,
        .ip_tx_ready             (  ip_tx_ready              ),//input           ip_tx_ready   ,
        .ip_tx_data              (  ip_tx_data               ),//input  [7:0]    ip_tx_data    ,
        .ip_tx_end               (  ip_tx_end                ),//input           ip_tx_end     ,
        .ip_tx_ack               (  ip_tx_ack                ),//output          ip_tx_ack     ,
        .mac_data_req            (  mac_data_req             ),//output          mac_data_req  ,
        .mac_send_end            (  mac_send_end             ),//output          mac_send_end  ,
                                                                                            
        .ip_rx_end               (  ip_rx_end                ),//input           ip_rx_end     ,
        .ip_rx_req               (  ip_rx_req                ),//output          ip_rx_req     ,
        .mac_rx_dataout          (  mac_rx_dataout           ),//output  [7:0]   mac_rx_dataout,                      
        .mac_rx_dest_mac_addr    (  mac_rx_dest_mac_addr     ),//output  [47:0]  mac_rx_dest_mac_addr,                
        .mac_rx_sour_mac_addr    (  mac_rx_sour_mac_addr     ),//output  [47:0]  mac_rx_sour_mac_addr,                
                                                             
        .arp_request_req         (  arp_req                  ),//input           arp_request_req, 
        .arp_found               (  arp_found                ),//output          arp_found,
        .mac_not_exist           (  mac_not_exist            ),//output          mac_not_exist,
        .arp_dest_mac_addr       (  arp_dest_mac_addr        ),//output [47:0]   arp_dest_mac_addr,
        
        .dest_ip_addr            (  DEST_IP                  ),//input           dest_ip_addr,
                                                             
        .o_frame_verdict_valid   (  mac_frame_verdict_valid   ),
        .o_frame_verdict_error   (  mac_frame_verdict_error   ),
        .mac_rec_error           (  mac_rec_err              ),//output          mac_rec_error
                                                             
        .mac_data_valid          (  mac_data_valid           ),//output          mac_data_valid, 
        .mac_tx_data             (  mac_tx_data              ),//output  [7:0]   mac_tx_data,           
                                                                                                         
        .rx_en                   (  rx_en                    ),
        .rx_error                (  rx_error                 ),//input           rx_en,                  
        .mac_rx_datain           (  mac_rx_datain            ) //input   [7:0]   mac_rx_datain,          
    );
endmodule
