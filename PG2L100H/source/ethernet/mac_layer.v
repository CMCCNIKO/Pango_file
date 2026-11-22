//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/07 08:43:14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: Ethernet MAC 收发层组合封装，连接 GMII 接口、ARP/IP 发送选择与接收协议分流。
// Module Name: mac_layer
// File Name: source/ethernet/mac_layer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// Ethernet MAC 收发层组合封装，连接 GMII 接口、ARP/IP 发送选择与接收协议分流。
// 保留物理帧结束与最终 FCS/RX_ER 判决，使上层控制提交与相应物理帧一致。
//
// Dependencies: mac_rx、mac_tx、mac_tx_mode。
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
// Create Date: 2022/03/07 08:43:14
// Design Name: 
// Module Name: mac_layer
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


module mac_layer(
    input           clk,          
    input           rstn,         
                                  
    input           arp_tx_req,   
    input           arp_tx_ready,
    input  [7:0]    arp_tx_data,  
    input           arp_tx_end,   
    output          arp_tx_ack,   
                                  
    input           ip_tx_req,    
    input           ip_tx_ready,  
    input  [7:0]    ip_tx_data,   
    input           ip_tx_end,    
    output          ip_tx_ack,    
                        
    output          mac_data_req, 
    output          mac_send_end,  
     
    output  [7:0]   mac_tx_data,  
    output          mac_data_valid, 

    input           rx_en,               
    input           rx_error,
    input   [7:0]   mac_rx_datain,       
                                         
    input           checksum_err,        
                                         
    input           ip_rx_end,          
    output          ip_rx_req,          
    input           arp_rx_end,            
    output          arp_rx_req,          
                                         
    output  [7:0]   mac_rx_dataout,      
    output          o_frame_verdict_valid,
    output          o_frame_verdict_error,
    output          mac_rec_error ,      
                                         
    output  [47:0]  mac_rx_dest_mac_addr,
    output  [47:0]  mac_rx_sour_mac_addr 
);

    wire               mac_tx_req;   
    wire     [7:0]     mac_frame_data;
    wire               mac_tx_ready; 
    wire               mac_tx_end;   
           
    wire               mac_tx_ack;  
    mac_tx_mode mac_tx_mode(
        .clk                  (  clk              ),//input               clk ,
        .rstn                 (  rstn             ),//input               rst_n,
        .mac_send_end         (  mac_send_end     ),//input               mac_send_end,
                                                  
        .arp_tx_req           (  arp_tx_req       ),//input               arp_tx_req,
        .arp_tx_ready         (  arp_tx_ready     ),//input               arp_tx_ready ,
        .arp_tx_data          (  arp_tx_data      ),//input      [7:0]    arp_tx_data,
        .arp_tx_end           (  arp_tx_end       ),//input               arp_tx_end,
        .arp_tx_ack           (  arp_tx_ack       ),//output reg          arp_tx_ack,
                                                  
        .ip_tx_req            (  ip_tx_req        ),//input               ip_tx_req,
        .ip_tx_ready          (  ip_tx_ready      ),//input               ip_tx_ready,
        .ip_tx_data           (  ip_tx_data       ),//input      [7:0]    ip_tx_data,
        .ip_tx_end            (  ip_tx_end        ),//input               ip_tx_end,
        .ip_tx_ack            (  ip_tx_ack        ),//output reg          ip_tx_ack,
                                                  
        .mac_tx_ack           (  mac_tx_ack       ),//input               mac_tx_ack,
        .mac_tx_req           (  mac_tx_req       ),//output reg          mac_tx_req,         
        .mac_tx_ready         (  mac_tx_ready     ),//output reg          mac_tx_ready,
        .mac_tx_data          (  mac_frame_data   ),//output reg [7:0]    mac_tx_data,
        .mac_tx_end           (  mac_tx_end       ) //output reg          mac_tx_end
    );
    
    mac_tx mac_tx(
        .clk                  (  clk                   ),//input                  clk,
        .rstn                 (  rstn                  ),//input                  rstn,
                                                        
        .mac_tx_req           (  mac_tx_req            ),//input                  mac_tx_req,      //upper layer 给mac发送数据请求
        .mac_frame_data       (  mac_frame_data        ),//input      [7:0]       mac_frame_data,  //data from ip or arp
        .mac_tx_ready         (  mac_tx_ready          ),//input                  mac_tx_ready,    //ready from ip or arp
        .mac_tx_end           (  mac_tx_end            ),//input                  mac_tx_end,      //end from ip or arp
                                                        
        .mac_tx_ack           (  mac_tx_ack            ),//output reg             mac_tx_ack,      //mac send data ack for upper layer req
        .mac_data_req         (  mac_data_req          ),//output reg             mac_data_req,    //request data from arp or ip
                                                     
        .mac_tx_data          (  mac_tx_data           ),//output reg [7:0]       mac_tx_data,     //mac send data
        .mac_send_end         (  mac_send_end          ),//output reg             mac_send_end,    //mac frame data send over flag
        .mac_data_valid       (  mac_data_valid        ) //output reg             mac_data_valid   //mac send data valid flag
    ) ;
    
    mac_rx mac_rx(
        .clk                  (  clk                   ),//input                  clk,   
        .rstn                 (  rstn                  ),//input                  rstn,    
        
        .rx_en                (  rx_en                 ),
        .rx_error             (  rx_error              ),//input                  rx_en,
        .mac_rx_datain        (  mac_rx_datain         ),//input      [7:0]       mac_rx_datain,
       
        .checksum_err         (  checksum_err          ),//input                  checksum_err,        //checksum error from IP layer
                                             
        .ip_rx_end            (  ip_rx_end             ),//input                  ip_rx_end,           //ip receive end
        .arp_rx_end           (  arp_rx_end            ),//input                  arp_rx_end,          //arp receive end 
        .ip_rx_req            (  ip_rx_req             ),//output reg             ip_rx_req,           //ip rx request
        .arp_rx_req           (  arp_rx_req            ),//output reg             arp_rx_req,          //arp rx request
       
        .mac_rx_dataout       (  mac_rx_dataout        ),//output     [7:0]       mac_rx_dataout,
        .o_frame_verdict_valid (  o_frame_verdict_valid ),
        .o_frame_verdict_error (  o_frame_verdict_error ),
        .mac_rec_error        (  mac_rec_error         ),//output reg             mac_rec_error ,
        
        .mac_rx_dest_mac_addr (  mac_rx_dest_mac_addr  ),//output reg [47:0]      mac_rx_dest_mac_addr,
        .mac_rx_sour_mac_addr (  mac_rx_sour_mac_addr  ) //output reg [47:0]      mac_rx_sour_mac_addr
    );
endmodule
