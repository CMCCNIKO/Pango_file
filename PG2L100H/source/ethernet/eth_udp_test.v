//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/16 23:40:05
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 工程以太网协议栈与 ACK 发送调度封装，接收 GMII 并输出视频/控制 UDP 字节事件，按需要进行 ARP 查询及控制 ACK 发送。
// Module Name: eth_udp_test
// File Name: source/ethernet/eth_udp_test.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 工程以太网协议栈与 ACK 发送调度封装，接收 GMII 并输出视频/控制 UDP 字节事件，按需要进行 ARP 查询及控制 ACK 发送。
// 整个 ACK 事务受 watchdog 保护，超时共同复位协议 owner/握手但保留外部 ACK 队列；实际 RXC 复位保持四沿，恢复时给已输出的半包补错误 end。
// 当前 i_test_request 连接常数 0，周期测试报文未开启；ARP 重试计时与共享 MAC 完成脉冲独立。
//
// Dependencies: udp_ip_mac_top、udp_tx_arbiter_v2。
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
// Create Date: 2022/03/16 23:40:05
// Design Name: 
// Module Name: eth_udp_test
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


module eth_udp_test#(
    parameter       LOCAL_MAC = 48'h11_11_11_11_11_11,
    parameter       LOCAL_IP  = 32'hC0_A8_01_6E,//192.168.1.110
    parameter       LOCL_PORT = 16'h8080,
    parameter CONTROL_PORT = 16'd5002,

    parameter       DEST_IP   = 32'hC0_A8_01_69,//192.168.1.105
    parameter       C_PERIOD_CYCLES = 32'd125_000_000,
    parameter       C_TX_TIMEOUT_CYCLES = 32'd1048576,
    parameter       DEST_PORT = 16'h8080 
)(
    input                rgmii_clk,
    input                rstn,
    input                i_ack_valid,
    input [143:0]        i_ack_payload,
    input [15:0]         i_ack_length,
    output               o_ack_ready,
    input                gmii_rx_dv,
    input                i_gmii_rx_error,
    input  [7:0]         gmii_rxd,
    output reg           gmii_tx_en,
    output reg [7:0]     gmii_txd,
                 
    output               udp_rec_data_valid,         
    output [7:0]         udp_rec_rdata ,             
    output [15:0]        udp_rec_data_length,
    output               udp_rec_data_start,
    output               udp_rec_data_end,
    output udp_rec_is_control,
    output udp_rec_error
);
    
    wire [7:0] ram_wr_data;
    wire ram_wr_en;
    wire [15:0] udp_send_data_length;
    wire udp_tx_req;
    wire udp_ram_data_req;
    wire udp_send_done;
    wire mac_send_end;
    wire mac_not_exist;
    wire arp_found;
    wire tx_busy;
    wire need_network;
    reg [19:0] tx_stall_count;
    reg [3:0] stack_reset_hold;
    wire stack_rstn = rstn && stack_reset_hold == 4'd0;
    wire raw_rx_valid;
    wire [7:0] raw_rx_data;
    wire [15:0] raw_rx_length;
    wire raw_rx_start;
    wire raw_rx_end;
    wire raw_rx_control;
    wire raw_rx_error;
    reg rx_packet_open;
    reg rx_packet_control;
    wire recovery_abort = !stack_rstn && rx_packet_open;

    // One deadline covers the accepted ACK transaction.  Reset all protocol
    // ownership and handshake state together, preserving the external ACK queues.
    // 2^20 cycles is 8.39 ms at 125 MHz, far beyond a normal Ethernet frame.
    // Hold the existing protocol reset for four actual RXC edges after raw reset.
    // PHY RXC may stop during reset; synchronous legacy FSMs must still see reset.
    // Reuse the watchdog register/fanout; do not delay the board PHY reset pin.
    always @(posedge rgmii_clk or negedge rstn) begin
        if (!rstn) begin
            tx_stall_count <= 20'd0;
            stack_reset_hold <= 4'hf;
        end else begin
            if (!stack_rstn || !tx_busy) tx_stall_count <= 20'd0;
            else tx_stall_count <= tx_stall_count + 20'd1;
            if (stack_reset_hold != 0)
                stack_reset_hold <= {1'b0,stack_reset_hold[3:1]};
            else if (tx_busy && tx_stall_count == C_TX_TIMEOUT_CYCLES - 1)
                stack_reset_hold <= 4'hf;
        end
    end

    // A TX recovery may overlap RX.  Terminate any delivered partial packet
    // with an error instead of leaving the external command parser open.
    always @(posedge rgmii_clk) begin
        if (!rstn) begin
            rx_packet_open <= 1'b0;
            rx_packet_control <= 1'b0;
        end else if (!stack_rstn) begin
            rx_packet_open <= 1'b0;
        end else begin
            if (raw_rx_start) begin
                rx_packet_open <= 1'b1;
                rx_packet_control <= raw_rx_control;
            end
            if (raw_rx_end) rx_packet_open <= 1'b0;
        end
    end
    assign udp_rec_data_valid = stack_rstn && raw_rx_valid;
    assign udp_rec_rdata = raw_rx_data;
    assign udp_rec_data_length = raw_rx_length;
    assign udp_rec_data_start = stack_rstn && raw_rx_start;
    assign udp_rec_data_end = (stack_rstn && raw_rx_end) || recovery_abort;
    assign udp_rec_is_control = recovery_abort ? rx_packet_control : raw_rx_control;
    assign udp_rec_error = (stack_rstn && raw_rx_error) || recovery_abort;
    reg [31:0] arp_retry_count;
    reg [1:0] cache_settle;
    wire cache_ready = cache_settle[1] && !mac_not_exist;
    wire arp_request_req = cache_settle[1] && need_network &&
                           mac_not_exist && !tx_busy && arp_retry_count == 0;

    always @(posedge rgmii_clk) begin
        if (!stack_rstn) begin
            cache_settle <= 0;
        end else begin
            cache_settle <= {cache_settle[0], 1'b1};
        end
    end
    // ARP retries are independent of shared MAC completion pulses.
    always @(posedge rgmii_clk) begin
        if (!stack_rstn) arp_retry_count <= 0;
        else if (!mac_not_exist || !need_network) arp_retry_count <= 0;
        else if (arp_request_req) arp_retry_count <= 1;
        else if (arp_retry_count == C_PERIOD_CYCLES - 1) arp_retry_count <= 0;
        else if (arp_retry_count != 0) arp_retry_count <= arp_retry_count + 1'b1;
    end

    udp_tx_arbiter_v2 udp_tx_arbiter_v2_Inst_status (
        .i_clk(rgmii_clk), .i_rstn(stack_rstn),
        .i_ack_valid(i_ack_valid), .i_ack_payload(i_ack_payload), .i_ack_length(i_ack_length), .o_ack_ready(o_ack_ready),
        .i_test_request(1'b0), .i_network_ready(cache_ready),
        .o_need_network(need_network), .o_tx_busy(tx_busy), .o_tx_done(),
        .o_udp_request(udp_tx_req), .o_udp_length(udp_send_data_length),
        .o_udp_valid(ram_wr_en), .o_udp_data(ram_wr_data),
        .i_udp_data_request(udp_ram_data_req), .i_udp_done(udp_send_done)
    );

    reg          gmii_rx_dv_1d;
    reg          gmii_rx_error_1d;
    reg  [7:0]   gmii_rxd_1d;
    wire         gmii_tx_en_tmp;
    wire [7:0]   gmii_txd_tmp;
    
    always@(posedge rgmii_clk)
    begin
        if(stack_rstn == 1'b0)
        begin
            gmii_rx_dv_1d <= 1'b0 ;
            gmii_rx_error_1d <= 1'b0;
            gmii_rxd_1d   <= 8'd0 ;
        end
        else
        begin
            gmii_rx_dv_1d <= gmii_rx_dv ;
            gmii_rx_error_1d <= i_gmii_rx_error;
            gmii_rxd_1d   <= gmii_rxd ;
        end
    end
      
    always@(posedge rgmii_clk)
    begin
        if(stack_rstn == 1'b0)
        begin
            gmii_tx_en <= 1'b0 ;
            gmii_txd   <= 8'd0 ;
        end
        else
        begin
            gmii_tx_en <= gmii_tx_en_tmp ;
            gmii_txd   <= gmii_txd_tmp ;
        end
    end
    
    udp_ip_mac_top#(
        .LOCAL_MAC                (LOCAL_MAC               ),// 48'h11_11_11_11_11_11,
        .LOCAL_IP                 (LOCAL_IP                ),// 32'hC0_A8_01_6E,//192.168.1.110
        .LOCL_PORT                (LOCL_PORT               ),
        .CONTROL_PORT(CONTROL_PORT),
                                                           
        .DEST_IP                  (DEST_IP                 ),// 32'hC0_A8_01_69,//192.168.1.105
        .DEST_PORT                (DEST_PORT               ) // 16'h8080 
)udp_ip_mac_top(
        .rgmii_clk                (  rgmii_clk             ),//input           rgmii_clk,
        .rstn                     (  stack_rstn            ),//input           rstn,
  
        .app_data_in_valid        (  ram_wr_en             ),//input           app_data_in_valid,
        .app_data_in              (  ram_wr_data           ),//input   [7:0]   app_data_in,      
        .app_data_length          (  udp_send_data_length  ),//input   [15:0]  app_data_length,   
        .app_data_request         (  udp_tx_req            ),//input           app_data_request, 
                                                           
        .udp_send_ack             (  udp_ram_data_req      ),
        .udp_send_done            (  udp_send_done         ),//output          udp_send_ack,   
                                                           
        .arp_req                  (  arp_request_req       ),//input           arp_req,
        .arp_found                (  arp_found             ),//output          arp_found,
        .mac_not_exist            (  mac_not_exist         ),//output          mac_not_exist, 
        .mac_send_end             (  mac_send_end          ),//output          mac_send_end,
        
        .udp_rec_rdata            (  raw_rx_data           ),//output  [7:0]   udp_rec_rdata ,      //udp ram read data   
        .udp_rec_data_length      (  raw_rx_length         ),//output  [15:0]  udp_rec_data_length,     //udp data length     
        .udp_rec_data_valid       (  raw_rx_valid          ),//output          udp_rec_data_valid,       //udp data valid
        .udp_rec_data_start       (  raw_rx_start          ),//output          udp_rec_data_start,       //first payload byte
        .udp_rec_data_end         (  raw_rx_end            ),
        .udp_rec_is_control(raw_rx_control),
        .udp_rec_error(raw_rx_error),
        
        .mac_data_valid           (  gmii_tx_en_tmp        ),//output          mac_data_valid,
        .mac_tx_data              (  gmii_txd_tmp          ),//output  [7:0]   mac_tx_data,   
                                      
        .rx_en                    (  gmii_rx_dv_1d         ),
        .rx_error                 (  gmii_rx_error_1d      ),//input           rx_en,         
        .mac_rx_datain            (  gmii_rxd_1d           ) //input   [7:0]   mac_rx_datain
    );


endmodule
