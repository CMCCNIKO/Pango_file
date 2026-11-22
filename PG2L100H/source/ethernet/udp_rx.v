//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2022/03/16 23:11:05
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: IPv4 上的 UDP 流式接收器，解析端口与长度，锁存视频/控制路由并输出同拍 start/data/valid/end。
// Module Name: udp_rx
// File Name: source/ethernet/udp_rx.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// IPv4 上的 UDP 流式接收器，解析端口与长度，锁存视频/控制路由并输出同拍 start/data/valid/end。
// 检查声明长度与实际帧边界；IPv4 允许的零 UDP checksum 直接接受，非零 checksum 包含伪首部、准确 UDP 字节及奇字节补零，错误通过终结事件上报。
//
// Dependencies: IPv4 解析数据/地址/校验状态与实际 MAC 帧结束；无新增缓存 IP。
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
// Create Date: 2022/03/16 23:11:05
// Design Name: 
// Module Name: udp_rx
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


module udp_rx #(
    parameter              LOCAL_PORT= 16'hF000,
    parameter CONTROL_PORT = 16'd5002
)(
    input                  clk,   
    input                  rstn,  
    
    input      [7:0]       udp_rx_data,
    input                  udp_rx_req,
    input                  i_frame_end,
    input [31:0] i_ip_source_addr,
    input [31:0] i_ip_dest_addr,
    input [15:0] i_ip_payload_length,
    
    input                  ip_checksum_error,
    input                  ip_addr_check_error,
    
    output reg [7:0]       udp_rec_rdata ,      //udp ram read data
    output reg [15:0]      udp_rec_data_length,     //udp data length
    output reg             udp_rec_data_valid,      //udp data valid
    output reg             udp_rec_data_start,      //first valid payload byte
    output reg             udp_rec_data_end,        //one cycle after last valid payload byte
    output reg udp_rec_is_control,
    output reg udp_rec_error
);

    reg  [15:0]             udp_rx_cnt ;
    reg  [15:0]             udp_data_length ;
    reg  [15:0]             udp_dest_port;
    reg flag_selected;
    reg flag_ip_error;
    reg [31:0] checksum_sum;
    reg [7:0] checksum_previous_byte;
    reg flag_checksum_present;
    wire [31:0] checksum_pseudo_sum;
    wire [16:0] checksum_fold;
    wire [15:0] checksum_result;
    wire flag_checksum_error;
    assign checksum_pseudo_sum = {16'd0,i_ip_source_addr[31:16]} +
        {16'd0,i_ip_source_addr[15:0]} + {16'd0,i_ip_dest_addr[31:16]} +
        {16'd0,i_ip_dest_addr[15:0]} + 32'd17 + {16'd0,udp_data_length};
    assign checksum_fold = {1'b0,checksum_sum[15:0]} + {1'b0,checksum_sum[31:16]};
    assign checksum_result = checksum_fold[15:0] + {15'd0,checksum_fold[16]};
    assign flag_checksum_error = flag_checksum_present && checksum_result != 16'hffff;

    localparam IDLE             =  8'b0000_0001  ;
    localparam REC_HEAD         =  8'b0000_0010  ;
    localparam REC_DATA         =  8'b0000_0100  ;
    localparam REC_ODD_DATA     =  8'b0000_1000  ;
    localparam VERIFY_CHECKSUM  =  8'b0001_0000  ;
    localparam REC_ERROR        =  8'b0010_0000  ;
    localparam REC_END_WAIT     =  8'b0100_0000  ;
    localparam REC_END          =  8'b1000_0000  ;
    
    reg [7:0]     state      ;
    reg [7:0]     state_n ;
    // IPv4 permits a zero UDP checksum. Nonzero checksums include the IPv4
    // pseudo header and exactly the declared UDP bytes, with an odd zero pad.
    // Accumulation runs beside the existing data path and adds no payload cycles.
    always @(posedge clk) begin
        if (!rstn || state == IDLE || i_frame_end) begin
            checksum_sum <= 32'd0;
            checksum_previous_byte <= 8'd0;
            flag_checksum_present <= 1'b0;
        end else if (state == REC_HEAD || state == REC_DATA) begin
            checksum_previous_byte <= udp_rx_data;
            if (udp_rx_cnt[0]) begin
                if (state == REC_HEAD && udp_rx_cnt == 16'd7) begin
                    checksum_sum <= checksum_sum + {16'd0,checksum_previous_byte,udp_rx_data} + checksum_pseudo_sum;
                    flag_checksum_present <= checksum_previous_byte != 8'd0 || udp_rx_data != 8'd0;
                end else begin
                    checksum_sum <= checksum_sum + {16'd0,checksum_previous_byte,udp_rx_data};
                end
            end else if (state == REC_DATA && udp_rx_cnt == udp_data_length - 16'd1) begin
                checksum_sum <= checksum_sum + {16'd0,udp_rx_data,8'd0};
            end
        end
    end
    
    always @(posedge clk)
    begin
      if (~rstn)
        state <= IDLE ;
      else 
        state <= state_n ;
    end
    
    always @(*)
    begin
        case(state)
         IDLE            :
         begin
             if (udp_rx_req == 1'b1)
                 state_n = REC_HEAD ;
             else
                 state_n = IDLE ;
         end
         REC_HEAD       :
         begin
             if (udp_rx_cnt == 16'd7)
             begin
                 if (((udp_dest_port == LOCAL_PORT) || (udp_dest_port == CONTROL_PORT)) &&
                     // Length includes the UDP header; REC_DATA needs at least
                     // one payload byte. Empty packets must not wrap the counter.
                     (udp_data_length > 16'd8) && (udp_data_length <= i_ip_payload_length) && !flag_ip_error &&
                     !ip_checksum_error && !ip_addr_check_error)
                     state_n = REC_DATA ;
                 else
                     state_n = REC_ERROR ;
             end
             else
                 state_n = REC_HEAD ;
         end
         REC_DATA       :
         begin
             if (udp_rx_cnt == udp_data_length - 1)
                 state_n = REC_END ;
             else
                 state_n = REC_DATA ;
         end
         REC_ERROR      : state_n = IDLE  ; 
         REC_END        : state_n = IDLE  ;
         default        : state_n = IDLE  ;
         endcase
        // MAC recovery alone cannot release a UDP parser whose declared
        // length extends beyond the received frame.
        if (i_frame_end) state_n = IDLE;
    end

    always @(posedge clk)
    begin
        if (~rstn)
            udp_dest_port <= 16'd0 ;
        else if (state == REC_HEAD && udp_rx_cnt > 16'd1 && udp_rx_cnt < 16'd4)
            udp_dest_port <= {udp_dest_port[7:0],udp_rx_data};
    end

    //udp data length 
    always @(posedge clk)
    begin
        if (~rstn)
            udp_data_length <= 16'd0 ;
        else if (state == REC_HEAD && udp_rx_cnt > 16'd3 && udp_rx_cnt < 16'd6)
            udp_data_length <= {udp_data_length[7:0],udp_rx_data};
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rec_data_length <= 16'd0 ;
        else if (state == REC_END)
            udp_rec_data_length <= udp_data_length - 16'd8;
        else if (i_frame_end && state == REC_DATA)
            udp_rec_data_length <= udp_rx_cnt > 16'd8 ? udp_rx_cnt - 16'd8 : 16'd0;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rx_cnt <= 16'd0 ;
        else if (state == REC_HEAD || state == REC_DATA)
            udp_rx_cnt <= udp_rx_cnt + 1'b1 ;
        else
            udp_rx_cnt <= 16'd0 ;
    end
    
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rec_rdata <= 8'd0 ;
        else if(udp_rx_cnt > 16'd7 && udp_rx_cnt < udp_data_length)
            udp_rec_rdata <= udp_rx_data ;
    end

    //**************************************************//
    //generate udp rx end
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rec_data_end <= 1'b0 ;
        else if (state == REC_END || (state == REC_ERROR && flag_selected) ||
                 (i_frame_end && state == REC_DATA))
            udp_rec_data_end <= 1'b1 ;
        else
            udp_rec_data_end <= 1'b0 ;
    end 
    
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rec_data_valid <= 1'b0 ;
        else if (state == REC_DATA && !i_frame_end)
            udp_rec_data_valid <= 1'b1 ;
        else
            udp_rec_data_valid <= 1'b0 ;
    end

    // udp_rx_cnt equals 8 for the first payload byte: the preceding eight
    // bytes are the UDP header.  Register the pulse so that it aligns with
    // udp_rec_data_valid and udp_rec_rdata at the module boundary.
    always @(posedge clk)
    begin
        if (~rstn)
            udp_rec_data_start <= 1'b0 ;
        else if (state == REC_DATA && udp_rx_cnt == 16'd8)
            udp_rec_data_start <= 1'b1 ;
        else
            udp_rec_data_start <= 1'b0 ;
    end

    // Latch route at the header boundary; errors on unknown ports stay silent.
    always @(posedge clk) begin
        if (!rstn) begin
            flag_ip_error <= 1'b0;
        end else if (state == IDLE) begin
            flag_ip_error <= 1'b0;
        end else if (state == REC_HEAD) begin
            flag_ip_error <= flag_ip_error | ip_checksum_error | ip_addr_check_error;
        end
    end
    always @(posedge clk) begin
        if (!rstn) begin
            flag_selected <= 1'b0;
        end else if (state == REC_HEAD && udp_rx_cnt == 16'd7) begin
            flag_selected <= (udp_dest_port == LOCAL_PORT) || (udp_dest_port == CONTROL_PORT);
        end
    end
    always @(posedge clk) begin
        if (!rstn) begin
            udp_rec_is_control <= 1'b0;
        end else if (state == REC_HEAD && udp_rx_cnt == 16'd7) begin
            udp_rec_is_control <= (udp_dest_port == CONTROL_PORT);
        end
    end
    always @(posedge clk) begin
        if (!rstn) begin
            udp_rec_error <= 1'b0;
        end else begin
            udp_rec_error <= ((state == REC_ERROR) && flag_selected) ||
                             ((state == REC_END) && flag_checksum_error) ||
                             (i_frame_end && state == REC_DATA);
        end
    end
endmodule
