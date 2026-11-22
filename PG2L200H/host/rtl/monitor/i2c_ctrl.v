//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2019/03/07 17:22:20
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: ADV7611 I2C 初始化封装：连接配置表与 I2C 总线驱动，按默认 50 MHz 管理时钟、250 kHz SCL 顺序写入接收芯片配置并导出初始化完成。
// Module Name: i2c_ctrl
// File Name: host/rtl/monitor/i2c_ctrl.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// ADV7611 I2C 初始化封装：连接配置表与 I2C 总线驱动，按默认 50 MHz 管理时钟、250 kHz SCL 顺序写入接收芯片配置并导出初始化完成。
//
// Dependencies: i2c_dri、i2c_adv7611_cfg；adv_clk、低有效 adv_rst；SDA 为双向端口。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原模板日期 2019/03/07 17:22:20，Company/Engineer 未注明；保留原历史模板，不推定原作者。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/03/07 17:22:20
// Design Name: 
// Module Name: IIC_ADV7611_Config
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

module i2c_ctrl
(
	input     adv_clk  ,
	input     adv_rst  ,//低电平复位
	output    adv_sclk ,
	inout     adv_sdat ,
	output    init_done/*synthesis syn_keep=1*/ 

);
parameter  CLK_FREQ   = 26'd50_000_000 ;  //i2c_dri模块的驱动时钟频率 50.0MHz
parameter  I2C_FREQ   = 18'd250000     ;  //I2C的SCL时钟频率,不超过400KHz

//wire define  
wire  [9:0]    i2c_config_index;
wire  [23:0]   i2c_config_data ;
wire  [9:0]    i2c_config_size ;
wire           i2c_config_done ;
wire           i2c_RW_flag     ;
wire  [7:0]    i2c_rdata       ;  //i2c register data
wire           i2c_exec        ; 
wire  [23:0]   i2c_data        ;   
wire           i2c_done        ;  //I2C寄存器配置完成信号

//*****************************************************
//**                    main code
//*****************************************************
	
i2c_dri
#(             
	.CLK_FREQ               (CLK_FREQ  ),           
	.I2C_FREQ               (I2C_FREQ  )            
	)          
u_i2c_dri
(
    //global clock
    .clk                    (adv_clk        ),   
    .rst_n                  (adv_rst        ),   
    //i2c interface         
    .i2c_exec               (i2c_exec       ),   
    .bit_ctrl               (1'b0           ),   
    .i2c_rh_wl              (1'b0           ),          
    .slaver_addr            (i2c_data[23:16]),
    .i2c_addr               (i2c_data[15:8] ),
    .i2c_data_w             (i2c_data[7:0]  ), 
    .i2c_data_r             (),   
    .i2c_done               (i2c_done       ), 
    .i2c_ack                (),    
    .scl                    (adv_sclk       ),   
    .sda                    (adv_sdat       ),    
    .dri_clk                (i2c_dri_clk    )  
);                    

i2c_adv7611_cfg    u_i2c_adv7611_cfg
(
    .clk                    (i2c_dri_clk),
    .rst_n                  (adv_rst    ),
    .i2c_done               (i2c_done   ),
    .i2c_exec               (i2c_exec   ),
    .i2c_data               (i2c_data   ),
    .init_done              (init_done  )
);   
endmodule
