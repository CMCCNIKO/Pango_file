//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: IPv4/ICMP 校验和共用函数头，checksum_adder 执行 32-bit 累加，checksum_out 将高低 16 位折叠相加。
// Module Name: (include) check_sum.vh
// File Name: source/ethernet/check_sum.vh
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// IPv4/ICMP 校验和共用函数头，checksum_adder 执行 32-bit 累加，checksum_out 将高低 16 位折叠相加。
// 在引用模块内部 include，不定义独立 module；调用方负责持续折叠、取反及协议字段顺序。
//
// Dependencies: ip_rx、ip_tx、icmp 等协议模块 include。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// Company/Engineer 仅标注本次用途说明整理；下方原作者和第三方版权继续有效。
// 历史协议栈/参考驱动，原始署名为空或来源未明确；保留历史说明，不把整理维护描述成原创。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

    //checksum function
    function    [31:0]  checksum_adder(
        input       [31:0]  dataina,
        input       [31:0]  datainb
    );
          
        begin
            checksum_adder = dataina + datainb;
        end
        
    endfunction
    
    function    [31:0]  checksum_out(
        input       [31:0]  dataina
    );
        begin
            checksum_out = dataina[15:0]+dataina[31:16];
        end
        
    endfunction