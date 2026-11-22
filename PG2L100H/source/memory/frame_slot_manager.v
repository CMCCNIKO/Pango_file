//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-13
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 单视频源三帧槽所有权管理器，DDR 域内根据帧开始、结束、错误、写入数和显示边界维护接收/就绪/显示/退役状态。
// Module Name: frame_slot_manager
// File Name: source/memory/frame_slot_manager.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 单视频源三帧槽所有权管理器，DDR 域内根据帧开始、结束、错误、写入数和显示边界维护接收/就绪/显示/退役状态。
// 仅分配未被读者 lease 占用的 FREE 槽；READY 要求见到 frame_end、输入流排空且 HMIC 实际接受字数恰好等于 C_FRAME_WORDS；异常也须排空后释放。
//
// Dependencies: 外部已完成 CDC 的帧事件、实际写接受计数、读者 lease/retire 信号。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-09-13
// Module Name: frame_slot_manager
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: 单 source 三槽帧所有权管理器。运行于 DDR core_clk 域；输入事件
//              必须已经完成 CDC。grant 只分配 FREE 槽，显示槽仅在 HDMI 帧边界
//              快照。READY 的唯一条件是 frame_end 已见、stream drained 且实际
//              HMIC 接受 beat 恰好为 C_FRAME_WORDS；异常槽在 drained 后才释放。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module frame_slot_manager #(
    parameter integer C_FRAME_WORDS = 12800,
    parameter integer C_SLOT_COUNT  = 3,
    parameter integer C_EXTERNAL_LEASE_ENABLE = 0
)(
    // All event inputs are single-clock pulses/levels already synchronized into
    // the DDR core clock domain. i_frame_prepare requests one complete source
    // frame; o_grant_valid/o_grant_slot is the stable descriptor event.
    input             i_clk,
    input             i_rst_n,
    input             i_frame_prepare,
    output reg        o_grant_valid,
    output reg [1:0]  o_grant_slot,
    output reg        o_grant_toggle,
    input             i_hmic_write_beat,
    input             i_frame_end,
    // Assert only after source FIFO is empty and this source has no outstanding
    // HMIC AW/W transaction. It is the final ownership release/commit fence.
    input             i_write_stream_drained,
    input             i_overflow,
    input             i_error,
    input             i_hdmi_frame_boundary,
    input             i_retire_done,
    input [2:0]       i_external_slot_held,
    output [15:0]     o_display_epoch,
    output reg        o_frame_ready,
    output reg        o_frame_aborted,
    output reg [1:0]  o_display_slot,
    output reg        o_display_valid,
    output reg        o_display_switch,
    output reg [31:0] o_valid_frame_count,
    output reg [31:0] o_dropped_frame_count,
    output reg [31:0] o_display_commit_count,
    output [2:0]      o_slot0_state,
    output [2:0]      o_slot1_state,
    output [2:0]      o_slot2_state,
    output reg [1:0]  o_receiving_slot,
    output reg [15:0] o_written_words
);
    localparam [2:0] ST_FREE       = 3'd0;
    localparam [2:0] ST_RECEIVING  = 3'd1;
    localparam [2:0] ST_READY      = 3'd2;
    localparam [2:0] ST_DISPLAYING = 3'd3;
    localparam [2:0] ST_RETIRING   = 3'd4;

    reg [2:0] reg_slot0_state;
    reg [2:0] reg_slot1_state;
    reg [2:0] reg_slot2_state;
    reg       reg_receiving_valid;
    reg [1:0] reg_receiving_slot;
    reg [31:0] reg_written_words;
    reg       reg_frame_end_seen;
    reg       reg_frame_invalid;
    reg       reg_latest_ready_valid;
    reg [1:0] reg_latest_ready_slot;
    reg       reg_retiring_valid;
    reg [1:0] reg_retiring_slot;
    reg [15:0] reg_slot0_epoch;
    reg [15:0] reg_slot1_epoch;
    reg [15:0] reg_slot2_epoch;
    wire [2:0] external_held;
    assign external_held = C_EXTERNAL_LEASE_ENABLE != 0 ? i_external_slot_held : 3'd0;
    assign o_display_epoch = o_display_slot == 0 ? reg_slot0_epoch :
                             (o_display_slot == 1 ? reg_slot1_epoch : reg_slot2_epoch);

    wire [31:0] next_written_words = reg_written_words + (i_hmic_write_beat ? 32'd1 : 32'd0);
    wire        beat_overflow = i_hmic_write_beat && (reg_written_words >= C_FRAME_WORDS);
    wire        next_invalid = reg_frame_invalid || i_overflow || i_error || beat_overflow;
    wire        next_end_seen = reg_frame_end_seen || i_frame_end;
    wire        stream_complete = i_write_stream_drained && next_end_seen;
    wire        stream_good = stream_complete && !next_invalid && (next_written_words == C_FRAME_WORDS);
    // A latched error/overflow may abort without a well-formed frame_end;
    // it is still held until drained so outstanding HMIC writes cannot race
    // a subsequent grant. A clean short frame requires frame_end first.
    wire        stream_bad = i_write_stream_drained &&
                             (next_invalid || (next_end_seen && (next_written_words != C_FRAME_WORDS)));

    assign o_slot0_state = reg_slot0_state;
    assign o_slot1_state = reg_slot1_state;
    assign o_slot2_state = reg_slot2_state;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            reg_slot0_state        <= ST_FREE;
            reg_slot1_state        <= ST_FREE;
            reg_slot2_state        <= ST_FREE;
            reg_receiving_valid    <= 1'b0;
            reg_receiving_slot     <= 2'd0;
            reg_written_words      <= 32'd0;
            reg_frame_end_seen     <= 1'b0;
            reg_frame_invalid      <= 1'b0;
            reg_latest_ready_valid <= 1'b0;
            reg_latest_ready_slot  <= 2'd0;
            reg_retiring_valid     <= 1'b0;
            reg_retiring_slot      <= 2'd0;
            reg_slot0_epoch        <= 16'd0;
            reg_slot1_epoch        <= 16'd0;
            reg_slot2_epoch        <= 16'd0;
            o_grant_valid          <= 1'b0;
            o_grant_slot           <= 2'd0;
            o_grant_toggle         <= 1'b0;
            o_frame_ready          <= 1'b0;
            o_frame_aborted        <= 1'b0;
            o_display_slot         <= 2'd0;
            o_display_valid        <= 1'b0;
            o_display_switch       <= 1'b0;
            o_valid_frame_count    <= 32'd0;
            o_dropped_frame_count  <= 32'd0;
            o_display_commit_count <= 32'd0;
            o_receiving_slot       <= 2'd0;
            o_written_words        <= 16'd0;
        end else begin
            o_grant_valid    <= 1'b0;
            o_frame_ready    <= 1'b0;
            o_frame_aborted  <= 1'b0;
            o_display_switch <= 1'b0;

            // 旧 DISPLAYING 槽只有读侧确认排空后才变 FREE。
            if (i_retire_done && reg_retiring_valid) begin
                case (reg_retiring_slot)
                    2'd0: reg_slot0_state <= ST_FREE;
                    2'd1: reg_slot1_state <= ST_FREE;
                    default: reg_slot2_state <= ST_FREE;
                endcase
                reg_retiring_valid <= 1'b0;
            end

            // grant 只针对 FREE 槽；同一 source 只允许一个 RECEIVING。
            if (i_frame_prepare && !reg_receiving_valid) begin
                if (reg_slot0_state == ST_FREE && !external_held[0]) begin
                    reg_slot0_state <= ST_RECEIVING; reg_receiving_slot <= 2'd0;
                    reg_receiving_valid <= 1'b1; reg_written_words <= 32'd0;
                    reg_frame_end_seen <= 1'b0; reg_frame_invalid <= 1'b0;
                    o_receiving_slot <= 2'd0; o_written_words <= 16'd0;
                    o_grant_slot <= 2'd0; o_grant_valid <= 1'b1; o_grant_toggle <= ~o_grant_toggle;
                end else if (reg_slot1_state == ST_FREE && !external_held[1]) begin
                    reg_slot1_state <= ST_RECEIVING; reg_receiving_slot <= 2'd1;
                    reg_receiving_valid <= 1'b1; reg_written_words <= 32'd0;
                    reg_frame_end_seen <= 1'b0; reg_frame_invalid <= 1'b0;
                    o_receiving_slot <= 2'd1; o_written_words <= 16'd0;
                    o_grant_slot <= 2'd1; o_grant_valid <= 1'b1; o_grant_toggle <= ~o_grant_toggle;
                end else if (reg_slot2_state == ST_FREE && !external_held[2]) begin
                    reg_slot2_state <= ST_RECEIVING; reg_receiving_slot <= 2'd2;
                    reg_receiving_valid <= 1'b1; reg_written_words <= 32'd0;
                    reg_frame_end_seen <= 1'b0; reg_frame_invalid <= 1'b0;
                    o_receiving_slot <= 2'd2; o_written_words <= 16'd0;
                    o_grant_slot <= 2'd2; o_grant_valid <= 1'b1; o_grant_toggle <= ~o_grant_toggle;
                end else begin
                    o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
                end
            end

            if (reg_receiving_valid) begin
                if (i_frame_end) reg_frame_end_seen <= 1'b1;
                if (i_hmic_write_beat) begin
                    if (!beat_overflow) begin
                        reg_written_words <= next_written_words;
                        o_written_words <= next_written_words[15:0];
                    end
                end
                // Latch faults and overlength immediately, but keep ownership until
                // stream_drained proves no outstanding AW/W can still touch the slot.
                if (i_overflow || i_error || beat_overflow)
                    reg_frame_invalid <= 1'b1;

                if (stream_good) begin
                    // A source can finish another frame while an older READY
                    // frame is still waiting for HDMI boundary. Keep only the
                    // newest complete frame; never disturb DISPLAYING/RETIRING.
                    if ((reg_receiving_slot != 2'd0) && (reg_slot0_state == ST_READY))
                        reg_slot0_state <= ST_FREE;
                    if ((reg_receiving_slot != 2'd1) && (reg_slot1_state == ST_READY))
                        reg_slot1_state <= ST_FREE;
                    if ((reg_receiving_slot != 2'd2) && (reg_slot2_state == ST_READY))
                        reg_slot2_state <= ST_FREE;
                    case (reg_receiving_slot)
                        2'd0: begin reg_slot0_state <= ST_READY; reg_slot0_epoch <= o_valid_frame_count[15:0] + 16'd1; end
                        2'd1: begin reg_slot1_state <= ST_READY; reg_slot1_epoch <= o_valid_frame_count[15:0] + 16'd1; end
                        default: begin reg_slot2_state <= ST_READY; reg_slot2_epoch <= o_valid_frame_count[15:0] + 16'd1; end
                    endcase
                    reg_receiving_valid <= 1'b0;
                    reg_frame_end_seen <= 1'b0;
                    reg_frame_invalid <= 1'b0;
                    reg_latest_ready_slot <= reg_receiving_slot;
                    reg_latest_ready_valid <= 1'b1;
                    o_frame_ready <= 1'b1;
                    o_valid_frame_count <= o_valid_frame_count + 1'b1;
                end else if (stream_bad) begin
                    case (reg_receiving_slot)
                        2'd0: reg_slot0_state <= ST_FREE;
                        2'd1: reg_slot1_state <= ST_FREE;
                        default: reg_slot2_state <= ST_FREE;
                    endcase
                    reg_receiving_valid <= 1'b0;
                    reg_frame_end_seen <= 1'b0;
                    reg_frame_invalid <= 1'b0;
                    o_frame_aborted <= 1'b1;
                    o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
                end
            end

            // READY -> DISPLAYING only at HDMI frame boundary. No READY means
            // retain the previous display; RETIRING blocks another switch.
            // If a new frame commits on the same edge as vblank, defer the
            // snapshot one boundary so the just-updated latest pointer is used;
            // this also prevents the old READY slot from being re-promoted.
            if (i_hdmi_frame_boundary && !reg_retiring_valid &&
                reg_latest_ready_valid && !stream_good) begin
                if ((reg_latest_ready_slot == 2'd0) && (reg_slot0_state == ST_READY)) begin
                    if (o_display_valid) begin
                        case (o_display_slot)
                            2'd0: reg_slot0_state <= ST_RETIRING;
                            2'd1: reg_slot1_state <= ST_RETIRING;
                            default: reg_slot2_state <= ST_RETIRING;
                        endcase
                        reg_retiring_valid <= 1'b1; reg_retiring_slot <= o_display_slot;
                    end
                    reg_slot0_state <= ST_DISPLAYING; o_display_slot <= 2'd0;
                    o_display_valid <= 1'b1; o_display_switch <= 1'b1;
                    o_display_commit_count <= o_display_commit_count + 1'b1;
                    reg_latest_ready_valid <= 1'b0;
                end else if ((reg_latest_ready_slot == 2'd1) && (reg_slot1_state == ST_READY)) begin
                    if (o_display_valid) begin
                        case (o_display_slot)
                            2'd0: reg_slot0_state <= ST_RETIRING;
                            2'd1: reg_slot1_state <= ST_RETIRING;
                            default: reg_slot2_state <= ST_RETIRING;
                        endcase
                        reg_retiring_valid <= 1'b1; reg_retiring_slot <= o_display_slot;
                    end
                    reg_slot1_state <= ST_DISPLAYING; o_display_slot <= 2'd1;
                    o_display_valid <= 1'b1; o_display_switch <= 1'b1;
                    o_display_commit_count <= o_display_commit_count + 1'b1;
                    reg_latest_ready_valid <= 1'b0;
                end else if ((reg_latest_ready_slot == 2'd2) && (reg_slot2_state == ST_READY)) begin
                    if (o_display_valid) begin
                        case (o_display_slot)
                            2'd0: reg_slot0_state <= ST_RETIRING;
                            2'd1: reg_slot1_state <= ST_RETIRING;
                            default: reg_slot2_state <= ST_RETIRING;
                        endcase
                        reg_retiring_valid <= 1'b1; reg_retiring_slot <= o_display_slot;
                    end
                    reg_slot2_state <= ST_DISPLAYING; o_display_slot <= 2'd2;
                    o_display_valid <= 1'b1; o_display_switch <= 1'b1;
                    o_display_commit_count <= o_display_commit_count + 1'b1;
                    reg_latest_ready_valid <= 1'b0;
                end
            end
        end
    end
endmodule
