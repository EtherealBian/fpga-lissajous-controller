`timescale 1ns/1ps

// =============================================================================
// 模块名称：wireless_uart_controller
// 功能说明：实现无线自动模式的 UART 命令面、流程状态机、回复/事件排队和 DDS 设置保持。
// 职责边界：FPGA 负责可靠通信、范围检查、状态约束及频率/相位寄存；摄像头识别、扫频
//           策略和图像算法运行在上位机，二者通过 CRC8 帧协议协作。
// 状态流程：IDLE→RECOGNIZE→SCAN→ADJUST→DONE；断开无线模式、标定失效或 ABORT
//           均安全返回空闲状态。响应优先于异步 START/ABORTED 事件发送。
// 单位约定：频率使用 mHz 无符号整数；相位使用 Q0.16 周；多字节载荷为小端序。
// 时钟/复位：全部逻辑位于 CLOCK_HZ 时钟域；rst_n 异步低有效。
// 作者：Ethereal
// =============================================================================
module wireless_uart_controller #(
    parameter integer CLOCK_HZ = 100_000_000,
    parameter integer BAUD_RATE = 921_600,
    parameter integer MIN_FREQUENCY_MILLIHZ = 1_000_000,
    parameter integer MAX_FREQUENCY_MILLIHZ = 100_000_000
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       wireless_mode,
    input  logic       calibration_valid,
    input  logic       wireless_start_pulse,
    input  logic [1:0] wireless_start_pattern,
    input  logic       wireless_abort_pulse,
    input  logic       uart_rx,
    output logic       uart_tx,

    output logic [2:0] wireless_state,
    output logic [1:0] active_pattern,
    output logic       wireless_pulse_active,
    output logic       wireless_sine_active,
    output logic       wireless_done,
    output logic [31:0] frequency_millihz,
    output logic [15:0] phase_q16,
    output logic       frequency_update_pulse,
    output logic       phase_update_pulse,
    output logic       uart_rx_framing_error,
    output logic       uart_packet_crc_error,
    output logic       uart_packet_protocol_error
);

    localparam logic [2:0] STATE_IDLE      = 3'd0;
    localparam logic [2:0] STATE_RECOGNIZE = 3'd1;
    localparam logic [2:0] STATE_SCAN      = 3'd2;
    localparam logic [2:0] STATE_ADJUST    = 3'd3;
    localparam logic [2:0] STATE_DONE      = 3'd4;

    localparam logic [7:0] CMD_SET_RANGE =
        8'h10;
    localparam logic [7:0] CMD_SCAN_BEGIN =
        8'h11;
    localparam logic [7:0] CMD_SET_FREQUENCY =
        8'h12;
    localparam logic [7:0] CMD_SET_PHASE =
        8'h13;
    localparam logic [7:0] CMD_DONE =
        8'h14;
    localparam logic [7:0] CMD_ABORT =
        8'h15;
    localparam logic [7:0] CMD_GET_STATUS =
        8'h16;
    localparam logic [7:0] CMD_SET_FREQUENCY_PHASE =
        8'h17;

    localparam logic [7:0] RSP_ACK =
        8'h80;
    localparam logic [7:0] RSP_START =
        8'h81;
    localparam logic [7:0] RSP_STATUS =
        8'h82;
    localparam logic [7:0] RSP_ABORTED =
        8'h83;

    localparam logic [7:0] STATUS_OK =
        8'd0;
    localparam logic [7:0] STATUS_BAD_LENGTH =
        8'd1;
    localparam logic [7:0] STATUS_BAD_VALUE =
        8'd2;
    localparam logic [7:0] STATUS_WRONG_STATE =
        8'd3;
    localparam logic [7:0] STATUS_NOT_WIRELESS =
        8'd4;
    localparam logic [7:0] STATUS_UNSUPPORTED =
        8'd5;
    localparam logic [7:0] STATUS_NOT_CALIBRATED =
        8'd6;

    logic [7:0] rx_byte;
    logic rx_byte_valid;
    logic packet_valid;
    logic [7:0] packet_command;
    logic [7:0] packet_sequence;
    logic [3:0] packet_length;
    logic [63:0] packet_payload;

    logic [2:0] state;
    logic range_valid;
    logic frequency_valid;
    logic [31:0] range_min_millihz;
    logic [31:0] range_max_millihz;
    logic [7:0] last_status;
    logic [7:0] status_flags;

    logic response_pending;
    logic [7:0] response_command;
    logic [7:0] response_sequence;
    logic [3:0] response_length;
    logic [63:0] response_payload;
    logic event_pending;
    logic [7:0] event_sequence;
    logic [7:0] event_command;
    logic [3:0] event_length;
    logic [63:0] event_payload;

    logic packet_tx_request_valid;
    logic packet_tx_request_ready;
    logic [7:0] packet_tx_command;
    logic [7:0] packet_tx_sequence;
    logic [3:0] packet_tx_length;
    logic [63:0] packet_tx_payload;

    logic [31:0] payload_word_0;
    logic [31:0] payload_word_1;
    logic [15:0] payload_phase;

    // 将小端载荷切分为范围、频率和相位字段，命令执行时统一做长度检查。
    assign payload_word_0 = packet_payload[31:0];
    assign payload_word_1 = packet_payload[63:32];
    assign payload_phase = packet_payload[47:32];

    // 对外状态由内部状态机直接译码，LED 与 DAC 路由无需理解协议细节。
    assign wireless_state = state;
    assign wireless_pulse_active =
        wireless_mode && (state == STATE_RECOGNIZE);
    assign wireless_sine_active =
        wireless_mode && frequency_valid &&
        ((state == STATE_SCAN) ||
         (state == STATE_ADJUST) ||
         (state == STATE_DONE));
    assign wireless_done =
        wireless_mode && (state == STATE_DONE);

    // 发送仲裁：普通命令响应优先，空闲时再发送按键触发的异步事件。
    always @* begin
        status_flags = 8'd0;
        status_flags[0] = wireless_mode;
        status_flags[1] = range_valid;
        status_flags[2] = frequency_valid;
        status_flags[3] = wireless_pulse_active;
        status_flags[4] = wireless_sine_active;
        status_flags[5] = wireless_done;
        status_flags[6] = calibration_valid;

        packet_tx_request_valid =
            response_pending || event_pending;
        if (response_pending) begin
            packet_tx_command = response_command;
            packet_tx_sequence = response_sequence;
            packet_tx_length = response_length;
            packet_tx_payload = response_payload;
        end else begin
            packet_tx_command = event_command;
            packet_tx_sequence = event_sequence;
            packet_tx_length = event_length;
            packet_tx_payload = event_payload;
        end
    end

    // 协议栈分层：字节接收、数据包解析、数据包发送分别由独立模块实现。
    uart_byte_rx #(
        .CLOCK_HZ(CLOCK_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_byte_rx (
        .clk(clk),
        .rst_n(rst_n),
        .rx(uart_rx),
        .data(rx_byte),
        .valid(rx_byte_valid),
        .framing_error(uart_rx_framing_error)
    );

    uart_packet_receiver #(
        .CLOCK_HZ(CLOCK_HZ),
        .MAX_PAYLOAD_BYTES(8)
    ) u_uart_packet_receiver (
        .clk(clk),
        .rst_n(rst_n),
        .byte_data(rx_byte),
        .byte_valid(rx_byte_valid),
        .packet_valid(packet_valid),
        .packet_command(packet_command),
        .packet_sequence(packet_sequence),
        .packet_length(packet_length),
        .packet_payload(packet_payload),
        .crc_error(uart_packet_crc_error),
        .protocol_error(uart_packet_protocol_error)
    );

    uart_packet_transmitter #(
        .CLOCK_HZ(CLOCK_HZ),
        .BAUD_RATE(BAUD_RATE),
        .MAX_PAYLOAD_BYTES(8)
    ) u_uart_packet_transmitter (
        .clk(clk),
        .rst_n(rst_n),
        .request_valid(packet_tx_request_valid),
        .request_ready(packet_tx_request_ready),
        .request_command(packet_tx_command),
        .request_sequence(packet_tx_sequence),
        .request_length(packet_tx_length),
        .request_payload(packet_tx_payload),
        .tx(uart_tx)
    );

    // 主控制时序锁存状态、执行命令、排队响应，并产生单周期 DDS 更新脉冲。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            active_pattern <= 2'd0;
            range_valid <= 1'b0;
            frequency_valid <= 1'b0;
            range_min_millihz <= 32'd0;
            range_max_millihz <= 32'd0;
            frequency_millihz <= 32'd0;
            phase_q16 <= 16'd0;
            frequency_update_pulse <= 1'b0;
            phase_update_pulse <= 1'b0;
            last_status <= STATUS_OK;
            response_pending <= 1'b0;
            response_command <= 8'd0;
            response_sequence <= 8'd0;
            response_length <= 4'd0;
            response_payload <= 64'd0;
            event_pending <= 1'b0;
            event_sequence <= 8'd0;
            event_command <= 8'd0;
            event_length <= 4'd0;
            event_payload <= 64'd0;
        end else begin
            frequency_update_pulse <= 1'b0;
            phase_update_pulse <= 1'b0;

            // 发送器接受当前消息后清除相应 pending 标志，避免重复发送。
            if (packet_tx_request_valid &&
                packet_tx_request_ready) begin
                if (response_pending) begin
                    response_pending <= 1'b0;
                end else begin
                    event_pending <= 1'b0;
                end
            end

            // 无线模式关闭或标定失效时强制回到安全空闲态，清除所有频率有效状态。
            if (!wireless_mode || !calibration_valid) begin
                state <= STATE_IDLE;
                active_pattern <= 2'd0;
                range_valid <= 1'b0;
                frequency_valid <= 1'b0;
                frequency_millihz <= 32'd0;
                phase_q16 <= 16'd0;
                event_pending <= 1'b0;
            end

            if (packet_valid) begin
                // 每条命令先建立默认 ACK，再按命令码覆盖状态码、载荷和状态转移。
                response_pending <= 1'b1;
                response_command <= RSP_ACK;
                response_sequence <= packet_sequence;
                response_length <= 4'd2;
                response_payload <= {
                    48'd0, STATUS_UNSUPPORTED,
                    packet_command
                };
                last_status <= STATUS_UNSUPPORTED;

                if (packet_command == CMD_GET_STATUS) begin
                    if (packet_length != 4'd0) begin
                        response_payload <= {
                            48'd0, STATUS_BAD_LENGTH,
                            packet_command
                        };
                        last_status <= STATUS_BAD_LENGTH;
                    end else begin
                        response_command <= RSP_STATUS;
                        response_length <= 4'd8;
                        response_payload <= {
                            frequency_millihz,
                            last_status,
                            status_flags,
                            6'd0, active_pattern,
                            5'd0, state
                        };
                    end
                end else if (!wireless_mode) begin
                    response_payload <= {
                        48'd0, STATUS_NOT_WIRELESS,
                        packet_command
                    };
                    last_status <= STATUS_NOT_WIRELESS;
                end else if (!calibration_valid) begin
                    response_payload <= {
                        48'd0, STATUS_NOT_CALIBRATED,
                        packet_command
                    };
                    last_status <= STATUS_NOT_CALIBRATED;
                end else begin
                    // 设置类命令同时检查长度、数值范围和当前流程状态。
                    case (packet_command)
                        CMD_SET_RANGE: begin
                            if (packet_length != 4'd8) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                (payload_word_0 <
                                 MIN_FREQUENCY_MILLIHZ) ||
                                (payload_word_1 >
                                 MAX_FREQUENCY_MILLIHZ) ||
                                (payload_word_0 >
                                 payload_word_1)) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_VALUE,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_VALUE;
                            end else if (
                                state != STATE_RECOGNIZE) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                range_min_millihz <=
                                    payload_word_0;
                                range_max_millihz <=
                                    payload_word_1;
                                range_valid <= 1'b1;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_SCAN_BEGIN: begin
                            if (packet_length != 4'd0) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                (state != STATE_RECOGNIZE) ||
                                !range_valid) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                state <= STATE_SCAN;
                                frequency_valid <= 1'b0;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_SET_FREQUENCY: begin
                            if (packet_length != 4'd4) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                (payload_word_0 <
                                 range_min_millihz) ||
                                (payload_word_0 >
                                 range_max_millihz)) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_VALUE,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_VALUE;
                            end else if (
                                (state != STATE_SCAN) &&
                                (state != STATE_ADJUST)) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                frequency_millihz <=
                                    payload_word_0;
                                frequency_valid <= 1'b1;
                                frequency_update_pulse <= 1'b1;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_SET_PHASE: begin
                            if (packet_length != 4'd2) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                ((state != STATE_SCAN) &&
                                 (state != STATE_ADJUST)) ||
                                !frequency_valid) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                phase_q16 <=
                                    packet_payload[15:0];
                                phase_update_pulse <= 1'b1;
                                state <= STATE_ADJUST;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_SET_FREQUENCY_PHASE: begin
                            if (packet_length != 4'd6) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                (payload_word_0 <
                                 range_min_millihz) ||
                                (payload_word_0 >
                                 range_max_millihz)) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_VALUE,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_VALUE;
                            end else if (
                                (state != STATE_SCAN) &&
                                (state != STATE_ADJUST)) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                frequency_millihz <=
                                    payload_word_0;
                                phase_q16 <= payload_phase;
                                frequency_valid <= 1'b1;
                                frequency_update_pulse <= 1'b1;
                                phase_update_pulse <= 1'b1;
                                state <= STATE_ADJUST;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_DONE: begin
                            if (packet_length != 4'd0) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else if (
                                ((state != STATE_SCAN) &&
                                 (state != STATE_ADJUST)) ||
                                !frequency_valid) begin
                                response_payload <= {
                                    48'd0, STATUS_WRONG_STATE,
                                    packet_command
                                };
                                last_status <= STATUS_WRONG_STATE;
                            end else begin
                                state <= STATE_DONE;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        CMD_ABORT: begin
                            if (packet_length != 4'd0) begin
                                response_payload <= {
                                    48'd0, STATUS_BAD_LENGTH,
                                    packet_command
                                };
                                last_status <= STATUS_BAD_LENGTH;
                            end else begin
                                state <= STATE_IDLE;
                                active_pattern <= 2'd0;
                                range_valid <= 1'b0;
                                frequency_valid <= 1'b0;
                                frequency_millihz <= 32'd0;
                                phase_q16 <= 16'd0;
                                response_payload <= {
                                    48'd0, STATUS_OK,
                                    packet_command
                                };
                                last_status <= STATUS_OK;
                            end
                        end

                        default: begin
                            response_payload <= {
                                48'd0, STATUS_UNSUPPORTED,
                                packet_command
                            };
                            last_status <= STATUS_UNSUPPORTED;
                        end
                    endcase
                end
            end

            // 板载按键发起识别流程时产生 START 异步事件，并冻结当前图形编号。
            if (wireless_mode &&
                calibration_valid &&
                wireless_start_pulse) begin
                state <= STATE_RECOGNIZE;
                active_pattern <= wireless_start_pattern;
                range_valid <= 1'b0;
                frequency_valid <= 1'b0;
                range_min_millihz <= 32'd0;
                range_max_millihz <= 32'd0;
                frequency_millihz <= 32'd0;
                phase_q16 <= 16'd0;
                phase_update_pulse <= 1'b1;
                last_status <= STATUS_OK;
                event_sequence <= event_sequence + 1'b1;
                event_command <= RSP_START;
                event_length <= 4'd1;
                event_payload <= {
                    56'd0, 6'd0, wireless_start_pattern
                };
                event_pending <= 1'b1;
            end

            if (wireless_mode &&
                wireless_abort_pulse &&
                (state != STATE_IDLE)) begin
                state <= STATE_IDLE;
                active_pattern <= 2'd0;
                range_valid <= 1'b0;
                frequency_valid <= 1'b0;
                range_min_millihz <= 32'd0;
                range_max_millihz <= 32'd0;
                frequency_millihz <= 32'd0;
                phase_q16 <= 16'd0;
                phase_update_pulse <= 1'b1;
                last_status <= STATUS_OK;
                event_sequence <= event_sequence + 1'b1;
                event_command <= RSP_ABORTED;
                event_length <= 4'd1;
                event_payload <= {
                    56'd0, 6'd0, active_pattern
                };
                event_pending <= 1'b1;
            end
        end
    end

endmodule
