`timescale 1ns/1ps

module modbus_master_fsm #(
  parameter CLK_FREQ       = 50_000_000,
  parameter BAUD_RATE      = 9600,
  parameter MAX_RETRIES    = 3,
  parameter TIMEOUT_CYCLES = CLK_FREQ
)(
  input  wire        clk,
  input  wire        rstb,

  // Modbus serial line
  input  wire        ser_rx,
  output wire        ser_tx,

  // Host write interface (host pushes request frame bytes one at a time)
  input  wire [7:0]  host_byte_in,
  input  wire        host_byte_valid,   // pulse per byte
  output reg         host_byte_ready,   // high = can accept byte
  input  wire        host_frame_done,   // pulse after last byte of request

  // Host read interface (master streams response bytes back to host)
  output reg  [7:0]  host_byte_out,
  output reg         host_byte_out_valid,
  output reg         host_frame_out_done,

  // Status
  output reg         req_ready,     // high = idle, free to send new request
  output reg         req_timeout,   // pulse on each timeout/retry
  output reg         req_failed     // pulse: all retries exhausted
);

  localparam STATE_IDLE            = 4'd0;
  localparam STATE_RECV_HOST       = 4'd1;  // accumulate bytes from host
  localparam STATE_SEND            = 4'd2;  // stream buffer to Modbus UART TX
  localparam STATE_SEND_WAIT       = 4'd3;
  localparam STATE_WAIT_RESPONSE   = 4'd4;
  localparam STATE_COPY_RESPONSE   = 4'd5;  // copy frame_detector buf to local buf
  localparam STATE_STREAM_RESPONSE = 4'd6;  // stream response bytes to host
  localparam STATE_DONE            = 4'd7;

  reg [3:0]  fsm_state;

  // Internal buffer — holds request (host→master) then response (slave→host)
  reg [7:0]  buffer [0:255];
  reg [7:0]  buf_len;       // valid byte count in buffer

  // TX
  reg [7:0]  tx_send_idx;

  // RX frame read
  reg  [7:0] fsm_frame_read_addr;
  wire [7:0] fsm_frame_read_data;

  // Response stream
  reg [7:0]  stream_idx;

  // Timeout + retry
  reg [31:0] timeout_cnt;
  reg [7:0]  retry_cnt;

  // UART RX
  wire [7:0] uart_recv_data;
  wire       uart_recv_data_valid;
  wire       uart_recv_frame_error;

  uart_rx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) uart_recv (
    .clk        (clk),
    .rstb       (rstb),
    .ser_rx     (ser_rx),
    .data       (uart_recv_data),
    .data_valid (uart_recv_data_valid),
    .frame_error(uart_recv_frame_error)
  );

  // UART TX
  reg  [7:0] uart_send_data;
  reg        uart_send_tx_valid;
  wire       uart_send_tx_ready;

  uart_tx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) uart_send (
    .clk       (clk),
    .rstb      (rstb),
    .data      (uart_send_data),
    .tx_valid  (uart_send_tx_valid),
    .tx_ready  (uart_send_tx_ready),
    .ser_tx_out(ser_tx)
  );

  // Frame detector
  reg        frmdet_frame_ack;
  wire [7:0] frmdet_frame_len;
  wire       frmdet_frame_done;

  frame_detector #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) frmdet (
    .clk       (clk),
    .rstb      (rstb),
    .byte_in   (uart_recv_data),
    .byte_valid(uart_recv_data_valid),
    .frame_ack (frmdet_frame_ack),
    .frame_len (frmdet_frame_len),
    .frame_done(frmdet_frame_done),
    .read_addr (fsm_frame_read_addr),
    .read_data (fsm_frame_read_data)
  );

  always @(posedge clk or negedge rstb) begin
    if (!rstb) begin
      fsm_state            <= STATE_IDLE;
      req_ready            <= 1;
      host_byte_ready      <= 0;
      host_byte_out_valid  <= 0;
      host_byte_out        <= 0;
      host_frame_out_done  <= 0;
      req_timeout          <= 0;
      req_failed           <= 0;
      uart_send_tx_valid   <= 0;
      frmdet_frame_ack     <= 0;
      buf_len              <= 0;
      tx_send_idx          <= 0;
      stream_idx           <= 0;
      timeout_cnt          <= 0;
      retry_cnt            <= 0;
      fsm_frame_read_addr  <= 0;
    end else begin
      // Defaults
      req_timeout         <= 0;
      req_failed          <= 0;
      uart_send_tx_valid  <= 0;
      frmdet_frame_ack    <= 0;
      host_byte_out_valid <= 0;
      host_frame_out_done <= 0;

      case (fsm_state)

        STATE_IDLE: begin
          req_ready       <= 1;
          host_byte_ready <= 0;
          if (host_byte_valid) begin
            // First byte arriving — start accumulating
            req_ready       <= 0;
            host_byte_ready <= 1;
            buffer[0]       <= host_byte_in;
            buf_len         <= 1;
            fsm_state       <= STATE_RECV_HOST;
          end
        end

        STATE_RECV_HOST: begin
          host_byte_ready <= 1;
          if (host_frame_done) begin
            // All bytes received, start sending
            host_byte_ready <= 0;
            tx_send_idx     <= 0;
            fsm_state       <= STATE_SEND;
          end else if (host_byte_valid) begin
            buffer[buf_len] <= host_byte_in;
            buf_len         <= buf_len + 1;
          end
        end

        STATE_SEND: begin
          if (tx_send_idx == buf_len) begin
            timeout_cnt         <= 0;
            fsm_frame_read_addr <= 0;
            fsm_state           <= STATE_WAIT_RESPONSE;
          end else if (uart_send_tx_ready) begin
            uart_send_data     <= buffer[tx_send_idx];
            uart_send_tx_valid <= 1;
            tx_send_idx        <= tx_send_idx + 1;
            fsm_state          <= STATE_SEND_WAIT;
          end
        end

        STATE_SEND_WAIT: begin
          if (!uart_send_tx_ready)
            fsm_state <= STATE_SEND;
        end

        STATE_WAIT_RESPONSE: begin
          if (frmdet_frame_done) begin
            timeout_cnt         <= 0;
            fsm_frame_read_addr <= 0;
            fsm_state           <= STATE_COPY_RESPONSE;
          end else if (timeout_cnt == TIMEOUT_CYCLES - 1) begin
            timeout_cnt <= 0;
            req_timeout <= 1;
            if (retry_cnt == MAX_RETRIES) begin
              req_failed <= 1;
              retry_cnt  <= 0;
              fsm_state  <= STATE_DONE;
            end else begin
              retry_cnt   <= retry_cnt + 1;
              tx_send_idx <= 0;
              fsm_state   <= STATE_SEND;
            end
          end else begin
            timeout_cnt <= timeout_cnt + 1;
          end
        end

        STATE_COPY_RESPONSE: begin
          buffer[fsm_frame_read_addr] <= fsm_frame_read_data;
          if (fsm_frame_read_addr == frmdet_frame_len - 1) begin
            buf_len          <= frmdet_frame_len;
            frmdet_frame_ack <= 1;
            stream_idx       <= 0;
            fsm_state        <= STATE_STREAM_RESPONSE;
          end else begin
            fsm_frame_read_addr <= fsm_frame_read_addr + 1;
          end
        end

        STATE_STREAM_RESPONSE: begin
          if (stream_idx == buf_len) begin
            host_frame_out_done <= 1;
            fsm_state           <= STATE_DONE;
          end else begin
            host_byte_out       <= buffer[stream_idx];
            host_byte_out_valid <= 1;
            stream_idx          <= stream_idx + 1;
          end
        end

        STATE_DONE: begin
          req_ready <= 1;
          fsm_state <= STATE_IDLE;
        end

        default: fsm_state <= STATE_IDLE;

      endcase
    end
  end

endmodule
