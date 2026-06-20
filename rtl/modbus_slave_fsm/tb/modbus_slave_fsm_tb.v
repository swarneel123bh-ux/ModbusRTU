`timescale 1ns/1ps
module modbus_slave_fsm_tb;

  localparam CLK_FREQ      = 1_000_000;
  localparam BAUD_RATE     = 100_000;
  localparam CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;
  localparam SILENCE       = (77 * CLKS_PER_BIT) / 2;
  localparam SLAVE_ADDR    = 8'h11;

  reg  clk, rstb;
  wire ser_line;   // master TX → slave RX (ser_rx)
  wire slave_tx;   // slave TX → master RX

  integer pass_cnt, fail_cnt;

  // Master TX
  reg  [7:0] m_tx_data;
  reg        m_tx_valid;
  wire       m_tx_ready;

  uart_tx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) m_tx (
    .clk     (clk),
    .rstb   (rstb),
    .data (m_tx_data),
    .tx_valid(m_tx_valid),
    .tx_ready(m_tx_ready),
    .ser_tx_out  (ser_line)
  );

  // DUT — slave FSM
  modbus_slave_fsm #(
    .DEFAULT_SLAVE_ADDR(SLAVE_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE)
  ) dut (
    .clk       (clk),
    .rstb      (rstb),
    .ser_rx    (ser_line),
    .slave_addr(SLAVE_ADDR),
    .ser_tx    (slave_tx)
  );

  // Master RX — decodes slave's serial response
  wire [7:0] m_rx_data;
  wire       m_rx_valid;
  wire       m_rx_frame_err;

  uart_rx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) m_rx (
    .clk        (clk),
    .rstb       (rstb),
    .ser_rx     (slave_tx),
    .data       (m_rx_data),
    .data_valid (m_rx_valid),
    .frame_error(m_rx_frame_err)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  // CRC16 Modbus function (matches module: POLY=0xA001, INIT=0xFFFF)
  function [15:0] crc16;
    input [7:0]  data;
    input [15:0] crc_in;
    integer j;
    reg [15:0] crc;
    begin
      crc = crc_in ^ {8'h00, data};
      for (j = 0; j < 8; j = j + 1) begin
        if (crc[0]) crc = (crc >> 1) ^ 16'hA001;
        else        crc = crc >> 1;
      end
      crc16 = crc;
    end
  endfunction

  task send_byte;
    input [7:0] b;
    begin
      @(posedge clk);
      m_tx_data  <= b;
      m_tx_valid <= 1;
      @(posedge clk);
      m_tx_valid <= 0;
      @(negedge m_tx_ready);
      @(posedge m_tx_ready);
    end
  endtask

  task wait_silence;
    begin
      repeat (SILENCE) @(posedge clk);
      #1;
    end
  endtask

  task check_byte;
    input [7:0]   expected;
    input [127:0] label;
    begin
      if (m_rx_data === expected) begin
        $display("PASS %s: 0x%02X", label, m_rx_data);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s: expected 0x%02X got 0x%02X", label, expected, m_rx_data);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  task wait_no_response;
    input integer cycles;
    integer k;
    reg got_response;
    begin
      got_response = 0;
      for (k = 0; k < cycles; k = k + 1) begin
        @(posedge clk);
        if (m_rx_valid) got_response = 1;
      end
      if (!got_response) begin
        $display("PASS no response (correct)");
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL unexpected response received");
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  reg [15:0] req_crc;
  reg [15:0] resp_crc;

  initial begin
    $dumpfile("build/vcd/modbus_slave_fsm_tb.vcd");
    $dumpvars(0, modbus_slave_fsm_tb);

    pass_cnt   = 0;
    fail_cnt   = 0;
    m_tx_valid = 0;
    m_tx_data  = 0;
    rstb       = 0;

    repeat (5) @(posedge clk);
    rstb = 1;
    wait_silence();   // frame_detector sync gap after reset

    // -------------------------------------------------------
    // Test 1: Valid addr, unsupported FC 0x7F → exception 0x01
    // -------------------------------------------------------
    $display("--- Test 1: unsupported FC 0x7F ---");
    req_crc  = crc16(8'h11, 16'hFFFF);
    req_crc  = crc16(8'h7F, req_crc);
    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'hFF, resp_crc);  // 0x7F | 0x80
    resp_crc = crc16(8'h01, resp_crc);  // exception code

    fork
      begin
        send_byte(8'h11);
        send_byte(8'h7F);
        send_byte(req_crc[7:0]);
        send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T1 ADDR");
        @(posedge m_rx_valid); check_byte(8'hFF,          "T1 FC|0x80");
        @(posedge m_rx_valid); check_byte(8'h01,          "T1 exc_code");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T1 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T1 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 2: Wrong slave address → no response
    // -------------------------------------------------------
    $display("--- Test 2: wrong address ---");
    req_crc = crc16(8'h22, 16'hFFFF);
    req_crc = crc16(8'h7F, req_crc);

    send_byte(8'h22);
    send_byte(8'h7F);
    send_byte(req_crc[7:0]);
    send_byte(req_crc[15:8]);
    wait_silence();
    wait_no_response(SILENCE * 2);

    // -------------------------------------------------------
    // Test 3: Correct address, deliberate bad CRC → no response
    // -------------------------------------------------------
    $display("--- Test 3: bad CRC ---");
    send_byte(8'h11);
    send_byte(8'h7F);
    send_byte(8'hDE);  // wrong CRC
    send_byte(8'hAD);
    wait_silence();
    wait_no_response(SILENCE * 2);

    // -------------------------------------------------------
    // Test 4: FC03 (not yet implemented) → exception 0x01
    // FC03 request is 8 bytes, CRC over first 6
    // -------------------------------------------------------
    $display("--- Test 4: FC03 not implemented → exception ---");
    req_crc  = crc16(8'h11, 16'hFFFF);
    req_crc  = crc16(8'h03, req_crc);
    req_crc  = crc16(8'h00, req_crc);
    req_crc  = crc16(8'h01, req_crc);
    req_crc  = crc16(8'h00, req_crc);
    req_crc  = crc16(8'h03, req_crc);
    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h83, resp_crc);  // 0x03 | 0x80
    resp_crc = crc16(8'h01, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h01);
        send_byte(8'h00); send_byte(8'h03);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T4 ADDR");
        @(posedge m_rx_valid); check_byte(8'h83,          "T4 FC|0x80");
        @(posedge m_rx_valid); check_byte(8'h01,          "T4 exc_code");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T4 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T4 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 5: Back-to-back — verifies FSM resets correctly
    // -------------------------------------------------------
    $display("--- Test 5: back-to-back ---");
    req_crc  = crc16(8'h11, 16'hFFFF);
    req_crc  = crc16(8'h7F, req_crc);
    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'hFF, resp_crc);
    resp_crc = crc16(8'h01, resp_crc);

    begin : test_5_block
      event t5a_done; // Event to synchronize TX and RX threads

      fork
        begin
          // Frame A
          send_byte(8'h11); send_byte(8'h7F);
          send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
          wait_silence();

          // Wait for the Slave to finish transmitting its response to Frame A
          @(t5a_done);

          // Small turnaround delay to mimic real Master behavior (optional but safe)
          wait_silence();

          // Frame B
          send_byte(8'h11); send_byte(8'h7F);
          send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
          wait_silence();
        end
        begin
          // Verify Frame A Response
          @(posedge m_rx_valid); check_byte(8'h11,          "T5a ADDR");
          @(posedge m_rx_valid); check_byte(8'hFF,          "T5a FC|0x80");
          @(posedge m_rx_valid); check_byte(8'h01,          "T5a exc_code");
          @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T5a CRC_LO");
          @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T5a CRC_HI");

          // Signal TX thread that Frame A is completely received
          -> t5a_done;

          // Verify Frame B Response
          @(posedge m_rx_valid); check_byte(8'h11,          "T5b ADDR");
          @(posedge m_rx_valid); check_byte(8'hFF,          "T5b FC|0x80");
          @(posedge m_rx_valid); check_byte(8'h01,          "T5b exc_code");
          @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T5b CRC_LO");
          @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T5b CRC_HI");
        end
      join
    end

    $display("-------------------------------");
    $display("%0d passed  %0d failed", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #100_000_000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
