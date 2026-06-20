`timescale 1ns/1ps
module modbus_slave_fsm_tb;

  localparam CLK_FREQ     = 1_000_000;
  localparam BAUD_RATE    = 100_000;
  localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
  localparam SILENCE      = (77 * CLKS_PER_BIT) / 2;
  localparam SLAVE_ADDR   = 8'h11;

  reg  clk, rstb;
  wire ser_line;
  wire slave_tx;

  integer pass_cnt, fail_cnt;

  reg  [7:0] m_tx_data;
  reg        m_tx_valid;
  wire       m_tx_ready;

  uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) m_tx (
    .clk     (clk),
    .rstb    (rstb),
    .data    (m_tx_data),
    .tx_valid(m_tx_valid),
    .tx_ready(m_tx_ready),
    .ser_tx_out(ser_line)
  );

  modbus_slave_fsm #(
    .DEFAULT_SLAVE_ADDR(SLAVE_ADDR),
    .CLK_FREQ(CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) dut (
    .clk       (clk),
    .rstb      (rstb),
    .ser_rx    (ser_line),
    .slave_addr(SLAVE_ADDR),
    .ser_tx    (slave_tx)
  );

  wire [7:0] m_rx_data;
  wire       m_rx_valid;
  wire       m_rx_frame_err;

  uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) m_rx (
    .clk        (clk),
    .rstb       (rstb),
    .ser_rx     (slave_tx),
    .data       (m_rx_data),
    .data_valid (m_rx_valid),
    .frame_error(m_rx_frame_err)
  );

  initial clk = 0;
  always #5 clk = ~clk;

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

  // Send a full frame and wait for silence
  task send_frame;
    input [7:0] b0, b1, b2, b3, b4, b5, b6, b7;
    input [3:0] len;  // how many bytes to send (4 or 8)
    reg [15:0] crc;
    begin
      crc = 16'hFFFF;
      if (len >= 1) begin send_byte(b0); crc = crc16(b0, crc); end
      if (len >= 2) begin send_byte(b1); crc = crc16(b1, crc); end
      if (len >= 3) begin send_byte(b2); crc = crc16(b2, crc); end
      if (len >= 4) begin send_byte(b3); crc = crc16(b3, crc); end
      if (len >= 5) begin send_byte(b4); crc = crc16(b4, crc); end
      if (len >= 6) begin send_byte(b5); crc = crc16(b5, crc); end
      send_byte(crc[7:0]);
      send_byte(crc[15:8]);
      wait_silence();
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
    reg got;
    begin
      got = 0;
      for (k = 0; k < cycles; k = k + 1) begin
        @(posedge clk);
        if (m_rx_valid) got = 1;
      end
      if (!got) begin $display("PASS no response (correct)"); pass_cnt = pass_cnt + 1; end
      else      begin $display("FAIL unexpected response");   fail_cnt = fail_cnt + 1; end
    end
  endtask

  reg [15:0] req_crc, resp_crc;

  initial begin
    $dumpfile("build/vcd/modbus_slave_fsm_tb.vcd");
    $dumpvars(0, modbus_slave_fsm_tb);

    pass_cnt = 0; fail_cnt = 0;
    m_tx_valid = 0; m_tx_data = 0; rstb = 0;

    repeat (5) @(posedge clk);
    rstb = 1;
    wait_silence();

    // -------------------------------------------------------
    // Test 1: FC06 — write 0x1234 to holding register 0x0005
    // Request:  [11][06][00][05][12][34][CRC_LO][CRC_HI]
    // Response: echo = same 8 bytes
    // -------------------------------------------------------
    $display("--- T1: FC06 write reg[5] = 0x1234 ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h12, req_crc);
    req_crc = crc16(8'h34, req_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h06);
        send_byte(8'h00); send_byte(8'h05);
        send_byte(8'h12); send_byte(8'h34);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T1 ADDR");
        @(posedge m_rx_valid); check_byte(8'h06,          "T1 FC");
        @(posedge m_rx_valid); check_byte(8'h00,          "T1 reg_hi");
        @(posedge m_rx_valid); check_byte(8'h05,          "T1 reg_lo");
        @(posedge m_rx_valid); check_byte(8'h12,          "T1 val_hi");
        @(posedge m_rx_valid); check_byte(8'h34,          "T1 val_lo");
        @(posedge m_rx_valid); check_byte(req_crc[7:0],   "T1 CRC_LO");
        @(posedge m_rx_valid); check_byte(req_crc[15:8],  "T1 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 2: FC03 — read back register 5 (should be 0x1234)
    // Request:  [11][03][00][05][00][01][CRC_LO][CRC_HI]  (qty=1)
    // Response: [11][03][02][12][34][CRC_LO][CRC_HI]
    // -------------------------------------------------------
    $display("--- T2: FC03 read reg[5] (expect 0x1234) ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h02, resp_crc);  // byte_count = 1*2
    resp_crc = crc16(8'h12, resp_crc);
    resp_crc = crc16(8'h34, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h05);
        send_byte(8'h00); send_byte(8'h01);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T2 ADDR");
        @(posedge m_rx_valid); check_byte(8'h03,          "T2 FC");
        @(posedge m_rx_valid); check_byte(8'h02,          "T2 byte_cnt");
        @(posedge m_rx_valid); check_byte(8'h12,          "T2 data_hi");
        @(posedge m_rx_valid); check_byte(8'h34,          "T2 data_lo");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T2 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T2 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 3: FC06 write reg[0]=0xABCD, reg[1]=0x5678
    // Then FC03 read both back (qty=2)
    // -------------------------------------------------------
    $display("--- T3: FC06 write reg[0]=0xABCD ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'hAB, req_crc);
    req_crc = crc16(8'hCD, req_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h06);
        send_byte(8'h00); send_byte(8'h00);
        send_byte(8'hAB); send_byte(8'hCD);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,         "T3a ADDR");
        @(posedge m_rx_valid); check_byte(8'h06,         "T3a FC");
        @(posedge m_rx_valid); check_byte(8'h00,         "T3a reg_hi");
        @(posedge m_rx_valid); check_byte(8'h00,         "T3a reg_lo");
        @(posedge m_rx_valid); check_byte(8'hAB,         "T3a val_hi");
        @(posedge m_rx_valid); check_byte(8'hCD,         "T3a val_lo");
        @(posedge m_rx_valid); check_byte(req_crc[7:0],  "T3a CRC_LO");
        @(posedge m_rx_valid); check_byte(req_crc[15:8], "T3a CRC_HI");
      end
    join

    $display("--- T3: FC06 write reg[1]=0x5678 ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);
    req_crc = crc16(8'h56, req_crc);
    req_crc = crc16(8'h78, req_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h06);
        send_byte(8'h00); send_byte(8'h01);
        send_byte(8'h56); send_byte(8'h78);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,         "T3b ADDR");
        @(posedge m_rx_valid); check_byte(8'h06,         "T3b FC");
        @(posedge m_rx_valid); check_byte(8'h00,         "T3b reg_hi");
        @(posedge m_rx_valid); check_byte(8'h01,         "T3b reg_lo");
        @(posedge m_rx_valid); check_byte(8'h56,         "T3b val_hi");
        @(posedge m_rx_valid); check_byte(8'h78,         "T3b val_lo");
        @(posedge m_rx_valid); check_byte(req_crc[7:0],  "T3b CRC_LO");
        @(posedge m_rx_valid); check_byte(req_crc[15:8], "T3b CRC_HI");
      end
    join

    $display("--- T3: FC03 read reg[0..1] (expect ABCD, 5678) ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h02, req_crc);  // qty=2

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h04, resp_crc);  // byte_count = 2*2
    resp_crc = crc16(8'hAB, resp_crc);
    resp_crc = crc16(8'hCD, resp_crc);
    resp_crc = crc16(8'h56, resp_crc);
    resp_crc = crc16(8'h78, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h00);
        send_byte(8'h00); send_byte(8'h02);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T3c ADDR");
        @(posedge m_rx_valid); check_byte(8'h03,          "T3c FC");
        @(posedge m_rx_valid); check_byte(8'h04,          "T3c byte_cnt");
        @(posedge m_rx_valid); check_byte(8'hAB,          "T3c reg0_hi");
        @(posedge m_rx_valid); check_byte(8'hCD,          "T3c reg0_lo");
        @(posedge m_rx_valid); check_byte(8'h56,          "T3c reg1_hi");
        @(posedge m_rx_valid); check_byte(8'h78,          "T3c reg1_lo");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T3c CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T3c CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 4: FC06 — out of range address (reg[64], HOLDING_COUNT=64)
    // Expect exception 0x02
    // -------------------------------------------------------
    $display("--- T4: FC06 OOR address → exception 0x02 ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h40, req_crc);  // addr=64, OOR
    req_crc = crc16(8'hAB, req_crc);
    req_crc = crc16(8'hCD, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h86, resp_crc);  // FC06 | 0x80
    resp_crc = crc16(8'h02, resp_crc);  // illegal data address

    fork
      begin
        send_byte(8'h11); send_byte(8'h06);
        send_byte(8'h00); send_byte(8'h40);
        send_byte(8'hAB); send_byte(8'hCD);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T4 ADDR");
        @(posedge m_rx_valid); check_byte(8'h86,          "T4 FC|0x80");
        @(posedge m_rx_valid); check_byte(8'h02,          "T4 exc_code");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T4 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T4 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 5: FC03 — qty=0 → exception 0x03
    // -------------------------------------------------------
    $display("--- T5: FC03 qty=0 → exception 0x03 ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);  // qty=0

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h83, resp_crc);
    resp_crc = crc16(8'h03, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h00);
        send_byte(8'h00); send_byte(8'h00);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T5 ADDR");
        @(posedge m_rx_valid); check_byte(8'h83,          "T5 FC|0x80");
        @(posedge m_rx_valid); check_byte(8'h03,          "T5 exc_code");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T5 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T5 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 6: FC03 — qty=126 (>125) → exception 0x03
    // -------------------------------------------------------
    $display("--- T6: FC03 qty=126 → exception 0x03 ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h7E, req_crc);  // qty=126

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h83, resp_crc);
    resp_crc = crc16(8'h03, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h00);
        send_byte(8'h00); send_byte(8'h7E);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T6 ADDR");
        @(posedge m_rx_valid); check_byte(8'h83,          "T6 FC|0x80");
        @(posedge m_rx_valid); check_byte(8'h03,          "T6 exc_code");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T6 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T6 CRC_HI");
      end
    join

    // -------------------------------------------------------
    // Test 7: FC03 read register 5 again (0x1234 from Test 1)
    // Verifies register persists across multiple transactions
    // -------------------------------------------------------
    $display("--- T7: FC03 re-read reg[5] (expect 0x1234 from T1) ---");
    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h02, resp_crc);
    resp_crc = crc16(8'h12, resp_crc);
    resp_crc = crc16(8'h34, resp_crc);

    fork
      begin
        send_byte(8'h11); send_byte(8'h03);
        send_byte(8'h00); send_byte(8'h05);
        send_byte(8'h00); send_byte(8'h01);
        send_byte(req_crc[7:0]); send_byte(req_crc[15:8]);
        wait_silence();
      end
      begin
        @(posedge m_rx_valid); check_byte(8'h11,          "T7 ADDR");
        @(posedge m_rx_valid); check_byte(8'h03,          "T7 FC");
        @(posedge m_rx_valid); check_byte(8'h02,          "T7 byte_cnt");
        @(posedge m_rx_valid); check_byte(8'h12,          "T7 data_hi");
        @(posedge m_rx_valid); check_byte(8'h34,          "T7 data_lo");
        @(posedge m_rx_valid); check_byte(resp_crc[7:0],  "T7 CRC_LO");
        @(posedge m_rx_valid); check_byte(resp_crc[15:8], "T7 CRC_HI");
      end
    join

    $display("-------------------------------");
    $display("%0d passed  %0d failed", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #1_000_000_000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
