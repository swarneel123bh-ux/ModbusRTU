`timescale 1ns/1ps
module modbus_master_fsm_tb;

  localparam CLK_FREQ       = 1_000_000;
  localparam BAUD_RATE      = 100_000;
  localparam CLKS_PER_BIT   = CLK_FREQ / BAUD_RATE;
  localparam SILENCE        = (77 * CLKS_PER_BIT) / 2;
  localparam MAX_RETRIES    = 3;
  localparam TIMEOUT_CYCLES = CLKS_PER_BIT * 200;  // 2000 cycles — enough for 9-byte response + 3.5T

  reg  clk, rstb;
  wire master_ser_tx;
  wire slave_ser_tx;

  integer pass_cnt, fail_cnt;

  // Host interface
  reg  [7:0] host_byte_in;
  reg        host_byte_valid;
  reg        host_frame_done;
  wire       host_byte_ready;
  wire [7:0] host_byte_out;
  wire       host_byte_out_valid;
  wire       host_frame_out_done;
  wire       req_ready;
  wire       req_timeout;
  wire       req_failed;

  modbus_master_fsm #(
    .CLK_FREQ      (CLK_FREQ),
    .BAUD_RATE     (BAUD_RATE),
    .MAX_RETRIES   (MAX_RETRIES),
    .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
  ) dut (
    .clk                (clk),
    .rstb               (rstb),
    .ser_rx             (slave_ser_tx),
    .ser_tx             (master_ser_tx),
    .host_byte_in       (host_byte_in),
    .host_byte_valid    (host_byte_valid),
    .host_byte_ready    (host_byte_ready),
    .host_frame_done    (host_frame_done),
    .host_byte_out      (host_byte_out),
    .host_byte_out_valid(host_byte_out_valid),
    .host_frame_out_done(host_frame_out_done),
    .req_ready          (req_ready),
    .req_timeout        (req_timeout),
    .req_failed         (req_failed)
  );

  // Testbench slave RX — receives master's request on ser_tx
  wire [7:0] slave_rx_data;
  wire       slave_rx_valid;

  uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) slave_rx (
    .clk        (clk),
    .rstb       (rstb),
    .ser_rx     (master_ser_tx),
    .data       (slave_rx_data),
    .data_valid (slave_rx_valid),
    .frame_error()
  );

  // Testbench slave TX — sends response to master's ser_rx
  reg  [7:0] slave_tx_data;
  reg        slave_tx_valid;
  wire       slave_tx_ready;

  uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)) slave_tx_inst (
    .clk       (clk),
    .rstb      (rstb),
    .data      (slave_tx_data),
    .tx_valid  (slave_tx_valid),
    .tx_ready  (slave_tx_ready),
    .ser_tx_out(slave_ser_tx)
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

  // Capture host_byte_out bytes as they stream out (one per clock cycle)
  reg [7:0] captured [0:255];
  integer   cap_idx;

  always @(posedge clk) begin
    if (host_byte_out_valid) begin
      captured[cap_idx] = host_byte_out;
      cap_idx           = cap_idx + 1;
    end
  end

  // -------------------------------------------------------
  // Tasks
  // -------------------------------------------------------

  task send_host_byte;
    input [7:0] b;
    begin
      @(posedge clk);
      host_byte_in    <= b;
      host_byte_valid <= 1;
      @(posedge clk);
      host_byte_valid <= 0;
      host_byte_in    <= 0;
    end
  endtask

  task pulse_host_frame_done;
    begin
      @(posedge clk);
      host_frame_done <= 1;
      @(posedge clk);
      host_frame_done <= 0;
    end
  endtask

  task wait_ready;
    begin
      while (!req_ready) @(posedge clk);
      @(posedge clk);
    end
  endtask

  task send_slave_byte;
    input [7:0] b;
    begin
      @(posedge clk);
      slave_tx_data  <= b;
      slave_tx_valid <= 1;
      @(posedge clk);
      slave_tx_valid <= 0;
      @(negedge slave_tx_ready);
      @(posedge slave_tx_ready);
    end
  endtask

  task wait_silence;
    begin
      repeat (SILENCE) @(posedge clk);
    end
  endtask

  task check_byte_at;
    input integer   idx;
    input [7:0]     expected;
    input [127:0]   label;
    begin
      if (captured[idx] === expected) begin
        $display("PASS %s[%0d]: 0x%02X", label, idx, captured[idx]);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s[%0d]: expected 0x%02X got 0x%02X",
                 label, idx, expected, captured[idx]);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  reg [15:0] req_crc, resp_crc;
  integer timeout_fire_cnt;

  // Track timeout pulses
  always @(posedge clk) begin
    if (req_timeout) timeout_fire_cnt = timeout_fire_cnt + 1;
  end

  initial begin
    $dumpfile("build/vcd/modbus_master_fsm_tb.vcd");
    $dumpvars(0, modbus_master_fsm_tb);

    pass_cnt         = 0;
    fail_cnt         = 0;
    host_byte_in     = 0;
    host_byte_valid  = 0;
    host_frame_done  = 0;
    slave_tx_data    = 0;
    slave_tx_valid   = 0;
    rstb             = 0;
    cap_idx          = 0;
    timeout_fire_cnt = 0;

    repeat (5) @(posedge clk);
    rstb = 1;
    wait_silence();  // frame_detector sync after reset

    // -------------------------------------------------------
    // T1: FC03 read 2 holding registers from slave 0x11
    // Request:  [11][03][00][01][00][02][CRC_LO][CRC_HI]
    // Response: [11][03][04][AB][CD][12][34][CRC_LO][CRC_HI]
    // -------------------------------------------------------
    $display("--- T1: FC03 read 2 registers ---");
    cap_idx = 0;

    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h02, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h04, resp_crc);
    resp_crc = crc16(8'hAB, resp_crc);
    resp_crc = crc16(8'hCD, resp_crc);
    resp_crc = crc16(8'h12, resp_crc);
    resp_crc = crc16(8'h34, resp_crc);

    fork
      begin
        wait_ready();
        send_host_byte(8'h11); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(8'h00); send_host_byte(8'h02);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        repeat (8) @(posedge slave_rx_valid);
        repeat (5) @(posedge clk);
        send_slave_byte(8'h11); send_slave_byte(8'h03);
        send_slave_byte(8'h04);
        send_slave_byte(8'hAB); send_slave_byte(8'hCD);
        send_slave_byte(8'h12); send_slave_byte(8'h34);
        send_slave_byte(resp_crc[7:0]); send_slave_byte(resp_crc[15:8]);
      end
      begin
        @(posedge host_frame_out_done);
        #1;
        check_byte_at(0, 8'h11,          "T1");
        check_byte_at(1, 8'h03,          "T1");
        check_byte_at(2, 8'h04,          "T1");
        check_byte_at(3, 8'hAB,          "T1");
        check_byte_at(4, 8'hCD,          "T1");
        check_byte_at(5, 8'h12,          "T1");
        check_byte_at(6, 8'h34,          "T1");
        check_byte_at(7, resp_crc[7:0],  "T1");
        check_byte_at(8, resp_crc[15:8], "T1");
      end
    join

    // -------------------------------------------------------
    // T2: FC06 write 0x5678 to register 0x0003
    // Request:  [11][06][00][03][56][78][CRC_LO][CRC_HI]
    // Response: echo = identical frame
    // -------------------------------------------------------
    $display("--- T2: FC06 write register ---");
    cap_idx = 0;

    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h56, req_crc);
    req_crc = crc16(8'h78, req_crc);

    fork
      begin
        wait_ready();
        send_host_byte(8'h11); send_host_byte(8'h06);
        send_host_byte(8'h00); send_host_byte(8'h03);
        send_host_byte(8'h56); send_host_byte(8'h78);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        repeat (8) @(posedge slave_rx_valid);
        repeat (5) @(posedge clk);
        // Echo response
        send_slave_byte(8'h11); send_slave_byte(8'h06);
        send_slave_byte(8'h00); send_slave_byte(8'h03);
        send_slave_byte(8'h56); send_slave_byte(8'h78);
        send_slave_byte(req_crc[7:0]); send_slave_byte(req_crc[15:8]);
      end
      begin
        @(posedge host_frame_out_done);
        #1;
        check_byte_at(0, 8'h11,         "T2");
        check_byte_at(1, 8'h06,         "T2");
        check_byte_at(2, 8'h00,         "T2");
        check_byte_at(3, 8'h03,         "T2");
        check_byte_at(4, 8'h56,         "T2");
        check_byte_at(5, 8'h78,         "T2");
        check_byte_at(6, req_crc[7:0],  "T2");
        check_byte_at(7, req_crc[15:8], "T2");
      end
    join

    // -------------------------------------------------------
    // T3: Slave never responds → MAX_RETRIES exhausted → req_failed
    // -------------------------------------------------------
    $display("--- T3: no slave response → retry → req_failed ---");
    cap_idx          = 0;
    timeout_fire_cnt = 0;

    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);

    fork
      begin
        wait_ready();
        send_host_byte(8'h11); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
        // No slave response task — let it timeout
      end
      begin
        @(posedge req_failed);
        #1;
        if (timeout_fire_cnt == MAX_RETRIES) begin
          $display("PASS T3: req_failed after %0d timeouts", timeout_fire_cnt);
          pass_cnt = pass_cnt + 1;
        end else begin
          $display("FAIL T3: expected %0d timeouts got %0d",
                   MAX_RETRIES, timeout_fire_cnt);
          fail_cnt = fail_cnt + 1;
        end
      end
    join

    // -------------------------------------------------------
    // T4: Slave exception response (FC|0x80)
    // Verify master passes it through unchanged
    // -------------------------------------------------------
    $display("--- T4: slave exception response passthrough ---");
    cap_idx = 0;

    req_crc = crc16(8'h11, 16'hFFFF);
    req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h40, req_crc);  // OOR address
    req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h01, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF);
    resp_crc = crc16(8'h83, resp_crc);  // FC03 | 0x80
    resp_crc = crc16(8'h02, resp_crc);  // exception code 0x02

    fork
      begin
        wait_ready();
        send_host_byte(8'h11); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h40);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        repeat (8) @(posedge slave_rx_valid);
        repeat (5) @(posedge clk);
        send_slave_byte(8'h11);
        send_slave_byte(8'h83);
        send_slave_byte(8'h02);
        send_slave_byte(resp_crc[7:0]);
        send_slave_byte(resp_crc[15:8]);
      end
      begin
        @(posedge host_frame_out_done);
        #1;
        check_byte_at(0, 8'h11,          "T4");
        check_byte_at(1, 8'h83,          "T4");
        check_byte_at(2, 8'h02,          "T4");
        check_byte_at(3, resp_crc[7:0],  "T4");
        check_byte_at(4, resp_crc[15:8], "T4");
      end
    join

    $display("-------------------------------");
    $display("%0d passed  %0d failed", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #500_000_000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
