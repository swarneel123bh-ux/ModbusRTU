`timescale 1ns/1ps

module modbus_top_tb;

  localparam CLK_FREQ       = 1_000_000;
  localparam BAUD_RATE      = 100_000;
  localparam CLKS_PER_BIT   = CLK_FREQ / BAUD_RATE;
  localparam SILENCE        = (77 * CLKS_PER_BIT) / 2;
  localparam SLAVE_ADDR     = 8'h11;
  localparam MAX_RETRIES    = 2;
  localparam TIMEOUT_CYCLES = CLKS_PER_BIT * 200;

  reg  clk, rstb;
  integer pass_cnt, fail_cnt;

  // Serial wires between master and slave
  wire master_to_slave;
  wire slave_to_master;

  // Master host interface
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

  // dut_master — MASTER mode, controlled by testbench via host interface
  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_master (
    .clk                (clk),
    .rstb               (rstb),
    .mode               (1'b1),           // MASTER
    .ser_rx             (slave_to_master),
    .ser_tx             (master_to_slave),
    .slave_addr         (SLAVE_ADDR),
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

  // dut_slave — SLAVE mode, responds autonomously
  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_slave (
    .clk                (clk),
    .rstb               (rstb),
    .mode               (1'b0),           // SLAVE
    .ser_rx             (master_to_slave),
    .ser_tx             (slave_to_master),
    .slave_addr         (SLAVE_ADDR),
    .host_byte_in       (8'h00),
    .host_byte_valid    (1'b0),
    .host_byte_ready    (),
    .host_frame_done    (1'b0),
    .host_byte_out      (),
    .host_byte_out_valid(),
    .host_frame_out_done(),
    .req_ready          (),
    .req_timeout        (),
    .req_failed         ()
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

  // Capture host_byte_out stream
  reg [7:0]  captured [0:255];
  integer    cap_idx;

  always @(posedge clk) begin
    if (host_byte_out_valid) begin
      captured[cap_idx] = host_byte_out;
      cap_idx           = cap_idx + 1;
    end
  end

  // Track timeouts
  integer timeout_fire_cnt;
  always @(posedge clk) begin
    if (req_timeout) timeout_fire_cnt = timeout_fire_cnt + 1;
  end

  // -------------------------------------------------------
  // Tasks
  // -------------------------------------------------------
  task wait_silence;
    begin repeat (SILENCE) @(posedge clk); #1; end
  endtask

  task send_host_byte;
    input [7:0] b;
    begin
      @(posedge clk); host_byte_in <= b; host_byte_valid <= 1;
      @(posedge clk); host_byte_valid <= 0; host_byte_in <= 0;
    end
  endtask

  task pulse_host_frame_done;
    begin
      @(posedge clk); host_frame_done <= 1;
      @(posedge clk); host_frame_done <= 0;
    end
  endtask

  task wait_master_ready;
    begin while (!req_ready) @(posedge clk); end
  endtask

  task check_captured;
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

  initial begin
    $dumpfile("build/vcd/modbus_top_tb.vcd");
    $dumpvars(0, modbus_top_tb);

    pass_cnt         = 0; fail_cnt = 0;
    cap_idx          = 0; timeout_fire_cnt = 0;
    host_byte_in     = 0; host_byte_valid  = 0;
    host_frame_done  = 0; rstb             = 0;

    repeat (5) @(posedge clk);
    rstb = 1;
    wait_silence();  // frame_detector sync on both instances

    // -------------------------------------------------------
    // T1: FC06 write reg[5] = 0x1234 (master→slave)
    //     Slave echoes request as response
    // -------------------------------------------------------
    $display("--- T1: FC06 write reg[5]=0x1234 ---");
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h12, req_crc);  req_crc = crc16(8'h34, req_crc);

    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h06);
        send_host_byte(8'h00); send_host_byte(8'h05);
        send_host_byte(8'h12); send_host_byte(8'h34);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        @(posedge host_frame_out_done); #1;
        check_captured(0, 8'h11,         "T1");
        check_captured(1, 8'h06,         "T1");
        check_captured(2, 8'h00,         "T1");
        check_captured(3, 8'h05,         "T1");
        check_captured(4, 8'h12,         "T1");
        check_captured(5, 8'h34,         "T1");
        check_captured(6, req_crc[7:0],  "T1");
        check_captured(7, req_crc[15:8], "T1");
      end
    join

    // -------------------------------------------------------
    // T2: FC03 read reg[5] — expect 0x1234 written in T1
    // -------------------------------------------------------
    $display("--- T2: FC03 read reg[5] expect 0x1234 ---");
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h01, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF); resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h02, resp_crc);
    resp_crc = crc16(8'h12, resp_crc); resp_crc = crc16(8'h34, resp_crc);

    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h05);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        @(posedge host_frame_out_done); #1;
        check_captured(0, 8'h11,          "T2");
        check_captured(1, 8'h03,          "T2");
        check_captured(2, 8'h02,          "T2");
        check_captured(3, 8'h12,          "T2");
        check_captured(4, 8'h34,          "T2");
        check_captured(5, resp_crc[7:0],  "T2");
        check_captured(6, resp_crc[15:8], "T2");
      end
    join

    // -------------------------------------------------------
    // T3: Wrong slave address → slave ignores → master times out
    // -------------------------------------------------------
    $display("--- T3: wrong slave address → req_failed ---");
    cap_idx = 0; timeout_fire_cnt = 0;
    req_crc = crc16(8'h22, 16'hFFFF); req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h05, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h01, req_crc);

    fork
      begin
        wait_master_ready();
        send_host_byte(8'h22); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h05);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        @(posedge req_failed); #1;
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
    // T4: Write two registers, read both back — multi-register
    //     FC06 reg[0]=0xABCD, FC06 reg[1]=0x5678, FC03 read qty=2
    // -------------------------------------------------------
    $display("--- T4: write reg[0]=0xABCD, reg[1]=0x5678, read both ---");

    // Write reg[0]
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'hAB, req_crc);  req_crc = crc16(8'hCD, req_crc);
    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h06);
        send_host_byte(8'h00); send_host_byte(8'h00);
        send_host_byte(8'hAB); send_host_byte(8'hCD);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin @(posedge host_frame_out_done); end
    join

    // Write reg[1]
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h06, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h01, req_crc);
    req_crc = crc16(8'h56, req_crc);  req_crc = crc16(8'h78, req_crc);
    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h06);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(8'h56); send_host_byte(8'h78);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin @(posedge host_frame_out_done); end
    join

    // Read reg[0] and reg[1]
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h03, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h02, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF); resp_crc = crc16(8'h03, resp_crc);
    resp_crc = crc16(8'h04, resp_crc);
    resp_crc = crc16(8'hAB, resp_crc); resp_crc = crc16(8'hCD, resp_crc);
    resp_crc = crc16(8'h56, resp_crc); resp_crc = crc16(8'h78, resp_crc);

    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h03);
        send_host_byte(8'h00); send_host_byte(8'h00);
        send_host_byte(8'h00); send_host_byte(8'h02);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        @(posedge host_frame_out_done); #1;
        check_captured(0, 8'h11,          "T4");
        check_captured(1, 8'h03,          "T4");
        check_captured(2, 8'h04,          "T4");
        check_captured(3, 8'hAB,          "T4");
        check_captured(4, 8'hCD,          "T4");
        check_captured(5, 8'h56,          "T4");
        check_captured(6, 8'h78,          "T4");
        check_captured(7, resp_crc[7:0],  "T4");
        check_captured(8, resp_crc[15:8], "T4");
      end
    join

    // -------------------------------------------------------
    // T5: Unsupported FC — slave sends exception, master
    //     passes it through unchanged
    // -------------------------------------------------------
    $display("--- T5: unsupported FC 0x7F → exception passthrough ---");
    cap_idx = 0;
    req_crc = crc16(8'h11, 16'hFFFF); req_crc = crc16(8'h7F, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h00, req_crc);
    req_crc = crc16(8'h00, req_crc);  req_crc = crc16(8'h01, req_crc);

    resp_crc = crc16(8'h11, 16'hFFFF); resp_crc = crc16(8'hFF, resp_crc);
    resp_crc = crc16(8'h01, resp_crc);  // exception code 0x01

    fork
      begin
        wait_master_ready();
        send_host_byte(8'h11); send_host_byte(8'h7F);
        send_host_byte(8'h00); send_host_byte(8'h00);
        send_host_byte(8'h00); send_host_byte(8'h01);
        send_host_byte(req_crc[7:0]); send_host_byte(req_crc[15:8]);
        pulse_host_frame_done();
      end
      begin
        @(posedge host_frame_out_done); #1;
        check_captured(0, 8'h11,          "T5");
        check_captured(1, 8'hFF,          "T5");
        check_captured(2, 8'h01,          "T5");
        check_captured(3, resp_crc[7:0],  "T5");
        check_captured(4, resp_crc[15:8], "T5");
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
