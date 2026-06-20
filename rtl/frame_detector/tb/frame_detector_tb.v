`timescale 1ns/1ps

module frame_detector_tb();

  localparam CLK_FREQ  = 1_000_000;
  localparam BAUD_RATE = 100_000;
  localparam CLKS_PER_BIT   = CLK_FREQ / BAUD_RATE;        // 10
  localparam SILENCE_PERIOD = (77 * CLKS_PER_BIT) / 2;     // 385

  reg        clk;
  reg        rstb;
  reg  [7:0] byte_in;
  reg        byte_valid;
  reg        frame_ack;
  reg  [7:0] read_addr;

  wire [7:0] frame_len;
  wire       frame_done;
  wire [7:0] read_data;

  integer pass_cnt, fail_cnt;

  frame_detector #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) dut (
    .clk        (clk),
    .rstb       (rstb),
    .byte_in    (byte_in),
    .byte_valid (byte_valid),
    .frame_ack  (frame_ack),
    .frame_len  (frame_len),
    .frame_done (frame_done),
    .read_addr  (read_addr),
    .read_data  (read_data)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  task send_byte;
    input [7:0] b;
    begin
      @(posedge clk);
      byte_in    <= b;
      byte_valid <= 1;
      @(posedge clk);
      byte_valid <= 0;
      byte_in    <= 8'h00;
    end
  endtask

  // Waits long enough for silence_cnt to reach SILENCE_PERIOD-1,
  // then #1 lets the NBA update of frame_done/frame_len settle
  task wait_silence;
    begin
      repeat (SILENCE_PERIOD) @(posedge clk);
      #1;
    end
  endtask

  task pulse_ack;
    begin
      @(posedge clk);
      frame_ack <= 1;
      @(posedge clk);
      frame_ack <= 0;
    end
  endtask

  task check_byte;
    input [7:0] addr;
    input [7:0] expected;
    begin
      read_addr = addr;
      #1;
      if (read_data === expected) begin
        $display("PASS read_data[%0d] = 0x%02X", addr, read_data);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL read_data[%0d] expected 0x%02X got 0x%02X", addr, expected, read_data);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  task check_frame_len;
    input [7:0] expected;
    input [127:0] label;
    begin
      if (frame_len === expected) begin
        $display("PASS %s frame_len = %0d", label, frame_len);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s frame_len expected %0d got %0d", label, expected, frame_len);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  task check_done;
    input expected;
    input [127:0] label;
    begin
      if (frame_done === expected) begin
        $display("PASS %s frame_done = %0d", label, frame_done);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s frame_done expected %0d got %0d", label, expected, frame_done);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  initial begin
    $dumpfile("build/vcd/frame_detector_tb.vcd");
    $dumpvars(0, frame_detector_tb);

    pass_cnt = 0;
    fail_cnt = 0;

    rstb       = 0;
    byte_in    = 0;
    byte_valid = 0;
    frame_ack  = 0;
    read_addr  = 0;

    repeat (3) @(posedge clk);
    rstb = 1;

    // -------------------------------------------------
    // Sync gap: first silence period after reset must
    // NOT produce a frame, even though frame_len == 0
    // -------------------------------------------------
    $display("--- sync gap ---");
    wait_silence();
    check_done(1'b0, "sync gap");
    check_frame_len(8'd0, "sync gap");

    // -------------------------------------------------
    // Valid 5-byte frame: ADDR FC D0 CRC_LO CRC_HI
    // -------------------------------------------------
    $display("--- valid 5-byte frame ---");
    send_byte(8'h11);
    send_byte(8'h03);
    send_byte(8'hAA);
    send_byte(8'h12);
    send_byte(8'h34);
    wait_silence();

    check_done(1'b1, "5-byte frame");
    check_frame_len(8'd5, "5-byte frame");
    check_byte(8'd0, 8'h11);
    check_byte(8'd1, 8'h03);
    check_byte(8'd2, 8'hAA);
    check_byte(8'd3, 8'h12);
    check_byte(8'd4, 8'h34);

    pulse_ack();
    @(posedge clk);
    check_frame_len(8'd0, "after ack");

    // -------------------------------------------------
    // Short 3-byte frame: must be discarded silently
    // -------------------------------------------------
    $display("--- short 3-byte frame (discard) ---");
    send_byte(8'h22);
    send_byte(8'h04);
    send_byte(8'h56);
    wait_silence();

    check_done(1'b0, "short frame");
    @(posedge clk);
    #1;
    check_frame_len(8'd0, "after discard");

    // -------------------------------------------------
    // Back-to-back valid frames
    // -------------------------------------------------
    $display("--- back-to-back frames ---");
    send_byte(8'h01);
    send_byte(8'h02);
    send_byte(8'h03);
    send_byte(8'h04);
    wait_silence();
    check_done(1'b1, "frame A");
    check_frame_len(8'd4, "frame A");
    check_byte(8'd0, 8'h01);
    check_byte(8'd3, 8'h04);
    pulse_ack();
    @(posedge clk);

    send_byte(8'hAA);
    send_byte(8'hBB);
    send_byte(8'hCC);
    send_byte(8'hDD);
    send_byte(8'hEE);
    wait_silence();
    check_done(1'b1, "frame B");
    check_frame_len(8'd5, "frame B");
    check_byte(8'd0, 8'hAA);
    check_byte(8'd4, 8'hEE);
    pulse_ack();
    @(posedge clk);

    $display("-------------------------------");
    $display("%0d passed  %0d failed", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #1_000_000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
