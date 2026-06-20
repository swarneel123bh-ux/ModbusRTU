`timescale 1ns/1ps

module crc16_tb();

  reg  [15:0] crc_reg;
  reg  [15:0] crc_reg2;
  reg  [ 7:0] data_in;
  wire [15:0] crc_out;

  integer pass_cnt, fail_cnt;

  // Right-shift implementation: REFIN=0 REFOUT=0 with reflected poly
  crc16 #(
    .POLY  (16'hA001),
    .INIT  (16'hFFFF),
    .REFIN (0),
    .REFOUT(0),
    .XOROUT(16'h0000)
  ) u_crc (
    .crc_in (crc_reg),
    .data_in(data_in),
    .crc_out(crc_out)
  );

  task reset_crc;
    begin
      crc_reg = 16'hFFFF;
      data_in = 8'h00;
    end
  endtask

  task feed_byte;
    input [7:0] b;
    begin
      data_in = b;
      #10;
      crc_reg = crc_out;
    end
  endtask

  task check;
    input [15:0] expected;
    input [127:0] label;
    begin
      if (crc_reg == expected) begin
        $display("PASS %s: 0x%04X", label, crc_reg);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s: expected 0x%04X  got 0x%04X", label, expected, crc_reg);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  initial begin
    $dumpfile("build/vcd/crc16_tb.vcd");
    $dumpvars(0, crc16_tb);
    pass_cnt = 0;
    fail_cnt = 0;

    // --------------------------------------------------
    // Test 1: Standard check value
    // "123456789" → Modbus CRC-16 = 0x4B37 (well-known)
    // --------------------------------------------------
    reset_crc();
    feed_byte(8'h31); feed_byte(8'h32); feed_byte(8'h33);
    feed_byte(8'h34); feed_byte(8'h35); feed_byte(8'h36);
    feed_byte(8'h37); feed_byte(8'h38); feed_byte(8'h39);
    check(16'h4B37, "123456789");

    // --------------------------------------------------
    // Test 2: INIT sanity
    // Before any bytes, accumulator must be 0xFFFF
    // --------------------------------------------------
    reset_crc();
    if (crc_reg == 16'hFFFF) begin
      $display("PASS INIT: 0x%04X", crc_reg);
      pass_cnt = pass_cnt + 1;
    end else begin
      $display("FAIL INIT: expected 0xFFFF  got 0x%04X", crc_reg);
      fail_cnt = fail_cnt + 1;
    end

    // --------------------------------------------------
    // Test 3: Reproducibility
    // Same input twice must give same CRC
    // --------------------------------------------------
    reset_crc();
    feed_byte(8'h55); feed_byte(8'hAA); feed_byte(8'hFF);
    crc_reg2 = crc_reg;

    reset_crc();
    feed_byte(8'h55); feed_byte(8'hAA); feed_byte(8'hFF);

    if (crc_reg == crc_reg2) begin
      $display("PASS reproducibility: 0x%04X", crc_reg);
      pass_cnt = pass_cnt + 1;
    end else begin
      $display("FAIL reproducibility: 0x%04X vs 0x%04X", crc_reg2, crc_reg);
      fail_cnt = fail_cnt + 1;
    end

    // --------------------------------------------------
    // Test 4: Order sensitivity
    // [0x55, 0xAA] must differ from [0xAA, 0x55]
    // --------------------------------------------------
    reset_crc();
    feed_byte(8'h55); feed_byte(8'hAA);
    crc_reg2 = crc_reg;

    reset_crc();
    feed_byte(8'hAA); feed_byte(8'h55);

    if (crc_reg != crc_reg2) begin
      $display("PASS order-sensitive: 0x%04X != 0x%04X", crc_reg2, crc_reg);
      pass_cnt = pass_cnt + 1;
    end else begin
      $display("FAIL order-sensitive: both gave 0x%04X", crc_reg);
      fail_cnt = fail_cnt + 1;
    end

    // --------------------------------------------------
    // Test 5: Single 0x00 byte
    // Verifies XOR with 0x00 does not leave CRC unchanged
    // --------------------------------------------------
    reset_crc();
    feed_byte(8'h00);
    if (crc_reg != 16'hFFFF) begin
      $display("PASS 0x00 changes CRC: 0x%04X", crc_reg);
      pass_cnt = pass_cnt + 1;
    end else begin
      $display("FAIL 0x00 left CRC unchanged at 0xFFFF");
      fail_cnt = fail_cnt + 1;
    end

    // --------------------------------------------------
    // Test 6: All-ones byte 0xFF
    // Verifies same — 0xFF XOR 0xFFFF low byte = 0xFF00
    // --------------------------------------------------
    reset_crc();
    feed_byte(8'hFF);
    if (crc_reg != 16'hFFFF) begin
      $display("PASS 0xFF changes CRC: 0x%04X", crc_reg);
      pass_cnt = pass_cnt + 1;
    end else begin
      $display("FAIL 0xFF left CRC unchanged");
      fail_cnt = fail_cnt + 1;
    end

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
