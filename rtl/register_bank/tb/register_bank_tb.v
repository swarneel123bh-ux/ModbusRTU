`timescale 1ns/1ps
module register_bank_tb;

  localparam COIL_COUNT     = 64;
  localparam DISCRETE_COUNT = 64;
  localparam HOLDING_COUNT  = 64;
  localparam INPUT_COUNT    = 64;

  localparam COIL     = 2'd0;
  localparam DISCRETE = 2'd1;
  localparam HOLDING  = 2'd2;
  localparam INPUT    = 2'd3;

  reg         clk;
  reg         rstb;
  reg  [1:0]  regtype;
  reg  [15:0] addr;
  reg         wen;
  reg  [15:0] wdata;
  wire [15:0] rdata;
  wire        addr_valid;

  integer pass_cnt, fail_cnt;

  register_bank #(
    .COIL_COUNT    (COIL_COUNT),
    .DISCRETE_COUNT(DISCRETE_COUNT),
    .HOLDING_COUNT (HOLDING_COUNT),
    .INPUT_COUNT   (INPUT_COUNT)
  ) dut (
    .clk       (clk),
    .rstb      (rstb),
    .regtype   (regtype),
    .addr      (addr),
    .wen       (wen),
    .wdata     (wdata),
    .rdata     (rdata),
    .addr_valid(addr_valid)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  task write_reg;
    input [1:0]  rt;
    input [15:0] a;
    input [15:0] d;
    begin
      @(posedge clk);
      regtype <= rt;
      addr    <= a;
      wdata   <= d;
      wen     <= 1;
      @(posedge clk);
      wen     <= 0;
    end
  endtask

  task check_read;
    input [1:0]   rt;
    input [15:0]  a;
    input [15:0]  expected;
    input [127:0] label;
    begin
      regtype = rt;
      addr    = a;
      #1;
      if (rdata === expected) begin
        $display("PASS %s rdata=0x%04X", label, rdata);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s expected 0x%04X got 0x%04X", label, expected, rdata);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  task check_addr_valid;
    input [1:0]   rt;
    input [15:0]  a;
    input         expected;
    input [127:0] label;
    begin
      regtype = rt;
      addr    = a;
      #1;
      if (addr_valid === expected) begin
        $display("PASS %s addr_valid=%0d", label, addr_valid);
        pass_cnt = pass_cnt + 1;
      end else begin
        $display("FAIL %s expected addr_valid=%0d got %0d", label, expected, addr_valid);
        fail_cnt = fail_cnt + 1;
      end
    end
  endtask

  initial begin
    $dumpfile("build/vcd/register_bank_tb.vcd");
    $dumpvars(0, register_bank_tb);

    pass_cnt = 0;
    fail_cnt = 0;

    rstb    = 0;
    regtype = 0;
    addr    = 0;
    wen     = 0;
    wdata   = 0;

    repeat (2) @(posedge clk);
    rstb = 1;
    @(posedge clk);

    // -------------------------------------------------
    // Holding register write/read
    // -------------------------------------------------
    $display("--- holding registers ---");
    write_reg(HOLDING, 16'd10, 16'hABCD);
    check_read(HOLDING, 16'd10, 16'hABCD, "holding[10]");

    // -------------------------------------------------
    // Coil write/read (bit-level)
    // -------------------------------------------------
    $display("--- coils ---");
    write_reg(COIL, 16'd5, 16'h0001);  // set coil 5
    check_read(COIL, 16'd5, 16'h0001, "coil[5] set");
    check_read(COIL, 16'd6, 16'h0000, "coil[6] untouched");

    write_reg(COIL, 16'd5, 16'h0000);  // clear coil 5
    check_read(COIL, 16'd5, 16'h0000, "coil[5] cleared");

    // -------------------------------------------------
    // Discrete input: write should be IGNORED (read-only)
    // -------------------------------------------------
    $display("--- discrete inputs (read-only) ---");
    write_reg(DISCRETE, 16'd3, 16'h0001);
    check_read(DISCRETE, 16'd3, 16'h0000, "discrete[3] write ignored");

    // -------------------------------------------------
    // Input register: write should be IGNORED (read-only)
    // -------------------------------------------------
    $display("--- input registers (read-only) ---");
    write_reg(INPUT, 16'd7, 16'h1234);
    check_read(INPUT, 16'd7, 16'h0000, "input[7] write ignored");

    // -------------------------------------------------
    // Out-of-range address -> addr_valid = 0, rdata = 0
    // -------------------------------------------------
    $display("--- out of range ---");
    check_addr_valid(HOLDING, 16'd64, 1'b0, "holding[64] OOR");
    check_read(HOLDING, 16'd64, 16'h0000, "holding[64] rdata=0");

    check_addr_valid(COIL, 16'd100, 1'b0, "coil[100] OOR");

    // Out-of-range write must not corrupt in-range data
    write_reg(HOLDING, 16'd64, 16'hFFFF);
    check_read(HOLDING, 16'd10, 16'hABCD, "holding[10] unaffected by OOR write");

    // -------------------------------------------------
    // Boundary check: last valid index
    // -------------------------------------------------
    $display("--- boundary ---");
    check_addr_valid(HOLDING, 16'd63, 1'b1, "holding[63] valid");
    check_addr_valid(HOLDING, 16'd64, 1'b0, "holding[64] invalid");

    write_reg(HOLDING, 16'd63, 16'h5555);
    check_read(HOLDING, 16'd63, 16'h5555, "holding[63] write/read");

    $display("-------------------------------");
    $display("%0d passed  %0d failed", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #100000;
    $display("TIMEOUT");
    $finish;
  end

endmodule
