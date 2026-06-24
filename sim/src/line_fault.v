`timescale 1ns/1ps

// line_fault — models transmission noise by flipping bits on a serial line.
//
// Deterministic given SEED, so a failure reproduces exactly on re-run. (seed with current time
// to have random behaviour)
//
// Mechanism: an LFSR rolls once per bit-time. If it fires, the line is
// inverted for one full bit-time (CLKS_PER_BIT cycles), guaranteeing the
// receiver's mid-bit sample sees the corrupted level (one bit flipped).
// RATE_PPM = expected bit-flips per million transmitted bits.

module line_fault #(
  parameter CLK_FREQ  = 1_000_000,
  parameter BAUD_RATE = 100_000,
  parameter SEED      = 32'hACE1_2345,
  parameter RATE_PPM  = 1000
)(
  input  wire clk,
  input  wire rstb,
  input  wire enable,
  input  wire line_in,
  output wire line_out
);
  localparam integer  CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
  // RATE_PPM / 1e6 * 2^32  ~=  RATE_PPM * 4295  (keep RATE_PPM well under 1e6)
  localparam [31:0]   THRESH = RATE_PPM * 32'd4295;

  reg [31:0] lfsr;
  reg [15:0] bit_cnt;
  reg [15:0] hold;
  reg        flip;

  wire [31:0] lfsr_next = {lfsr[30:0],
                           lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};

  always @(posedge clk or negedge rstb) begin
    if (!rstb) begin
      lfsr    <= SEED;
      bit_cnt <= 16'd0;
      hold    <= 16'd0;
      flip    <= 1'b0;
    end else begin
      lfsr <= lfsr_next;

      if (hold != 16'd0) begin
        hold <= hold - 16'd1;
        if (hold == 16'd1) flip <= 1'b0;
      end

      if (bit_cnt == CLKS_PER_BIT - 1) begin
        bit_cnt <= 16'd0;
        if (enable && hold == 16'd0 && lfsr_next < THRESH) begin
          flip <= 1'b1;
          hold <= CLKS_PER_BIT[15:0];
        end
      end else begin
        bit_cnt <= bit_cnt + 16'd1;
      end
    end
  end

  assign line_out = line_in ^ flip;
endmodule
