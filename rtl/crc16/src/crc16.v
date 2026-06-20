`timescale 1ns/1ps

module crc16 #(
	// Defaults are for modbus RTU
	parameter [15:0] POLY   = 16'hA001,
  parameter [15:0] INIT   = 16'hFFFF,
  parameter        REFIN  = 0,
  parameter        REFOUT = 0,
  parameter [15:0] XOROUT = 16'h0000
)(
  input  wire [15:0] crc_in,
  input  wire [ 7:0] data_in,
  output wire [15:0] crc_out
);

  function [7:0] reflect8;
    input [7:0] d;
    integer j;
    begin
      for (j = 0; j < 8; j = j + 1)
        reflect8[j] = d[7-j];
    end
  endfunction

  function [15:0] reflect16;
    input [15:0] d;
    integer j;
    begin
      for (j = 0; j < 16; j = j + 1)
        reflect16[j] = d[15-j];
    end
  endfunction

  reg [15:0] crc;
  reg [ 7:0] byte_in;
  integer    i;

  always @(*) begin
    byte_in = REFIN ? reflect8(data_in) : data_in;
    crc     = crc_in ^ {8'h00, byte_in};
    for (i = 0; i < 8; i = i + 1) begin
      if (crc[0])
        crc = (crc >> 1) ^ POLY;
      else
        crc = crc >> 1;
    end
  end

  assign crc_out = (REFOUT ? reflect16(crc) : crc) ^ XOROUT;

endmodule
