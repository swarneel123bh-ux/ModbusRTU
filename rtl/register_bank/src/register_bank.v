`timescale 1ns/1ps
module register_bank #(
	parameter COIL_COUNT     = 64,   // number of implemented coils
  parameter DISCRETE_COUNT = 64,   // number of implemented discrete inputs
  parameter HOLDING_COUNT  = 64,   // number of implemented holding registers
  parameter INPUT_COUNT    = 64    // number of implemented input registers
) (
	input wire clk,										// Master clock
	input wire rstb, 									// Active low reset

	input wire [1:0] regtype,					// Type of modbus register to address (coils/discrete inputs/...)
	input wire [15:0] addr,						// Address of the register to address
	input wire wen,										// Write signal
	input wire [15:0] wdata,					// Write data
	output reg [15:0] rdata,					// Read data
	output reg addr_valid							// In case invalid address, only if we override counts in testbench
);

	// Register type specifiers
	localparam COIL       = 2'd0;
  localparam DISCRETE   = 2'd1;
  localparam HOLDING    = 2'd2;
  localparam INPUT      = 2'd3;

  // Coils and DIs
  // Each bit represents a unique single hardware bit
  reg [COIL_COUNT-1:0] coils;
  reg [DISCRETE_COUNT-1:0] discrete_inputs;

  // 16 x 16 register files
  reg [15:0] holding_registers [0:HOLDING_COUNT-1];
  reg [15:0] input_registers [0:INPUT_COUNT-1];

  integer i;

  // Bounds check (optional)
  always @(*) begin
  	case (regtype)
   		COIL:			addr_valid = (addr < COIL_COUNT);
     	DISCRETE: addr_valid = (addr < DISCRETE_COUNT);
      HOLDING:	addr_valid = (addr < HOLDING_COUNT);
      INPUT:		addr_valid = (addr < INPUT_COUNT);
      default:	addr_valid = 0;
    endcase
  end

  // Write logic
  always @(posedge clk or negedge rstb) begin
  	if (!rstb) begin
   		coils 					<= {COIL_COUNT{1'b0}};
     	discrete_inputs <= {DISCRETE_COUNT{1'b0}};
      for (i = 0; i < HOLDING_COUNT; i = i + 1) begin
      	holding_registers[i] <= 16'h0000;
      end
      for (i = 0; i < INPUT_COUNT; i = i + 1) begin
      	input_registers[i] <= 16'h0000;
      end
   	end else if (wen && addr_valid) begin
     	case (regtype)
      	COIL: 		coils[addr[$clog2(COIL_COUNT)-1:0]] <= wdata[0];
       	HOLDING: 	holding_registers[addr[$clog2(HOLDING_COUNT)-1:0]] <= wdata;
        default:;
      endcase
    end
  end

  // Read logic (combinational)
  always @(*) begin
  	if (!addr_valid) begin
   		rdata = 16'h0000;
   	end else begin
    	case (regtype)
     		COIL:				rdata = {15'b0, coils[addr[$clog2(COIL_COUNT)-1:0]]};
       	DISCRETE:   rdata = {15'b0, discrete_inputs[addr[$clog2(DISCRETE_COUNT)-1:0]]};
        HOLDING:		rdata = holding_registers[addr[$clog2(HOLDING_COUNT)-1:0]];
        INPUT:			rdata = input_registers[addr[$clog2(INPUT_COUNT)-1:0]];
        default:		rdata = 16'h0000;
     	endcase
    end
  end

endmodule
