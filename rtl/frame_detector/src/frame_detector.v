`timescale 1ns/1ps

module frame_detector #(
  parameter CLK_FREQ  = 50_000_000,
  parameter BAUD_RATE = 9600
)(
  input  wire	clk,										// Master clock
  input  wire rstb,										// Active low reset signal

  // From uart
  input  wire [7:0] byte_in,      		// from uart_rx data output
  input  wire byte_valid,   					// from uart_rx data_valid

  // Internal buffer + signals
  input wire frame_ack,								// reader pulses to let module clear buffer
  output reg [7:0] frame_len,    			// number of bytes received
  output reg frame_done,    					// 1-cycle pulse on frame complete

  // Reader side conns
  input wire [7:0] read_addr,					// Address to read from in the internal buffer (max 255)
  output wire [7:0] read_data					// Output byte
);

	localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
	// 3.5 char times = 3.5 * 11 = 38.5 bit times = 38.5 * CLKS_PER_BIT = (77/2) * CLKS_PER_BIT
	localparam SILENCE_PERIOD = (77 * CLKS_PER_BIT) / 2;

	reg [31:0] silence_cnt;
  reg        synced;       // 0 until first post-reset silence seen
  reg        frame_ready;  // frame complete, waiting for reader ack
  reg        discard;      // pulse: frame too short, throw away
  reg [7:0]  frame_buffer [0:255];

	assign read_data = frame_buffer[read_addr];

	// byte capture logic
	always @(posedge clk or negedge rstb) begin
		if (!rstb) begin
			frame_len <= 0;
		end else if (frame_ready && frame_ack || discard) begin
			frame_len <= 0;
		end else if (byte_valid && synced) begin
			frame_buffer[frame_len] <= byte_in;
			frame_len <= frame_len + 1;
		end

	end

	// silence timer, sync and frame done/ready logic
	always @(posedge clk or negedge rstb) begin
		if (!rstb) begin
			silence_cnt <= 0;
			synced 			<= 0;
			frame_done 	<= 0;
			frame_ready <= 0;
			discard 		<= 0;
		end else begin
			frame_done <= 0;
			discard <= 0;

			if (frame_ready && frame_ack) begin
				frame_ready <= 0;
			end else if (byte_valid) begin
				silence_cnt <= 0;
			end else if (silence_cnt == SILENCE_PERIOD - 1) begin
				silence_cnt <= 0;
				if (!synced) begin
					synced <= 1;
				end else if (frame_len >= 4) begin
					frame_done <= 1;
					frame_ready <= 1;
				end else begin
					discard <= 1;
				end
			end else begin
				silence_cnt <= silence_cnt + 1;
			end

		end
	end



endmodule
