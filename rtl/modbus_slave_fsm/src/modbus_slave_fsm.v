`timescale 1ns/1ps

module modbus_slave_fsm #(
  parameter DEFAULT_SLAVE_ADDR = 8'h11,
  parameter CLK_FREQ  = 50_000_000,
  parameter BAUD_RATE = 9600
) (
	input wire clk,								// Master system clock
	input wire rstb,							// Active low reset
	input wire ser_rx,						// Serial input line
	input wire [7:0] slave_addr, 	// Modbus Slave address
	output wire ser_tx						// Serial output line
);

	// states
	localparam STATE_IDLE       = 4'd0;
  localparam STATE_CHECK_ADDR = 4'd1;
  localparam STATE_CRC_LOOP   = 4'd2;
  localparam STATE_GET_CRC_LO = 4'd3;
  localparam STATE_GET_CRC_HI = 4'd4;
  localparam STATE_CHECK_CRC  = 4'd5;
  localparam STATE_DECODE_FC  = 4'd6;
  localparam STATE_FETCH_FIELDS = 4'd7;
  localparam STATE_EXECUTE    = 4'd8;
  localparam STATE_SEND       = 4'd9;
  localparam STATE_DONE       = 4'd10;
  localparam STATE_EXEC_SETUP  = 4'd11;
  localparam STATE_EXEC_COMMIT = 4'd12;
  localparam STATE_RESP_CRC    = 4'd13;
  localparam STATE_SEND_WAIT = 4'd14;

  // Exception Codes (need to check modbus spec for accuracy)
  localparam EXECPTION_ILLEGAL_FC = 8'h01;

	// FSM
	reg [3:0] fsm_state;						// Current state of the machine
	reg [7:0] fsm_frame_read_addr;	// Address to read frame from
	wire [7:0] fsm_frame_read_data;	// Data of the frame from address fsm_frame_read_addr
	reg [15:0] fsm_crc_in;					// Frame CRC word placeholder
	reg [7:0] fsm_fc;              	// latched function code
	reg [7:0] fsm_field_buf [0:3];  // bytes 2-5 of the request
	reg [1:0] fsm_field_idx;        // 0-3, index into fsm_field_buf

	// Response creation
	reg [7:0] resp_buf [0:255];
	reg [7:0] resp_len;
	reg [7:0] resp_crc_idx;
	reg [15:0] resp_crc;
	reg [7:0] resp_send_idx;

	// UART RX
	wire [7:0] uart_recv_data;
	wire uart_recv_data_valid;
	wire uart_recv_frame_error;
	// CLK_FREQ and BAUD_RATE must be firmware settable
	uart_rx #(
		.CLK_FREQ(CLK_FREQ),
		.BAUD_RATE(BAUD_RATE)
	) uart_recv (
		.clk(clk),
		.rstb(rstb),
		.ser_rx(ser_rx),
		.data(uart_recv_data),
		.data_valid(uart_recv_data_valid),
		.frame_error(uart_recv_frame_error)
	);

	// UART TX
	reg [7:0] uart_send_data;
	reg uart_send_tx_valid;
	wire uart_send_tx_ready;
	// CLK_FREQ and BAUD_RATE must be firmware settable
	uart_tx #(
		.CLK_FREQ(CLK_FREQ),
		.BAUD_RATE(BAUD_RATE)
	) uart_send (
		.clk(clk),										// Master clk
		.rstb(rstb),									// Active low reset pin
		.data(uart_send_data),						// Data input bus to uart tx module
		.tx_valid(uart_send_tx_valid),							// Flag to signal data ready to send (must be high for 1 cycle to start)
		.tx_ready(uart_send_tx_ready),							// Low=>module is transmitting, dont change data bus
		.ser_tx_out(ser_tx)							// Serial output line
	);

	// CRC16
	reg crc_src;
	reg [15:0] crcgen_crc_reg;
	wire [7:0] crcgen_data_in = crc_src ?
	resp_buf[resp_crc_idx] : fsm_frame_read_data;
	wire [15:0] crcgen_crc_out;
	crc16 #(
    .POLY  (16'hA001),
    .INIT  (16'hFFFF),
    .REFIN (0),
    .REFOUT(0),
    .XOROUT(16'h0000)
  ) crcgen (
    .crc_in (crcgen_crc_reg),
    .data_in(crcgen_data_in),
    .crc_out(crcgen_crc_out)
  );

  // Frame detector
  reg frmdet_frame_ack;
  wire [7:0] frmdet_frame_len;
  wire frmdet_frame_done;
  // CLK_FREQ and BAUD_RATE must be firmware settable
  frame_detector #(
  	.CLK_FREQ(CLK_FREQ),
   	.BAUD_RATE(BAUD_RATE)
  ) frmdet (
  	// SYSTEM
 		.clk(clk),
   	.rstb(rstb),

    // UART RX
    .byte_in(uart_recv_data),
    .byte_valid(uart_recv_data_valid),

    // FSM
    .frame_ack(frmdet_frame_ack),
    .frame_len(frmdet_frame_len),
    .frame_done(frmdet_frame_done),
    .read_addr(fsm_frame_read_addr),
    .read_data(fsm_frame_read_data)
  );

  // Register bank
  reg [1:0]	rb_regtype;
  reg [15:0] rb_addr;
  reg rb_wen;
  reg [15:0] rb_wdata;
  wire [15:0] rb_rdata;
  wire rb_addr_valid;
  // Counts need to be firmware configurable
  register_bank #(
  	.COIL_COUNT(64),
   	.DISCRETE_COUNT(64),
   	.HOLDING_COUNT(64),
   	.INPUT_COUNT(64)
  ) regbank (
  	.clk(clk),
   	.rstb(rstb),
    .regtype(rb_regtype),
    .addr(rb_addr),
    .wen(rb_wen),
    .wdata(rb_wdata),
    .rdata(rb_rdata),
    .addr_valid(rb_addr_valid)
  );

  always @(posedge clk or negedge rstb) begin

  	if (!rstb) begin
   		fsm_state <= STATE_IDLE;
     	crc_src <= 0;
   	end else begin

   		frmdet_frame_ack <= 0;
  		rb_wen <= 0;
   		uart_send_tx_valid <= 0;

   		case (fsm_state)
   			STATE_IDLE: begin
	     		frmdet_frame_ack <= 0;
	     		if (frmdet_frame_done) begin
	       		fsm_frame_read_addr <= 0;
	      		fsm_state <= STATE_CHECK_ADDR;
	       	end
     		end

      	STATE_CHECK_ADDR: begin
      		if (
       			fsm_frame_read_data == slave_addr ||
       			fsm_frame_read_data == DEFAULT_SLAVE_ADDR)
       		begin
       			fsm_state <= STATE_CRC_LOOP;
         		fsm_frame_read_addr <= 0;
          	crcgen_crc_reg <= 16'hFFFF;	// INIT VALUE FOR LOOP
        	end else begin	// Message not for this, do nothing
        		fsm_state <= STATE_DONE;
        	end
      	end

      	STATE_CRC_LOOP: begin
      		crcgen_crc_reg <= crcgen_crc_out;
       		if (fsm_frame_read_addr == frmdet_frame_len - 3) begin
        		fsm_frame_read_addr <= frmdet_frame_len - 2;
         		fsm_state <= STATE_GET_CRC_LO;
        	end else begin
        		fsm_frame_read_addr <= fsm_frame_read_addr + 1;
        	end
      	end

      	STATE_GET_CRC_LO: begin
        	fsm_crc_in[7:0]     <= fsm_frame_read_data;
        	fsm_frame_read_addr <= frmdet_frame_len - 1;
        	fsm_state           <= STATE_GET_CRC_HI;
      	end

      	STATE_GET_CRC_HI: begin
        	fsm_crc_in[15:8] <= fsm_frame_read_data;
        	fsm_state        <= STATE_CHECK_CRC;
      	end

      	STATE_CHECK_CRC: begin
      		if (crcgen_crc_reg == fsm_crc_in) begin
       			fsm_frame_read_addr <= 1;	// Function code at pos 1
         		fsm_state <= STATE_DECODE_FC;
      		end else begin	// CRC mismatch, MODBUS drops frame silently
       			fsm_state <= STATE_DONE;
        	end
      	end

      	STATE_DECODE_FC: begin
      		fsm_fc <= fsm_frame_read_data;

       		case (fsm_frame_read_data)
        		// Only FC03 and FC06 for now, later expand
       			8'h03, 8'h06: begin
         			fsm_frame_read_addr <= 2;
           		fsm_field_idx <= 0;
            	fsm_state <= STATE_FETCH_FIELDS;
         		end
          	default: begin
          		// Cause exception
            	// Execute but build excepiton response
          		fsm_state <= STATE_EXECUTE;
          	end
        	endcase
      	end

      	// Needs to handle longer fields for other function codes
      	STATE_FETCH_FIELDS: begin
      		fsm_field_buf[fsm_field_idx] <= fsm_frame_read_data;
       		fsm_frame_read_addr <= fsm_frame_read_addr + 1;
        	fsm_field_idx <= fsm_field_idx + 1;
        	if (fsm_field_idx == 3) begin
        		fsm_state <= STATE_EXECUTE;
        	end
      	end

      	STATE_EXECUTE: begin
       		case (fsm_fc)
         		// 8'h06: begin

           	// end
            default: begin	// Unsupported FC
            	// Build exception response
             	resp_buf[0] <= slave_addr;
              resp_buf[1] <= fsm_fc + 8'h80;
              resp_buf[2] <= EXECPTION_ILLEGAL_FC;
              resp_len <= 3;
              crc_src <= 1;	// 1 => make crc from response buffer
              resp_crc_idx <= 0;	// Setup CRC calc byte index
              crcgen_crc_reg <= 16'hFFFF;
              fsm_state <= STATE_RESP_CRC;
            end
         	endcase
       	end

        STATE_RESP_CRC: begin
       		crcgen_crc_reg <=  crcgen_crc_out;
         	if (resp_crc_idx == resp_len - 1) begin
         		resp_buf[resp_len] <= crcgen_crc_out[7:0];
         		resp_buf[resp_len + 1] <= crcgen_crc_out[15:8];
          	resp_len <= resp_len + 2;
           	resp_send_idx <= 0;
          	fsm_state <= STATE_SEND;
          end else begin
         		resp_crc_idx <= resp_crc_idx + 1;
          end
        end

        STATE_SEND: begin

        	if (resp_send_idx == resp_len) begin
         		fsm_state <= STATE_DONE;
         	end
        	else if (uart_send_tx_ready) begin
           	uart_send_data <= resp_buf[resp_send_idx];
            uart_send_tx_valid <= 1;
            resp_send_idx <= resp_send_idx + 1;
            fsm_state <= STATE_SEND_WAIT;
         	end
        end

        STATE_SEND_WAIT: begin
       		if (uart_send_tx_ready) begin
         		fsm_state <= STATE_SEND;
         	end
        end

      	STATE_DONE: begin
      		frmdet_frame_ack <= 1;
       		fsm_state <= STATE_IDLE;
         	crc_src <= 0;
      	end

      	default: fsm_state <= STATE_IDLE;
     	endcase
    end
  end



endmodule
