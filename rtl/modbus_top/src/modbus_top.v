`timescale 1ns/1ps

module modbus_top #(
  parameter DEFAULT_SLAVE_ADDR = 8'h11,
  parameter CLK_FREQ           = 50_000_000,
  parameter BAUD_RATE          = 9600,
  parameter MAX_RETRIES        = 3,
  parameter TIMEOUT_CYCLES     = CLK_FREQ
)(
  input  wire        clk,
  input  wire        rstb,
  input  wire        mode,       // 0 = SLAVE, 1 = MASTER

  input  wire        ser_rx,
  output wire        ser_tx,

  // SLAVE mode
  input  wire [7:0]  slave_addr,

  // MASTER mode — host write
  input  wire [7:0]  host_byte_in,
  input  wire        host_byte_valid,
  output wire        host_byte_ready,
  input  wire        host_frame_done,

  // MASTER mode — host read
  output wire [7:0]  host_byte_out,
  output wire        host_byte_out_valid,
  output wire        host_frame_out_done,

  // MASTER mode — status
  output wire        req_ready,
  output wire        req_timeout,
  output wire        req_failed
);

  localparam SLAVE  = 1'b0;
  localparam MASTER = 1'b1;

  wire slave_ser_tx, master_ser_tx;
  wire slave_ser_rx  = (mode == SLAVE)  ? ser_rx : 1'b1;
  wire master_ser_rx = (mode == MASTER) ? ser_rx : 1'b1;

  assign ser_tx = (mode == MASTER) ? master_ser_tx : slave_ser_tx;

  modbus_slave_fsm #(
    .DEFAULT_SLAVE_ADDR(DEFAULT_SLAVE_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE)
  ) modbus_slave (
    .clk       (clk),
    .rstb      (rstb),
    .ser_rx    (slave_ser_rx),
    .slave_addr(slave_addr),
    .ser_tx    (slave_ser_tx)
  );

  modbus_master_fsm #(
    .CLK_FREQ      (CLK_FREQ),
    .BAUD_RATE     (BAUD_RATE),
    .MAX_RETRIES   (MAX_RETRIES),
    .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
  ) modbus_master (
    .clk                (clk),
    .rstb               (rstb),
    .ser_rx             (master_ser_rx),
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

endmodule
