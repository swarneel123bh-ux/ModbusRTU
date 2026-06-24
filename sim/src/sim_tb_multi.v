`timescale 1ns/1ps

// sim_tb (multi-slave) — one master + three slaves on a shared serial bus.
// Same module name / port list as the single-slave sim_tb, so the existing
// harness and makefile build it unchanged:
//   make -f sim.mk simtb-run SIM_TB_V=src/sim_tb_multi.v
//
// Bus model (half-duplex, idle-high):
//   master TX  --> broadcast to every slave RX
//   slaves TX  --> wired-AND --> master RX
// Only the addressed slave drives its TX low to send; the others idle high,
// so the AND of all slave TX lines is the responding slave's output.
// This is correct ONLY if uart_tx idles HIGH when not transmitting.

module sim_tb #(
  parameter CLK_FREQ       = 1_000_000,
  parameter BAUD_RATE      = 100_000,
  parameter SLAVE0_ADDR    = 8'h11,
  parameter SLAVE1_ADDR    = 8'h12,
  parameter SLAVE2_ADDR    = 8'h13,
  parameter MAX_RETRIES    = 2,
  parameter TIMEOUT_CYCLES = (CLK_FREQ / BAUD_RATE) * 200
)(
  input  wire        clk,
  input  wire        rstb,

  // master host interface — driver writes
  input  wire [7:0]  host_byte_in,
  input  wire        host_byte_valid,
  input  wire        host_frame_done,

  // master host interface — driver reads
  output wire        host_byte_ready,
  output wire [7:0]  host_byte_out,
  output wire        host_byte_out_valid,
  output wire        host_frame_out_done,
  output wire        req_ready,
  output wire        req_timeout,
  output wire        req_failed
);

  // shared bus wires
  wire master_to_slaves;          // master TX, broadcast to all slave RX
  wire s0_tx, s1_tx, s2_tx;       // each slave's TX
  wire slaves_to_master = s0_tx & s1_tx & s2_tx;   // wired-AND return line

  // master TX param unused on slaves' side except as the broadcast source.
  // DEFAULT_SLAVE_ADDR on the master is don't-care (addressing comes from the
  // host frame); kept defined for the instance.

  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE0_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_master (
    .clk                (clk),
    .rstb               (rstb),
    .mode               (1'b1),
    .ser_rx             (slaves_to_master),
    .ser_tx             (master_to_slaves),
    .slave_addr         (SLAVE0_ADDR),
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

  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE0_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_slave0 (
    .clk(clk), .rstb(rstb), .mode(1'b0),
    .ser_rx(master_to_slaves), .ser_tx(s0_tx),
    .slave_addr(SLAVE0_ADDR),
    .host_byte_in(8'h00), .host_byte_valid(1'b0), .host_byte_ready(),
    .host_frame_done(1'b0), .host_byte_out(), .host_byte_out_valid(),
    .host_frame_out_done(), .req_ready(), .req_timeout(), .req_failed()
  );

  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE1_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_slave1 (
    .clk(clk), .rstb(rstb), .mode(1'b0),
    .ser_rx(master_to_slaves), .ser_tx(s1_tx),
    .slave_addr(SLAVE1_ADDR),
    .host_byte_in(8'h00), .host_byte_valid(1'b0), .host_byte_ready(),
    .host_frame_done(1'b0), .host_byte_out(), .host_byte_out_valid(),
    .host_frame_out_done(), .req_ready(), .req_timeout(), .req_failed()
  );

  modbus_top #(
    .DEFAULT_SLAVE_ADDR(SLAVE2_ADDR),
    .CLK_FREQ          (CLK_FREQ),
    .BAUD_RATE         (BAUD_RATE),
    .MAX_RETRIES       (MAX_RETRIES),
    .TIMEOUT_CYCLES    (TIMEOUT_CYCLES)
  ) dut_slave2 (
    .clk(clk), .rstb(rstb), .mode(1'b0),
    .ser_rx(master_to_slaves), .ser_tx(s2_tx),
    .slave_addr(SLAVE2_ADDR),
    .host_byte_in(8'h00), .host_byte_valid(1'b0), .host_byte_ready(),
    .host_frame_done(1'b0), .host_byte_out(), .host_byte_out_valid(),
    .host_frame_out_done(), .req_ready(), .req_timeout(), .req_failed()
  );

endmodule
