// sim_tb_main.cpp
// Verilator harness for sim_tb (the card: master + slave, serial bus internal).
// Responsibilities: drive clk, own the PTY, bridge PTY bytes <-> the master
// host interface. No serial wiring here — that lives in sim_tb.v.
//
//   mbpoll <--PTY--> [driver FSM] <--host_byte_*--> dut_master --(serial)--> dut_slave
//
// Build (top module = sim_tb):
//   verilator --cc --exe --build --top-module sim_tb <-y dirs> sim_tb.v sim_tb_main.cpp
// Run: ./Vsim_tb  -> prints /dev/pts/N; point mbpoll at it.

#include <verilated.h>
#include "Vsim_tb.h"
#include "card.h"
#if VM_TRACE
#include <verilated_vcd_c.h>
#endif

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#if defined(__APPLE__) || defined(__FreeBSD__)
#include <util.h>      // openpty() on macOS / BSD
#else
#include <pty.h>       // openpty() on Linux (glibc)
#endif
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// Inbound frame-end heuristic now lives in master_driver.c.

static Vsim_tb*   dut       = nullptr;
static vluint64_t main_time = 0;
static int        pty_fd    = -1;
#if VM_TRACE
static VerilatedVcdC* tfp = nullptr;
#endif

double sc_time_stamp() { return main_time; }

// ---- card.h HAL: the only surface master_driver.c sees ----
extern "C" {

int     card_req_ready(void)           { return dut->req_ready; }
int     card_req_failed(void)          { return dut->req_failed; }
int     card_host_byte_out_valid(void) { return dut->host_byte_out_valid; }
uint8_t card_host_byte_out(void)       { return (uint8_t)dut->host_byte_out; }
int     card_host_frame_out_done(void) { return dut->host_frame_out_done; }

void card_host_send_byte(uint8_t b) {
  dut->host_byte_in    = b;
  dut->host_byte_valid = 1;
}

void card_host_frame_done(void) {
  dut->host_frame_done = 1;
}

int card_pty_read(uint8_t* buf, int n) {
  ssize_t r = read(pty_fd, buf, n);
  return r < 0 ? 0 : (int)r;
}

void card_pty_write(const uint8_t* buf, int n) {
  int off = 0;
  while (off < n) {
    ssize_t w = write(pty_fd, buf + off, n - off);
    if (w <= 0) continue;
    off += (int)w;
  }
}

}  // extern "C"

// One clock phase. Single eval(): sim_tb is one netlist, Verilator settles
// all internal (serial) combinational logic itself.
static void half(int clkval) {
  dut->clk = clkval;
  dut->eval();
#if VM_TRACE
  if (tfp) tfp->dump(main_time);
#endif
  main_time++;
}

static void tick() {
  half(0);
  half(1);
}

static void open_pty() {
  int slave_fd;
  char name[256];
  if (openpty(&pty_fd, &slave_fd, name, nullptr, nullptr) != 0) {
    perror("openpty");
    exit(1);
  }
  struct termios tio;
  tcgetattr(slave_fd, &tio);
  cfmakeraw(&tio);
  tcsetattr(slave_fd, TCSANOW, &tio);
  int fl = fcntl(pty_fd, F_GETFL, 0);
  fcntl(pty_fd, F_SETFL, fl | O_NONBLOCK);
  printf("PTY ready: %s\n", name);
  printf("Point mbpoll at it, e.g.:\n");
  printf("  mbpoll -m rtu -a 17 -r 1 -b 100000 %s 4660      (FC06 write)\n", name);
  printf("  mbpoll -1 -m rtu -a 17 -r 1 -c 1 -b 100000 %s   (FC03 read)\n", name);
  fflush(stdout);
}

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);

  dut = new Vsim_tb;

#if VM_TRACE
  Verilated::traceEverOn(true);
  tfp = new VerilatedVcdC;
  dut->trace(tfp, 99);
  tfp->open("build/sim_tb.vcd");
#endif

  open_pty();

  // host inputs idle
  dut->host_byte_in    = 0;
  dut->host_byte_valid = 0;
  dut->host_frame_done = 0;

  // reset, then idle the line so both frame_detectors sync on silence.
  dut->rstb = 0;
  for (int i = 0; i < 8; i++) tick();
  dut->rstb = 1;
  for (int i = 0; i < 500; i++) tick();

  driver_init();

  while (!Verilated::gotFinish()) {
    // one-cycle controls default low each cycle; driver re-asserts as needed
    dut->host_byte_valid = 0;
    dut->host_frame_done = 0;

    driver_step();
    tick();
  }

#if VM_TRACE
  if (tfp) tfp->close();
#endif
  delete dut;
  return 0;
}
