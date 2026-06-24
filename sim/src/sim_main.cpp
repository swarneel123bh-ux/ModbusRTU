// sim_main.cpp
// Verilator harness for the configurable Modbus RTU card.
//
// Topology (two-DUT loopback, both are the real modbus_top):
//
//   mbpoll --PTY bytes--> driver --host_byte_in--> dut_master(mode=1)
//                                                       | ser_tx
//                                                       v  (RTL UART wire)
//                                                  dut_slave(mode=0) ser_rx
//   mbpoll <--PTY bytes-- driver <-host_byte_out-- dut_master  <--ser_tx--+
//
// The two ser lines are cross-wired in software each settle step; the
// actual serialization is done by the RTL UARTs inside each modbus_top.
// The driver only ever touches dut_master's BYTE interface.
//
// Build: see sim.mk
// Run:   ./sim   (prints the /dev/pts/N path; point mbpoll at it)
//        mbpoll -m rtu -a 17 -t 4 -r 1 -c 4 -b 100000 /dev/pts/N
//        (slave addr 0x11 = 17; FC03 read holding regs)

#include <verilated.h>
#include "Vmodbus_top.h"
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

// ----- sim params (must match the reduced TB params in the RTL build) -----
// CLK_FREQ=1_000_000, BAUD_RATE=100_000  -> 10 cyc/bit.
// Inbound frame-end heuristic: if no PTY byte arrives for this many sim
// cycles while bytes are buffered, treat the request frame as complete.
// mbpoll bursts a frame then blocks waiting for the reply, so the gap is
// "an idle process", not a tight race -- generous threshold is safe.
static const int   FRAME_GAP_CYCLES = 4000;   // ~ a few ms wall at our ratio
static const uint8_t SLAVE_ADDR     = 0x11;

// ----- globals -----
static Vmodbus_top* dut_master = nullptr;
static Vmodbus_top* dut_slave  = nullptr;
static vluint64_t   main_time  = 0;
#if VM_TRACE
static VerilatedVcdC* tfp = nullptr;
#endif

static int pty_fd = -1;   // master side of PTY; we read/write this

double sc_time_stamp() { return main_time; }

// Settle the cross-wired serial lines, then advance one clock edge.
// We eval twice per phase so a combinational ser_tx change propagates to
// the other DUT's ser_rx within the same phase.
static void settle() {
  for (int i = 0; i < 2; i++) {
    dut_master->ser_rx = dut_slave->ser_tx;
    dut_slave->ser_rx  = dut_master->ser_tx;
    dut_master->eval();
    dut_slave->eval();
  }
}

static void half(int clkval) {
  dut_master->clk = clkval;
  dut_slave->clk  = clkval;
  settle();
#if VM_TRACE
  if (tfp) tfp->dump(main_time);
#endif
  main_time++;
}

// One full clock cycle: negedge then posedge. Inputs set by caller before
// this; design state updates on the posedge inside.
static void tick() {
  half(0);
  half(1);
}

// ----- PTY setup -----
static void open_pty() {
  int slave_fd;
  char name[256];
  if (openpty(&pty_fd, &slave_fd, name, nullptr, nullptr) != 0) {
    perror("openpty");
    exit(1);
  }
  // raw mode on the slave end so mbpoll's bytes pass untouched
  struct termios tio;
  tcgetattr(slave_fd, &tio);
  cfmakeraw(&tio);
  tcsetattr(slave_fd, TCSANOW, &tio);
  // non-blocking reads on our (master) end so the clock never stalls
  int fl = fcntl(pty_fd, F_GETFL, 0);
  fcntl(pty_fd, F_SETFL, fl | O_NONBLOCK);
  printf("PTY ready: %s\n", name);
  printf("Point your master at it, e.g.:\n");
  printf("  mbpoll -m rtu -a 17 -t 4 -r 1 -c 4 -b 100000 %s\n", name);
  fflush(stdout);
}

// Non-blocking read of up to n bytes. Returns count (0 if none ready).
static int pty_read(uint8_t* buf, int n) {
  ssize_t r = read(pty_fd, buf, n);
  if (r < 0) return 0;       // EAGAIN -> nothing this cycle
  return (int)r;
}

static void pty_write_all(const uint8_t* buf, int n) {
  int off = 0;
  while (off < n) {
    ssize_t w = write(pty_fd, buf + off, n - off);
    if (w <= 0) continue;    // best-effort; PTY won't block long for a frame
    off += (int)w;
  }
}

// ----- driver state machine (PTY <-> dut_master byte interface) -----
enum DrvState { D_RECV, D_FEED, D_DONE_PULSE, D_AWAIT_RESP };

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);

  dut_master = new Vmodbus_top;
  dut_slave  = new Vmodbus_top;

#if VM_TRACE
  Verilated::traceEverOn(true);
  tfp = new VerilatedVcdC;
  // trace only the master here; add a second VCD for the slave if needed
  dut_master->trace(tfp, 99);
  tfp->open("build/sim.vcd");
#endif

  open_pty();

  // static config
  dut_master->mode       = 1;            // MASTER
  dut_slave->mode        = 0;            // SLAVE
  dut_slave->slave_addr  = SLAVE_ADDR;
  dut_master->slave_addr = SLAVE_ADDR;   // unused on master, keep defined

  // slave host-port is inert in SLAVE mode; tie its inputs off
  dut_slave->host_byte_in    = 0;
  dut_slave->host_byte_valid = 0;
  dut_slave->host_frame_done = 0;

  // master host inputs start idle
  dut_master->host_byte_in    = 0;
  dut_master->host_byte_valid = 0;
  dut_master->host_frame_done = 0;

  // NOTE on register init (roadmap item 3, deferred):
  // The holding registers live in register_bank inside dut_slave and are
  // NOT a top-level port. For first bring-up give them known values either
  // by an `initial` block in register_bank.v, or by poking the verilated
  // array here via a `verilator public` hook on that memory. Until then
  // FC03 returns whatever register_bank powers up to.

  // reset both: hold rstb low a few cycles
  dut_master->rstb = 0;
  dut_slave->rstb  = 0;
  for (int i = 0; i < 8; i++) tick();
  dut_master->rstb = 1;
  dut_slave->rstb  = 1;
  for (int i = 0; i < 8; i++) tick();

  // driver buffers
  uint8_t  req[512];      int req_len = 0; int req_idx = 0;
  uint8_t  resp[512];     int resp_len = 0;
  int      gap = 0;
  DrvState st = D_RECV;

  while (!Verilated::gotFinish()) {
    // default: deassert one-cycle controls
    dut_master->host_byte_valid = 0;
    dut_master->host_frame_done = 0;

    switch (st) {
      case D_RECV: {
        // accumulate request bytes off the PTY; infer frame-end by gap
        uint8_t tmp[256];
        int n = pty_read(tmp, sizeof(tmp));
        if (n > 0) {
          if (req_len + n <= (int)sizeof(req)) {
            memcpy(req + req_len, tmp, n);
            req_len += n;
          }
          gap = 0;
        } else if (req_len > 0) {
          if (++gap >= FRAME_GAP_CYCLES && dut_master->req_ready) {
            req_idx = 0;
            st = D_FEED;
          }
        }
        break;
      }

      case D_FEED: {
        // Master captures one byte per cycle that host_byte_valid is high
        // (true in both IDLE and RECV_HOST). host_byte_ready is status
        // only -- do NOT gate on it (IDLE grabs byte 0 with ready still 0).
        // host_frame_done must land on a SEPARATE cycle with valid low,
        // else RECV_HOST drops the last byte -> handled by D_DONE_PULSE.
        if (req_idx < req_len) {
          dut_master->host_byte_in    = req[req_idx];
          dut_master->host_byte_valid = 1;
          req_idx++;
        } else {
          st = D_DONE_PULSE;            // valid stays 0 (loop default)
        }
        break;
      }

      case D_DONE_PULSE: {
        // one-cycle host_frame_done -> master leaves RECV_HOST, starts SEND
        dut_master->host_frame_done = 1;
        resp_len = 0;
        st = D_AWAIT_RESP;
        break;
      }

      case D_AWAIT_RESP: {
        // collect the response bytes the master streams back, then flush
        // the whole frame to the PTY in one write so mbpoll sees a burst.
        if (dut_master->host_byte_out_valid) {
          if (resp_len < (int)sizeof(resp))
            resp[resp_len++] = (uint8_t)dut_master->host_byte_out;
        }
        if (dut_master->host_frame_out_done) {
          if (resp_len > 0) pty_write_all(resp, resp_len);
          req_len = 0; req_idx = 0; gap = 0;
          st = D_RECV;
        }
        // req_failed: master exhausted retries with no reply -> drop request
        if (dut_master->req_failed) {
          req_len = 0; req_idx = 0; gap = 0;
          resp_len = 0;
          st = D_RECV;
        }
        break;
      }
    }

    tick();
  }

#if VM_TRACE
  if (tfp) tfp->close();
#endif
  delete dut_master;
  delete dut_slave;
  return 0;
}
