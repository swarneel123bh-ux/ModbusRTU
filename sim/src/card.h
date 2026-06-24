// card.h — contract between the Verilator harness (sim_tb_main.cpp) and the
// pure-C master driver (master_driver.c).
//
//   card_*    : implemented by the harness, called by the driver.
//               (read/write the master host interface; read/write the PTY)
//   driver_*  : implemented by the driver, called by the harness.
//
// The driver includes only this header — it never sees Verilator or sim_tb.

#ifndef CARD_H
#define CARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- master host interface: status (driver reads) ----
int     card_req_ready(void);
int     card_req_failed(void);
int     card_host_byte_out_valid(void);
uint8_t card_host_byte_out(void);
int     card_host_frame_out_done(void);

// ---- master host interface: control (driver drives, this cycle only) ----
void    card_host_send_byte(uint8_t b);   // host_byte_in = b, host_byte_valid = 1
void    card_host_frame_done(void);        // host_frame_done = 1

// ---- PTY (driver reads/writes application bytes) ----
int     card_pty_read(uint8_t* buf, int n);        // nonblocking; returns count
void    card_pty_write(const uint8_t* buf, int n); // blocking best-effort

// ---- driver entry points (harness calls) ----
void    driver_init(void);    // once, after reset
void    driver_step(void);    // once per clock cycle, before tick()

#ifdef __cplusplus
}
#endif

#endif  // CARD_H
