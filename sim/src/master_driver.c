// master_driver.c — pure-C master-side driver.
// Bridges PTY application bytes <-> the master host interface. Knows nothing
// about Verilator or RTL; talks only through card.h.

#include "card.h"
#include <string.h>

// No PTY byte for this many cycles (with bytes buffered) => request frame done.
// mbpoll bursts a frame then blocks for the reply, so this gap is an idle
// process, not a tight race.
#define FRAME_GAP_CYCLES 4000

enum { D_RECV, D_FEED, D_DONE_PULSE, D_AWAIT_RESP };

static int     st;
static uint8_t req[512];   static int req_len;  static int req_idx;
static uint8_t resp[512];  static int resp_len;
static int     gap;

void driver_init(void) {
  st = D_RECV;
  req_len = 0; req_idx = 0; resp_len = 0; gap = 0;
}

void driver_step(void) {
  switch (st) {
    case D_RECV: {
      // accumulate request bytes; infer frame-end by inter-byte gap
      uint8_t tmp[256];
      int n = card_pty_read(tmp, (int)sizeof(tmp));
      if (n > 0) {
        if (req_len + n <= (int)sizeof(req)) {
          memcpy(req + req_len, tmp, n);
          req_len += n;
        }
        gap = 0;
      } else if (req_len > 0) {
        if (++gap >= FRAME_GAP_CYCLES && card_req_ready()) {
          req_idx = 0;
          st = D_FEED;
        }
      }
      break;
    }

    case D_FEED: {
      // one byte per cycle while valid high; do NOT gate on host_byte_ready
      // (IDLE captures byte 0 with ready still low). frame_done lands on a
      // separate cycle with valid low -> D_DONE_PULSE.
      if (req_idx < req_len) {
        card_host_send_byte(req[req_idx]);
        req_idx++;
      } else {
        st = D_DONE_PULSE;
      }
      break;
    }

    case D_DONE_PULSE: {
      card_host_frame_done();   // one-cycle pulse, valid low
      resp_len = 0;
      st = D_AWAIT_RESP;
      break;
    }

    case D_AWAIT_RESP: {
      if (card_host_byte_out_valid()) {
        if (resp_len < (int)sizeof(resp))
          resp[resp_len++] = card_host_byte_out();
      }
      if (card_host_frame_out_done()) {
        if (resp_len > 0) card_pty_write(resp, resp_len);
        req_len = 0; req_idx = 0; gap = 0;
        st = D_RECV;
      }
      if (card_req_failed()) {   // retries exhausted, no reply
        req_len = 0; req_idx = 0; gap = 0; resp_len = 0;
        st = D_RECV;
      }
      break;
    }
  }
}
