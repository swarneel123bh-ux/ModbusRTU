#!/usr/bin/env python3
# master_tui.py — interactive Modbus RTU master over a serial port (the sim's
# PTY, or any real tty). Continuously polls registered slaves and shows a live
# table; a command line at the bottom lets you add/remove polls and write.
#
#   python3 master_tui.py /dev/ttysNNN [-b 100000]
#
# Commands (type at the bottom, Enter to run):
#   add <id> <start> <count>   poll <count> holding regs from <start> on <id>
#   rm  <id>                    stop polling slave <id>
#   w   <id> <reg> <val>        write one holding reg (FC06); val hex or dec
#   to  <ms>                    set per-request timeout (default 500 ms)
#   q                           quit
#
# Architecture: fully non-blocking.  The main loop runs at ~100 Hz, handling
# keyboard input and screen redraws every tick.  Port.try_read_frame() is
# called every tick to accumulate bytes; a complete frame is delivered once
# the inter-frame gap elapses.  Requests are sent one at a time (round-robin);
# responses are routed by slave address, so it doesn't matter if a response
# arrives "late" — it still reaches the right result slot.

import argparse
import curses
import os
import sys
import termios
import time
import tty

# ---------------- Modbus RTU framing ----------------


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def make_frame(addr, pdu):
    body = bytes([addr]) + pdu
    c = crc16(body)
    return body + bytes([c & 0xFF, (c >> 8) & 0xFF])


def pdu_read_holding(start, count):
    return bytes(
        [0x03, (start >> 8) & 0xFF, start & 0xFF, (count >> 8) & 0xFF, count & 0xFF]
    )


def pdu_write_single(reg, val):
    return bytes([0x06, (reg >> 8) & 0xFF, reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF])


# ---------------- serial port (raw tty, non-blocking) ----------------


class Port:
    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.last_rx_hex = ""
        # inter-frame gap: 3.5 char times (1 char = 11 bits)
        self.gap = max(3.5 * 11.0 / baud, 0.002)
        self._rxbuf = b""
        self._last_rx_time = 0.0
        try:
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error:
            pass

    def write(self, data):
        total = 0
        while total < len(data):
            try:
                sent = os.write(self.fd, data[total:])
                if sent > 0:
                    total += sent
                    self.tx_bytes += sent
                else:
                    time.sleep(0.001)
            except (BlockingIOError, InterruptedError):
                time.sleep(0.001)
            except OSError:
                break

    def try_read_frame(self):
        """Non-blocking: read any available bytes from the fd.  Returns a
        complete frame (bytes) once the inter-frame gap has elapsed after the
        last received byte, or None if still accumulating / nothing available."""
        try:
            chunk = os.read(self.fd, 256)
        except (BlockingIOError, OSError):
            chunk = b""

        if chunk:
            self._rxbuf += chunk
            self._last_rx_time = time.monotonic()
            self.rx_bytes += len(chunk)
            return None  # might be more bytes coming

        # no new bytes — check if gap has elapsed on buffered data
        if self._rxbuf and (time.monotonic() - self._last_rx_time) >= self.gap:
            buf = self._rxbuf
            self._rxbuf = b""
            # try to extract first CRC-valid frame (handles coalesced responses)
            for end in range(4, len(buf) + 1):
                if crc16(buf[: end - 2]) == (buf[end - 2] | (buf[end - 1] << 8)):
                    remainder = buf[end:]
                    if remainder:
                        self._rxbuf = remainder
                        self._last_rx_time = time.monotonic()
                    self.last_rx_hex = buf[:end].hex()[-32:]
                    return buf[:end]
            # no valid CRC prefix — return whole buffer (caller rejects on CRC)
            self.last_rx_hex = buf.hex()[-32:]
            return buf

        return None

    def read_frame_blocking(self, timeout_s):
        """Blocking read for one-off commands (writes). Falls back to
        polling try_read_frame."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            f = self.try_read_frame()
            if f is not None:
                return f
            time.sleep(0.001)
        return b""

    def flush(self):
        """Discard any buffered and in-flight data."""
        self._rxbuf = b""
        try:
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except termios.error:
            pass
        while True:
            try:
                if not os.read(self.fd, 4096):
                    break
            except (BlockingIOError, OSError):
                break

    def close(self):
        os.close(self.fd)


# ---------------- blocking write (FC06) ----------------

OK, TIMEOUT, CRCERR, EXC = "OK", "TIMEOUT", "CRCERR", "EXC"


def write_single(port, addr, reg, val, timeout_s):
    """Blocking FC06 write — only used for user-initiated 'w' command."""
    port.write(make_frame(addr, pdu_write_single(reg, val)))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rx = port.read_frame_blocking(deadline - time.monotonic())
        if not rx:
            return TIMEOUT
        if len(rx) < 4 or crc16(rx[:-2]) != (rx[-2] | (rx[-1] << 8)):
            continue
        if rx[0] != addr:
            continue
        if rx[1] & 0x80:
            return "EXC:0x%02X" % rx[2]
        return OK
    return TIMEOUT


# ---------------- TUI ----------------

EXC_NAMES = {
    0x01: "ILLEGAL_FUNCTION",
    0x02: "ILLEGAL_ADDRESS",
    0x03: "ILLEGAL_VALUE",
    0x04: "DEVICE_FAILURE",
}


class App:
    def __init__(self, port):
        self.port = port
        self.polls = {}  # sid -> (start, count)
        self.results = {}  # sid -> (status, regs_or_code, timestamp)
        self.timeout_s = 0.5
        self.cmd = ""
        self.msg = "ready. try:  add 17 0 4   |   w 17 0 0x1234   |   q"
        self._poll_keys = []
        self._poll_idx = 0
        self._pending_sid = None  # slave we're currently waiting on
        self._pending_deadline = 0.0

    def _rebuild_poll_keys(self):
        self._poll_keys = sorted(self.polls.keys())
        if self._poll_idx >= len(self._poll_keys):
            self._poll_idx = 0

    # ---- frame routing ----

    def _handle_frame(self, rx):
        """Validate CRC, then route the frame to the correct slave's result
        slot by address byte.  Clears _pending if it matches."""
        if len(rx) < 4:
            return
        if crc16(rx[:-2]) != (rx[-2] | (rx[-1] << 8)):
            return  # bad CRC — discard
        addr = rx[0]

        # clear pending state if this is the reply we were waiting for
        if addr == self._pending_sid:
            self._pending_sid = None

        if addr not in self.polls:
            return  # unknown slave — discard

        # exception response
        if rx[1] & 0x80:
            self.results[addr] = (EXC, rx[2], time.monotonic())
            return

        # FC03 read holding registers response
        if rx[1] == 0x03:
            _, count = self.polls[addr]
            bc = rx[2]
            if bc == 2 * count and len(rx) >= 3 + bc + 2:
                data = rx[3 : 3 + bc]
                regs = [(data[2 * i] << 8) | data[2 * i + 1] for i in range(count)]
                self.results[addr] = (OK, regs, time.monotonic())

    # ---- non-blocking poll engine ----

    def _send_next(self):
        """Send a read-holding-registers request to the next slave in the
        round-robin.  Does nothing if _poll_keys is empty."""
        if not self._poll_keys:
            return
        if self._poll_idx >= len(self._poll_keys):
            self._poll_idx = 0
        sid = self._poll_keys[self._poll_idx]
        self._poll_idx = (self._poll_idx + 1) % len(self._poll_keys)
        if sid not in self.polls:
            return
        start, count = self.polls[sid]
        self.port.write(make_frame(sid, pdu_read_holding(start, count)))
        self._pending_sid = sid
        self._pending_deadline = time.monotonic() + self.timeout_s

    def _tick_poll(self):
        """Called every main-loop tick (~10 ms).  Reads any available frame,
        checks timeout, and sends the next request when ready."""
        # 1) harvest any complete frame from the wire
        rx = self.port.try_read_frame()
        if rx:
            self._handle_frame(rx)

        # 2) check timeout on pending request
        if self._pending_sid is not None:
            if time.monotonic() >= self._pending_deadline:
                # only write TIMEOUT if this slave doesn't already have a live
                # result from a stray-routed frame
                prev = self.results.get(self._pending_sid)
                if prev is None or prev[0] != OK:
                    self.results[self._pending_sid] = (TIMEOUT, None, time.monotonic())
                self._pending_sid = None

        # 3) if nothing pending, send next request
        if self._pending_sid is None and self._poll_keys:
            self._send_next()

    # ---- commands ----

    def do_command(self, line):
        parts = line.split()
        if not parts:
            return
        op = parts[0]
        try:
            if op == "add" and len(parts) == 4:
                sid, start, count = int(parts[1], 0), int(parts[2], 0), int(parts[3], 0)
                if not (1 <= sid <= 247):
                    self.msg = "id out of range 1..247"
                    return
                self.polls[sid] = (start, count)
                self._rebuild_poll_keys()
                self.msg = "polling %d: regs %d..%d" % (sid, start, start + count - 1)
            elif op == "rm" and len(parts) == 2:
                sid = int(parts[1], 0)
                self.polls.pop(sid, None)
                self.results.pop(sid, None)
                self._rebuild_poll_keys()
                self.msg = "stopped polling %d" % sid
            elif op == "w" and len(parts) == 4:
                sid, reg, val = int(parts[1], 0), int(parts[2], 0), int(parts[3], 0)
                self._pending_sid = None  # cancel in-flight poll
                self.port.flush()  # discard its response
                st = write_single(self.port, sid, reg, val & 0xFFFF, self.timeout_s)
                self.msg = "write %d[%d]=0x%04X -> %s" % (sid, reg, val & 0xFFFF, st)
            elif op == "to" and len(parts) == 2:
                self.timeout_s = max(50, int(parts[1], 0)) / 1000.0
                self.msg = "timeout = %d ms" % int(self.timeout_s * 1000)
            elif op in ("q", "quit", "exit"):
                return "quit"
            else:
                self.msg = "?: %s" % line
        except ValueError:
            self.msg = "bad number in: %s" % line

    # ---- drawing ----

    def draw(self, scr):
        scr.erase()
        h, w = scr.getmaxyx()
        scr.addnstr(
            0, 0, " Modbus master -- live poll ".ljust(w - 1), w - 1, curses.A_REVERSE
        )
        scr.addnstr(2, 0, "  ID    START  CNT   VALUES / STATUS", w - 1, curses.A_BOLD)

        row = 3
        for sid in sorted(self.polls):
            start, count = self.polls[sid]
            st, regs, ts = self.results.get(sid, ("--", None, 0))
            if st == OK and regs is not None:
                cell = " ".join("%5d" % r for r in regs)
                attr = curses.A_NORMAL
            elif st == EXC:
                cell = "EXCEPTION 0x%02X %s" % (regs, EXC_NAMES.get(regs, "?"))
                attr = curses.A_BOLD
            elif st == "--":
                cell = "(waiting)"
                attr = curses.A_DIM
            else:
                cell = st
                attr = curses.A_BOLD
            line = "  0x%02X  %5d  %3d   %s" % (sid, start, count, cell)
            if row < h - 4:
                scr.addnstr(row, 0, line, w - 1, attr)
                row += 1

        try:
            pend = "0x%02X" % self._pending_sid if self._pending_sid else "---"
            diag = " TX=%d  RX=%d  pend=%s  last=[%s]" % (
                self.port.tx_bytes,
                self.port.rx_bytes,
                pend,
                self.port.last_rx_hex,
            )
            scr.addnstr(h - 3, 0, diag.ljust(w - 1), w - 1, curses.A_DIM)
            scr.addnstr(h - 2, 0, (" " + self.msg).ljust(w - 1), w - 1, curses.A_DIM)
            scr.addnstr(
                h - 1, 0, ("> " + self.cmd).ljust(w - 1), w - 1, curses.A_REVERSE
            )
        except curses.error:
            pass
        scr.move(h - 1, min(2 + len(self.cmd), w - 1))

    # ---- main loop ----

    def run(self, scr):
        curses.curs_set(1)
        scr.nodelay(True)
        while True:
            self._tick_poll()

            try:
                ch = scr.getch()
            except curses.error:
                ch = -1
            if ch != -1:
                if ch in (curses.KEY_ENTER, 10, 13):
                    if self.do_command(self.cmd.strip()) == "quit":
                        break
                    self.cmd = ""
                elif ch in (curses.KEY_BACKSPACE, 127, 8):
                    self.cmd = self.cmd[:-1]
                elif 32 <= ch < 127:
                    self.cmd += chr(ch)

            self.draw(scr)
            scr.refresh()
            time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser(description="Interactive Modbus RTU master TUI.")
    ap.add_argument("port", help="serial device, e.g. /dev/ttys003")
    ap.add_argument("-b", "--baud", type=int, default=100000)
    args = ap.parse_args()

    try:
        port = Port(args.port, args.baud)
    except OSError as e:
        sys.stderr.write("cannot open %s: %s\n" % (args.port, e))
        sys.exit(1)

    app = App(port)
    try:
        curses.wrapper(app.run)
    finally:
        port.close()


if __name__ == "__main__":
    main()
