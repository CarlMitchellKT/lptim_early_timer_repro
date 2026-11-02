#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial>=3.5"]
# ///
"""Host side of the STM32 LPTIM early-timer reproducer (B-G474E-DPOW1).

Streams bursts of back-to-back characters into the board's ST-LINK virtual COM
port (USART3, 9600 8N1 -> one RX ISR per ~1.04 ms). The firmware restarts a
one-shot k_timer from every RX ISR and cross-checks kernel time against
DWT->CYCCNT, which is independent of the LPTIM/LSI tick source. It reports:

  EVT early  the timer expired far sooner (real time, per DWT) than the
             k_timer_start() contract allows
  EVT jump   kernel time (k_cycle_get_32) advanced > 500 us more than DWT
             between two consecutive RX ISRs

The test sends 'S' before and after the traffic to read the board's counters
(STAT line). The firmware's event lines go out at 9600 baud (~70 ms each), so a
burst of anomalies is drained slowly; the counters are authoritative and the
EVT lines are samples.

Exit status: 0 = no anomaly seen, 1 = bug reproduced, 2 = setup problem (no
port, no firmware, board silent/rebooted, traffic not seen by the board).

    uv run --script test_lptim_early_timer.py [--port /dev/ttyACM0] [--duration 30]
"""

import argparse
import glob
import queue
import random
import re
import subprocess
import sys
import threading
import time

import serial

BAUD = 9600
# frame -> (parity, stop bits, bits on the wire per character incl. start bit)
FRAMES = {
    "8N1": (serial.PARITY_NONE, serial.STOPBITS_ONE, 10),
    "8E2": (serial.PARITY_EVEN, serial.STOPBITS_TWO, 12),
}
STLINK_GLOB = "/dev/serial/by-id/usb-STMicroelectronics_STLINK-*"
KV = re.compile(r"(\w+)=(\S+)")
COUNTERS = ("rx", "starts", "fires", "early", "jump", "drop", "rxerr")


def find_port() -> str:
    ports = sorted(glob.glob(STLINK_GLOB))
    if not ports:
        sys.exit(f"error: no ST-LINK VCP found ({STLINK_GLOB}); use --port")
    return ports[0]


def parse(line: str):
    kind, _, rest = line.partition(" ")
    fields = {k: int(v, 0) if re.fullmatch(r"(0x[0-9a-fA-F]+|\d+)", v) else v
              for k, v in KV.findall(rest)}
    if kind == "EVT":
        return f"EVT_{rest.split()[0]}", fields
    return kind, fields


class Board:
    def __init__(self, port: str, frame: str):
        parity, stop, _ = FRAMES[frame]
        self.ser = serial.Serial(port, BAUD, parity=parity, stopbits=stop, timeout=0.05)
        self.q: queue.Queue = queue.Queue()
        self.events: list = []
        self.reboots = 0
        self.ready: dict = {}
        self._stop = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                buf += self.ser.read(256)
            except serial.SerialException:
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self.q.put(line)

    def pump(self, timeout: float, until: str | None = None, quiet: bool = False):
        """Collect lines for up to `timeout` s; stop early on a `until` line."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.q.get(timeout=0.05)
            except queue.Empty:
                continue
            kind, f = parse(line)
            if kind.startswith("EVT_"):
                self.events.append((kind, f))
                if not quiet and len(self.events) <= 12:
                    print(f"  {kind[4:].upper():5s} dwt={f['dwt_us']}us kernel={f['kern_us']}us "
                          f"CNT={f['cnt']} ARR={f['arr']} ISR={f['isr']:#x}")
            elif kind == "READY":
                self.reboots += 1
                self.ready = f
            elif kind == until:
                return f
        return None

    def stat(self, timeout: float):
        self.ser.write(b"S")
        return self.pump(timeout, until="STAT")

    def close(self):
        self._stop.set()
        self.ser.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="ST-LINK VCP (default: auto-detect)")
    ap.add_argument("--duration", type=float, default=30, help="seconds of traffic")
    ap.add_argument("--min-burst", type=int, default=5)
    ap.add_argument("--max-burst", type=int, default=20)
    ap.add_argument("--min-gap-ms", type=float, default=3)
    ap.add_argument("--max-gap-ms", type=float, default=15)
    ap.add_argument("--frame", choices=FRAMES, default="8N1",
                    help="UART frame; must match the firmware build (8E2 needs "
                         "-DEXTRA_DTC_OVERLAY_FILE=boards/frame_8e2.overlay)")
    ap.add_argument("--seed", type=int, help="RNG seed (default: random)")
    ap.add_argument("--verbose", action="store_true", help="print a STAT line every ~2 s")
    ap.add_argument("--flash", metavar="ELF", help="flash ELF first")
    args = ap.parse_args()

    if args.flash:
        subprocess.run(
            ["openocd", "-f", "interface/stlink.cfg", "-c",
             "reset_config srst_only srst_nogate connect_assert_srst", "-f", "target/stm32g4x.cfg",
             "-c", f"program {args.flash} verify reset exit"], check=True, capture_output=True)
        time.sleep(1)

    port = args.port or find_port()
    seed = args.seed if args.seed is not None else random.randrange(1 << 32)
    rng = random.Random(seed)
    board = Board(port, args.frame)
    char_time_s = FRAMES[args.frame][2] / BAUD

    time.sleep(0.5)
    board.pump(0.2, quiet=True)
    base = board.stat(3)
    if base is None:
        print(f"FAIL(setup): no STAT from firmware on {port}; flashed? right port?")
        return 2
    board.events.clear()
    board.reboots = 0

    print(f"port={port} frame={args.frame} seed={seed} duration={args.duration}s "
          f"burst={args.min_burst}-{args.max_burst} gap={args.min_gap_ms}-{args.max_gap_ms}ms")
    bursts = chars = 0
    t_start = time.monotonic()
    next_report = t_start + 2
    while time.monotonic() - t_start < args.duration:
        n = rng.randint(args.min_burst, args.max_burst)
        board.ser.write(b"U" * n)  # one write: the bridge clocks it out back-to-back
        bursts += 1
        chars += n
        time.sleep(n * char_time_s + rng.uniform(args.min_gap_ms, args.max_gap_ms) / 1000)
        board.pump(0, quiet=False)
        if args.verbose and time.monotonic() >= next_report:
            next_report += 2
            print(f"  t={time.monotonic() - t_start:5.1f}s sent={chars} chars, "
                  f"events seen so far={len(board.events)}")

    time.sleep(0.3)  # last one-shot expires
    final = board.stat(30)  # event printing is slow: allow it to drain first
    board.close()
    if final is None:
        print("FAIL(setup): no final STAT from firmware (board stopped answering)")
        return 2
    if board.reboots:
        print(f"FAIL(setup): board rebooted {board.reboots}x during the test")
        return 2

    d = {k: final[k] - base[k] for k in COUNTERS}
    print(f"sent {bursts} bursts / {chars} chars; board saw rx={d['rx']} "
          f"restarts={d['starts']} expiries={d['fires']} uart_errors={d['rxerr']}")
    if d["rx"] < chars * 0.99:
        print(f"FAIL(setup): board saw only {d['rx']}/{chars} chars")
        return 2
    def rate(n, per):
        return f"1 in {per // n}" if n else "none"
    print(f"early expiries   : {d['early']:5d}  ({rate(d['early'], d['fires'])} expiries)")
    print(f"kernel time jumps: {d['jump']:5d}  ({rate(d['jump'], d['starts'])} timer restarts)")
    if d["early"] or d["jump"]:
        print("RESULT: BUG REPRODUCED")
        return 1
    print("RESULT: no anomaly observed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
