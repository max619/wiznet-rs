#!/usr/bin/env python3
"""Host-side integrity + throughput test for the echo firmware.

The STM32F103 + W6100 firmware runs a TCP echo server on 192.168.10.10:5555
(see examples/echo/src/main.rs). This tool connects, streams data, and
verifies every echoed byte matches what was sent, in order, while measuring
round-trip throughput.

Because the echo is a pure byte mirror, correctness = "received stream is
identical to sent stream". We drive send and receive on separate threads (the
device only has 512-byte rx/tx buffers, so a single-threaded send-then-read
loop would deadlock once those fill). Both threads index into one shared,
immutable random payload by absolute byte offset, so the receiver knows exactly
what each echoed byte must be — this catches corruption, reordering, loss, and
duplication, not just "some bytes came back".

Usage:
    ./echo_test.py                       # 10s test against the default target
    ./echo_test.py --seconds 30
    ./echo_test.py --bytes 1048576       # send exactly 1 MiB then stop
    ./echo_test.py --host 192.168.10.10 --port 5555 --chunk 512
"""

import argparse
import os
import socket
import sys
import threading
import time

DEFAULT_HOST = "192.168.10.10"
DEFAULT_PORT = 5555

# Size of the reference payload both threads index into. Larger than any device
# buffer so a stuck/looping pointer shows up as a mismatch rather than aliasing.
PAYLOAD_LEN = 64 * 1024


def human(n):
    """Bytes -> human-readable (decimal) string."""
    for unit in ("B", "KB", "MB", "GB"):
        if abs(n) < 1000.0:
            return f"{n:.2f} {unit}" if unit != "B" else f"{int(n)} {unit}"
        n /= 1000.0
    return f"{n:.2f} TB"


def rate(nbytes, seconds):
    if seconds <= 0:
        return "n/a"
    bps = nbytes / seconds
    return f"{human(bps)}/s  ({bps * 8 / 1e6:.3f} Mbit/s)"


class Stats:
    def __init__(self):
        self.sent = 0
        self.recv = 0
        self.error = None          # first integrity/socket error, if any
        self.stop = threading.Event()
        self.start = None
        self.end = None


def sender(sock, payload, limit, stats):
    """Stream `payload` cyclically until `limit` bytes are sent (or forever)."""
    L = len(payload)
    off = 0
    try:
        while not stats.stop.is_set():
            if limit is not None and stats.sent >= limit:
                break
            end = off + args_chunk
            chunk = payload[off:end]
            if limit is not None:
                remaining = limit - stats.sent
                if remaining < len(chunk):
                    chunk = chunk[:remaining]
            sock.sendall(chunk)
            stats.sent += len(chunk)
            off = (off + len(chunk)) % L
    except OSError as e:
        if stats.error is None:
            stats.error = f"send failed: {e}"
        stats.stop.set()
    finally:
        # Signal EOF to the peer so a byte-count-limited run can drain and close
        # cleanly. Harmless for an open-ended run since we stop() right after.
        try:
            sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def receiver(sock, payload, expected_total, stats):
    """Verify each echoed byte against payload[absolute_offset % L]."""
    L = len(payload)
    try:
        while not stats.stop.is_set():
            if expected_total is not None and stats.recv >= expected_total:
                break
            buf = sock.recv(65536)
            if not buf:
                if expected_total is not None and stats.recv < expected_total:
                    stats.error = (
                        f"connection closed early: got {stats.recv} of "
                        f"{expected_total} echoed bytes"
                    )
                break
            base = stats.recv
            for i, got in enumerate(buf):
                exp = payload[(base + i) % L]
                if got != exp:
                    off = base + i
                    stats.error = (
                        f"integrity mismatch at echoed byte {off}: "
                        f"got 0x{got:02x}, expected 0x{exp:02x}"
                    )
                    stats.stop.set()
                    return
            stats.recv += len(buf)
    except OSError as e:
        if stats.error is None:
            stats.error = f"recv failed: {e}"
        stats.stop.set()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--seconds", type=float, default=10.0,
                   help="test duration; ignored if --bytes is given (default 10)")
    p.add_argument("--bytes", type=int, default=None,
                   help="send exactly N bytes then stop (overrides --seconds)")
    p.add_argument("--chunk", type=int, default=512,
                   help="send() size in bytes (default 512, the device buffer size)")
    p.add_argument("--connect-timeout", type=float, default=5.0)
    a = p.parse_args()

    global args_chunk
    args_chunk = a.chunk

    payload = os.urandom(PAYLOAD_LEN)
    stats = Stats()
    limit = a.bytes  # None => time-bounded

    print(f"Connecting to {a.host}:{a.port} ...", flush=True)
    try:
        sock = socket.create_connection((a.host, a.port), timeout=a.connect_timeout)
    except OSError as e:
        print(f"ERROR: could not connect: {e}", file=sys.stderr)
        return 2
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("Connected. Streaming...", flush=True)

    stats.start = time.monotonic()
    rx = threading.Thread(target=receiver, args=(sock, payload, limit, stats),
                          daemon=True)
    tx = threading.Thread(target=sender, args=(sock, payload, limit, stats),
                          daemon=True)
    rx.start()
    tx.start()

    try:
        if limit is None:
            # Time-bounded run: report progress once a second, then stop.
            deadline = stats.start + a.seconds
            last = stats.start
            while time.monotonic() < deadline and not stats.stop.is_set():
                time.sleep(0.25)
                now = time.monotonic()
                if now - last >= 1.0:
                    print(f"  sent {human(stats.sent)}  echoed {human(stats.recv)}",
                          flush=True)
                    last = now
            stats.stop.set()
            tx.join(timeout=2.0)
            # Give the last in-flight bytes a moment to echo back.
            drain = time.monotonic() + 1.0
            while stats.recv < stats.sent and time.monotonic() < drain \
                    and not stats.error:
                time.sleep(0.02)
        else:
            # Byte-bounded run: wait for exactly `limit` bytes to echo back.
            tx.join()
            rx.join(timeout=max(a.connect_timeout, 10.0))
    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)
        stats.stop.set()

    stats.end = time.monotonic()
    stats.stop.set()
    try:
        sock.close()
    except OSError:
        pass

    elapsed = stats.end - stats.start
    print("\n=== Results ===")
    print(f"  duration:   {elapsed:.3f} s")
    print(f"  sent:       {human(stats.sent)} ({stats.sent} bytes)")
    print(f"  echoed:     {human(stats.recv)} ({stats.recv} bytes)")
    print(f"  throughput: {rate(stats.recv, elapsed)}  (echoed, round-trip)")

    if stats.error:
        print(f"\nFAIL: {stats.error}", file=sys.stderr)
        return 1
    if limit is not None and stats.recv != limit:
        print(f"\nFAIL: expected {limit} echoed bytes, got {stats.recv}",
              file=sys.stderr)
        return 1
    if stats.recv == 0:
        print("\nFAIL: no data echoed back", file=sys.stderr)
        return 1
    print("\nPASS: all echoed bytes matched.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
