#!/usr/bin/env python3
"""Host-side integrity + throughput test for the echo firmware.

The STM32F103 + W6100 firmware runs two echo servers (see examples/echo/src/
main.rs): a **TCP** echo on 192.168.10.10:5555 and a **UDP** echo on :5556. This
tool exercises either or both, verifying every echoed byte / datagram matches
what was sent while measuring round-trip throughput.

TCP (byte stream)
    Correctness = "received stream is identical to sent stream". Send and
    receive run on separate threads (the device only has 512-byte rx/tx buffers,
    so a single-threaded send-then-read loop would deadlock once those fill).
    Both threads index into one shared, immutable random payload by absolute
    byte offset, so the receiver knows exactly what each echoed byte must be —
    catching corruption, reordering, loss, and duplication.

UDP (datagrams)
    Correctness = "each echoed datagram is identical to the one sent, boundaries
    intact". Every datagram carries a 4-byte big-endian sequence number followed
    by a deterministic body derived from that sequence, so an echo can be matched
    and fully validated regardless of reordering/loss. UDP is lossy by design, so
    a bounded in-flight window paces the sender to the device's echo rate and
    loss is reported (and only fails past --udp-max-loss); any *corrupted* or
    mis-sized datagram fails immediately. The device UDP ring caps the echoable
    payload at 504 bytes, so --udp-size is clamped to that.

Usage:
    ./echo_test.py                       # 10s TCP+UDP test against the default target
    ./echo_test.py --proto tcp
    ./echo_test.py --proto udp --seconds 30
    ./echo_test.py --bytes 1048576       # send exactly 1 MiB (per proto) then stop
    ./echo_test.py --host 192.168.10.10 --port 5555 --udp-port 5556
    ./echo_test.py --proto udp --udp-size 504 --udp-window 16
"""

import argparse
import os
import socket
import sys
import threading
import time

DEFAULT_HOST = "192.168.10.10"
DEFAULT_TCP_PORT = 5555
DEFAULT_UDP_PORT = 5556

# Size of the reference payload both threads index into. Larger than any device
# buffer so a stuck/looping pointer shows up as a mismatch rather than aliasing.
PAYLOAD_LEN = 64 * 1024

# The device's UDP ring is 512 B and prepends an 8 B per-datagram frame header,
# so the largest datagram it can echo back is 504 B. Bigger ones are dropped by
# the firmware (see udp_socket.rs), so we never send them.
UDP_MAX_PAYLOAD = 504


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


# ---------------------------------------------------------------------------
# TCP: byte-stream echo
# ---------------------------------------------------------------------------


class TcpStats:
    def __init__(self):
        self.sent = 0
        self.recv = 0
        self.error = None          # first integrity/socket error, if any
        self.stop = threading.Event()
        self.start = None
        self.end = None


def tcp_sender(sock, payload, limit, chunk, stats):
    """Stream `payload` cyclically until `limit` bytes are sent (or forever)."""
    L = len(payload)
    off = 0
    try:
        while not stats.stop.is_set():
            if limit is not None and stats.sent >= limit:
                break
            end = off + chunk
            buf = payload[off:end]
            if limit is not None:
                remaining = limit - stats.sent
                if remaining < len(buf):
                    buf = buf[:remaining]
            sock.sendall(buf)
            stats.sent += len(buf)
            off = (off + len(buf)) % L
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


def tcp_receiver(sock, payload, expected_total, stats):
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


def run_tcp(a, payload):
    """Run the TCP echo test. Returns True on pass."""
    print(f"\n=== TCP {a.host}:{a.port}  (chunk {a.chunk} B) ===", flush=True)
    stats = TcpStats()
    limit = a.bytes  # None => time-bounded

    print(f"Connecting to {a.host}:{a.port} ...", flush=True)
    try:
        sock = socket.create_connection((a.host, a.port), timeout=a.connect_timeout)
    except OSError as e:
        print(f"ERROR: could not connect: {e}", file=sys.stderr)
        return False
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("Connected. Streaming...", flush=True)

    stats.start = time.monotonic()
    rx = threading.Thread(target=tcp_receiver, args=(sock, payload, limit, stats),
                          daemon=True)
    tx = threading.Thread(target=tcp_sender, args=(sock, payload, limit, a.chunk, stats),
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
    print("  --- TCP results ---")
    print(f"  duration:   {elapsed:.3f} s")
    print(f"  sent:       {human(stats.sent)} ({stats.sent} bytes)")
    print(f"  echoed:     {human(stats.recv)} ({stats.recv} bytes)")
    print(f"  throughput: {rate(stats.recv, elapsed)}  (echoed, round-trip)")

    if stats.error:
        print(f"  FAIL: {stats.error}", file=sys.stderr)
        return False
    if limit is not None and stats.recv != limit:
        print(f"  FAIL: expected {limit} echoed bytes, got {stats.recv}",
              file=sys.stderr)
        return False
    if stats.recv == 0:
        print("  FAIL: no data echoed back", file=sys.stderr)
        return False
    print("  PASS: all echoed bytes matched.")
    return True


# ---------------------------------------------------------------------------
# UDP: datagram echo
# ---------------------------------------------------------------------------


def udp_body_start(seq, L, n_body):
    """A deterministic, well-spread offset into the reference payload for `seq`.

    Both sender and receiver compute this from the datagram's sequence number,
    so the receiver can reconstruct and verify the exact expected body of any
    echoed datagram without shared per-datagram state. (Knuth multiplicative
    hash, bounded so the `n_body`-length slice never wraps the buffer end.)"""
    return (seq * 2654435761) % (L - n_body)


class UdpStats:
    def __init__(self):
        self.sent = 0              # payload bytes sent
        self.recv = 0              # payload bytes echoed back (unique datagrams)
        self.sent_count = 0        # datagrams sent
        self.recv_count = 0        # unique datagrams echoed back
        self.dup = 0              # duplicate echoes seen
        self.forfeited = 0         # window credits released for presumed-lost dgrams
        self.seen = set()          # sequence numbers already echoed back
        self.error = None
        self.stop = threading.Event()          # hard stop, both threads
        self.stop_sending = threading.Event()  # time's up: stop sending, keep draining
        self.sending_done = threading.Event()  # sender has exited
        self.start = None
        self.end = None


def udp_sender(sock, payload, limit_bytes, size, window, stall, stats):
    """Send sequence-tagged datagrams, bounded to `window` in flight."""
    L = len(payload)
    n_body = size - 4
    seq = 0
    try:
        while not stats.stop.is_set() and not stats.stop_sending.is_set():
            if limit_bytes is not None and stats.sent >= limit_bytes:
                break

            # Bounded in-flight window paces us to the device's echo rate so we
            # don't overrun its buffers. A datagram that is never echoed (real
            # UDP loss) would otherwise stall us forever, so after `stall`
            # seconds without progress we forfeit a credit and move on.
            wait_start = None
            last_seen = stats.recv_count
            while (stats.sent_count - stats.recv_count - stats.forfeited) >= window:
                if stats.stop.is_set() or stats.stop_sending.is_set():
                    return
                now = time.monotonic()
                if stats.recv_count != last_seen:
                    last_seen = stats.recv_count
                    wait_start = now
                elif wait_start is None:
                    wait_start = now
                elif now - wait_start > stall:
                    stats.forfeited += 1  # presume one outstanding datagram lost
                    break
                time.sleep(0.0005)

            start = udp_body_start(seq, L, n_body)
            dgram = seq.to_bytes(4, "big") + payload[start:start + n_body]
            sock.send(dgram)
            stats.sent += len(dgram)
            stats.sent_count += 1
            seq += 1
            if seq > 0xFFFFFFFE:  # keep the sequence number inside 4 bytes
                break
    except OSError as e:
        if stats.error is None:
            stats.error = f"udp send failed: {e}"
        stats.stop.set()
    finally:
        stats.sending_done.set()


def udp_receiver(sock, payload, size, stats):
    """Validate each echoed datagram against its sequence-derived expectation."""
    L = len(payload)
    n_body = size - 4
    sock.settimeout(0.5)
    while not stats.stop.is_set():
        try:
            data = sock.recv(65536)
        except socket.timeout:
            if stats.sending_done.is_set():
                break  # nothing more coming
            continue
        except OSError as e:
            if stats.error is None:
                stats.error = f"udp recv failed: {e}"
            stats.stop.set()
            break

        if len(data) < 4:
            stats.error = f"udp: runt datagram ({len(data)} bytes)"
            stats.stop.set()
            break
        seq = int.from_bytes(data[:4], "big")
        start = udp_body_start(seq, L, n_body)
        expected = data[:4] + payload[start:start + n_body]
        if data != expected:
            stats.error = (
                f"udp integrity mismatch on datagram seq={seq}: "
                f"got {len(data)} bytes (expected {size}) or body corrupt"
            )
            stats.stop.set()
            break

        if seq in stats.seen:
            stats.dup += 1
        else:
            stats.seen.add(seq)
            stats.recv += len(data)
            stats.recv_count += 1


def run_udp(a, payload):
    """Run the UDP echo test. Returns True on pass."""
    size = min(max(a.udp_size, 8), UDP_MAX_PAYLOAD)
    if size != a.udp_size:
        print(f"note: --udp-size {a.udp_size} clamped to {size} "
              f"(device UDP ring caps the echoable payload at {UDP_MAX_PAYLOAD} B)")

    print(f"\n=== UDP {a.host}:{a.udp_port}  (datagram {size} B, window {a.udp_window}) ===",
          flush=True)
    stats = UdpStats()
    limit = a.bytes  # None => time-bounded

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # `connect` fixes the peer so send/recv work and stray datagrams from
        # other hosts are filtered out.
        sock.connect((a.host, a.udp_port))
    except OSError as e:
        print(f"ERROR: udp socket setup failed: {e}", file=sys.stderr)
        return False
    print("Streaming datagrams...", flush=True)

    stats.start = time.monotonic()
    rx = threading.Thread(target=udp_receiver, args=(sock, payload, size, stats),
                          daemon=True)
    tx = threading.Thread(target=udp_sender,
                          args=(sock, payload, limit, size, a.udp_window, a.udp_stall, stats),
                          daemon=True)
    rx.start()
    tx.start()

    try:
        if limit is None:
            # Time-bounded run: report progress once a second, then stop sending.
            deadline = stats.start + a.seconds
            last = stats.start
            while time.monotonic() < deadline and not stats.stop.is_set():
                time.sleep(0.25)
                now = time.monotonic()
                if now - last >= 1.0:
                    print(f"  sent {stats.sent_count} dgrams ({human(stats.sent)})  "
                          f"echoed {stats.recv_count} ({human(stats.recv)})", flush=True)
                    last = now
            stats.stop_sending.set()
            tx.join(timeout=2.0)
        else:
            # Byte-bounded run: the sender stops itself at the limit.
            tx.join()
        stats.sending_done.set()

        # Drain: let the last outstanding echoes arrive before stopping.
        drain = time.monotonic() + max(1.0, a.udp_stall * 4)
        while stats.recv_count < stats.sent_count \
                and time.monotonic() < drain and not stats.error:
            time.sleep(0.02)
        stats.stop.set()
        rx.join(timeout=2.0)
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
    lost = stats.sent_count - stats.recv_count
    loss_pct = (100.0 * lost / stats.sent_count) if stats.sent_count else 0.0
    print("  --- UDP results ---")
    print(f"  duration:   {elapsed:.3f} s")
    print(f"  sent:       {stats.sent_count} dgrams ({human(stats.sent)})")
    print(f"  echoed:     {stats.recv_count} dgrams ({human(stats.recv)})")
    if stats.dup:
        print(f"  duplicates: {stats.dup}")
    print(f"  lost:       {lost} ({loss_pct:.2f}%)")
    print(f"  throughput: {rate(stats.recv, elapsed)}  (echoed, round-trip)")

    if stats.error:
        print(f"  FAIL: {stats.error}", file=sys.stderr)
        return False
    if stats.recv_count == 0:
        print("  FAIL: no datagrams echoed back", file=sys.stderr)
        return False
    if loss_pct > a.udp_max_loss:
        print(f"  FAIL: datagram loss {loss_pct:.2f}% exceeds "
              f"--udp-max-loss {a.udp_max_loss:.2f}%", file=sys.stderr)
        return False
    print("  PASS: all echoed datagrams matched.")
    return True


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--proto", choices=["tcp", "udp", "both"], default="both",
                   help="which echo server(s) to test (default both)")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT,
                   help="TCP echo port (default 5555)")
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT,
                   help="UDP echo port (default 5556)")
    p.add_argument("--seconds", type=float, default=10.0,
                   help="test duration per proto; ignored if --bytes is given (default 10)")
    p.add_argument("--bytes", type=int, default=None,
                   help="send exactly N bytes per proto then stop (overrides --seconds)")
    p.add_argument("--chunk", type=int, default=512,
                   help="TCP send() size in bytes (default 512, the device buffer size)")
    p.add_argument("--udp-size", type=int, default=256,
                   help=f"UDP datagram size in bytes, max {UDP_MAX_PAYLOAD} (default 256)")
    p.add_argument("--udp-window", type=int, default=8,
                   help="max UDP datagrams in flight, paces the sender (default 8)")
    p.add_argument("--udp-stall", type=float, default=0.25,
                   help="seconds without an echo before a windowed datagram is "
                        "presumed lost (default 0.25)")
    p.add_argument("--udp-max-loss", type=float, default=1.0,
                   help="max tolerated UDP datagram loss %% before FAIL (default 1.0)")
    p.add_argument("--connect-timeout", type=float, default=5.0)
    a = p.parse_args()

    payload = os.urandom(PAYLOAD_LEN)

    results = []
    if a.proto in ("tcp", "both"):
        results.append(("TCP", run_tcp(a, payload)))
    if a.proto in ("udp", "both"):
        results.append(("UDP", run_udp(a, payload)))

    print("\n=== Summary ===")
    all_ok = True
    for name, ok in results:
        print(f"  {name}: {'PASS' if ok else 'FAIL'}")
        all_ok = all_ok and ok

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
