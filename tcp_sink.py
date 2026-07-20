#!/usr/bin/env python3
"""
TCP sink for the ESP32-P4 `wifispeed` console command.

Run this on the PC, then from the board's UART0 console:

    join <ssid> <password>
    wifispeed <this-pc-ip> -t 10

The board opens a TCP connection and blasts data at it; this script drains the
socket and reports what it actually received, which is the honest end-to-end
number (the board's own figure counts bytes handed to lwIP, not bytes that
arrived).

    python tcp_sink.py [--port 5001] [--bind 0.0.0.0]

Remember to allow the port through the PC firewall - a silent Windows Defender
block looks exactly like "connect: errno 113" on the board.
"""

import argparse
import socket
import time

# Seconds of silence before a connection is assumed dead and dropped. Longer
# than any gap a real run produces, short enough that a wedged sink recovers
# on its own rather than looking like a board fault on the next attempt.
IDLE_TIMEOUT = 15.0


def serve(bind: str, port: int) -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((bind, port))
    srv.listen(1)
    print(f"listening on {bind}:{port} — Ctrl-C to stop")

    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Without this, a half-open connection wedges the whole sink. If the
        # board reboots or is reflashed mid-transfer the PC never sees a FIN,
        # so recv() below blocks forever and we never get back to accept().
        # The next run then *looks* like a board-side failure: the kernel
        # completes the handshake from the listen backlog so connect() succeeds,
        # but nothing drains the socket, the window shuts, and the sender stalls
        # after exactly one receive-buffer's worth - the same byte count every
        # time. Time out instead, so a dead peer costs one idle period.
        conn.settimeout(IDLE_TIMEOUT)
        print(f"\nconnection from {addr[0]}:{addr[1]}")

        total = 0
        t0 = time.perf_counter()
        last_report = t0
        last_total = 0

        try:
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                total += len(chunk)

                now = time.perf_counter()
                if now - last_report >= 1.0:
                    delta = total - last_total
                    mbits = (delta * 8) / ((now - last_report) * 1e6)
                    print(f"  {mbits:7.2f} Mbit/s   ({total / 1e6:.1f} MB total)")
                    last_report = now
                    last_total = total
        except socket.timeout:
            print(f"  no data for {IDLE_TIMEOUT:.0f}s — dropping this connection "
                  f"and listening again")
        except ConnectionResetError:
            print("  peer reset the connection")
        finally:
            conn.close()

        elapsed = time.perf_counter() - t0
        if elapsed > 0 and total > 0:
            mbits = (total * 8) / (elapsed * 1e6)
            print(f"done: {total} bytes in {elapsed:.2f} s = "
                  f"{mbits:.2f} Mbit/s ({mbits / 8:.2f} MB/s)")
        else:
            print("done: nothing received")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5001, help="TCP port (default 5001)")
    ap.add_argument("--bind", default="0.0.0.0", help="bind address (default all)")
    args = ap.parse_args()

    try:
        serve(args.bind, args.port)
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
