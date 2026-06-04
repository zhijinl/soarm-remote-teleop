"""Network-backed LeRobot leader (runs on the REMOTE machine).

The physical leader lives on the local machine, which streams raw Present_Position ticks.
This turns that stream into a LeRobot leader whose `get_action()` returns the exact same
dict the real `get_action()` would, by feeding the ticks through LeRobot's own
`bus._decode_sign` + `bus._normalize` (so the leader's calibration is applied identically).

Two injection styles (see INTEGRATION.md):
  - wrap_leader_for_network(leader, "host:port")        -> wrap a constructed leader object
  - attach_network_to_leader_class(LeaderCls, "host:port") -> patch the class in place
                                                              (zero edits to the host app)

Stdlib-only; LeRobot is reached only through the wrapped/patched object's `.bus`.
"""
from __future__ import annotations

import socket
import threading
import time

from .protocol import parse_latest


class LeaderStreamReceiver:
    """Background thread holding the most recent frame from the local leader stream."""

    def __init__(self, host: str, port: int):
        self.host, self.port = host, port
        self._latest: dict[int, int] | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="leader-rx", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def latest(self) -> dict[int, int] | None:
        with self._lock:
            return None if self._latest is None else dict(self._latest)

    def wait_for_first(self, timeout: float = 15.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.latest() is not None:
                return True
            time.sleep(0.02)
        return False

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                time.sleep(0.5)
                continue
            try:
                while not self._stop.is_set():
                    chunk = sock.recv(65536)
                    if not chunk:
                        raise ConnectionResetError("stream closed")
                    buf += chunk
                    latest, buf = parse_latest(buf)
                    if latest is not None:
                        with self._lock:
                            self._latest = latest
            except OSError:
                time.sleep(0.3)
            finally:
                try:
                    sock.close()
                except OSError:
                    pass


def action_from_ticks(bus, ticks: dict[int, int]) -> dict[str, float]:
    """Raw Present_Position ticks -> LeRobot action dict, using the bus's calibration."""
    ids_values = bus._decode_sign("Present_Position", dict(ticks))
    ids_values = bus._normalize(ids_values)
    return {f"{bus._id_to_name(i)}.pos": v for i, v in ids_values.items()}


def _split(net_addr: str) -> tuple[str, int]:
    host, port = net_addr.rsplit(":", 1)
    return host, int(port)


class NetworkLeaderAdapter:
    """Wraps a real LeRobot leader; delegates everything except connect/get_action/
    is_connected/disconnect, which use the network stream + the real object's bus."""

    def __init__(self, real_leader, host: str, port: int):
        object.__setattr__(self, "_robot", real_leader)
        object.__setattr__(self, "_rx", LeaderStreamReceiver(host, port))

    def __getattr__(self, name):
        return getattr(self._robot, name)

    @property
    def is_connected(self) -> bool:
        return self._rx.alive

    @property
    def is_calibrated(self) -> bool:
        return bool(self._robot.bus.calibration)

    def connect(self, calibrate: bool = True) -> None:
        if not self._robot.bus.calibration:
            raise RuntimeError(
                "Leader calibration missing on this machine. Generate it on the local side "
                "(`soarm-local calibrate`) and place the JSON in this host's LeRobot "
                "calibration dir, keyed by the leader id."
            )
        self._rx.start()
        if not self._rx.wait_for_first():
            raise TimeoutError(
                "No leader frames received. Is the local streamer running and the tunnel up?"
            )

    def get_action(self) -> dict[str, float]:
        ticks = self._rx.latest()
        if ticks is None:
            raise RuntimeError("No leader frame available yet.")
        return action_from_ticks(self._robot.bus, ticks)

    def disconnect(self) -> None:
        self._rx.stop()


def wrap_leader_for_network(real_leader, net_addr: str) -> NetworkLeaderAdapter:
    host, port = _split(net_addr)
    return NetworkLeaderAdapter(real_leader, host, port)


def attach_network_to_leader_class(leader_cls, net_addr: str) -> None:
    """Patch a LeRobot leader class in place so every instance reads from the network.

    Works regardless of how the host app constructs the leader; just call this before the
    leader's connect()/get_action() are invoked. Idempotent.
    """
    host, port = _split(net_addr)
    if getattr(leader_cls, "_soarm_net_patched", False):
        return

    def connect(self, calibrate: bool = True):
        if not self.bus.calibration:
            raise RuntimeError(
                "Leader calibration missing on this machine (keyed by the leader id)."
            )
        self._soarm_rx = LeaderStreamReceiver(host, port)
        self._soarm_rx.start()
        if not self._soarm_rx.wait_for_first():
            raise TimeoutError(
                "No leader frames received. Is the local streamer running and the tunnel up?"
            )

    def get_action(self):
        ticks = self._soarm_rx.latest()
        if ticks is None:
            raise RuntimeError("No leader frame available yet.")
        return action_from_ticks(self.bus, ticks)

    def disconnect(self):
        rx = getattr(self, "_soarm_rx", None)
        if rx is not None:
            rx.stop()

    leader_cls.connect = connect
    leader_cls.get_action = get_action
    leader_cls.disconnect = disconnect
    leader_cls.is_connected = property(
        lambda self: getattr(self, "_soarm_rx", None) is not None and self._soarm_rx.alive
    )
    leader_cls._soarm_net_patched = True


# ----------------------------------------------------------------------------
# Standalone remote-side tooling (no LeRobot needed): transport self-test + ping.
#   python -m soarm_remote_teleop.network_leader --addr 127.0.0.1:5599
#   python -m soarm_remote_teleop.network_leader --addr 127.0.0.1:5599 --ping 100
# ----------------------------------------------------------------------------
def _selftest(addr: str, calib_path: str | None):
    import json
    from .protocol import MOTORS, PRESENT_POSITION_SIGN_BIT, decode_sign_magnitude
    host, port = _split(addr)
    rx = LeaderStreamReceiver(host, port)
    rx.start()
    if not rx.wait_for_first(10.0):
        print("No frames received within 10s (streamer/tunnel up?).")
        return 1
    cal = json.loads(open(calib_path).read()) if calib_path else None

    def norm(name, raw):
        c = cal[name]
        v = max(c["range_min"], min(c["range_max"], decode_sign_magnitude(raw, PRESENT_POSITION_SIGN_BIT)))
        frac = (v - c["range_min"]) / (c["range_max"] - c["range_min"])
        return frac * 100.0 if name == "gripper" else frac * 200.0 - 100.0

    print("Streaming decoded leader values (Ctrl-C to stop)...")
    try:
        while True:
            ticks = rx.latest()
            vals = ({n: round(norm(n, ticks[i]), 1) for n, i in MOTORS} if cal
                    else {n: ticks[i] for n, i in MOTORS})
            print("\r" + "  ".join(f"{n[:4]}:{v}" for n, v in vals.items()) + "      ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        rx.stop()
    return 0


def _ping(addr: str, count: int, interval: float):
    import struct
    host, port = _split(addr)
    s = socket.create_connection((host, port), timeout=5)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    rtts = []
    try:
        for seq in range(count):
            t0 = time.perf_counter()
            s.sendall(struct.pack("<Q", seq))
            buf = b""
            while len(buf) < 8:
                c = s.recv(8 - len(buf))
                if not c:
                    raise ConnectionError("echo server closed")
                buf += c
            rtts.append((time.perf_counter() - t0) * 1000.0)
            print(f"\rping {seq + 1}/{count}  rtt={rtts[-1]:.1f} ms   ", end="", flush=True)
            time.sleep(interval)
    finally:
        s.close()
    rtts.sort()
    n = len(rtts)
    pct = lambda q: rtts[min(n - 1, int(q * n))]  # noqa: E731
    print(f"\n--- {n} round-trips through the tunnel ---")
    print(f"RTT     min={rtts[0]:.1f}  median={pct(0.5):.1f}  p95={pct(0.95):.1f}  max={rtts[-1]:.1f} ms")
    print(f"one-way ~ median/2 = {pct(0.5) / 2:.1f} ms  (the constant lag leader->remote)")
    return 0


def main():
    import argparse
    ap = argparse.ArgumentParser(
        prog="network_leader", description="Remote-side transport self-test / latency ping")
    ap.add_argument("--addr", default="127.0.0.1:5599", help="host:port of the leader stream")
    ap.add_argument("--calib", default=None, help="leader calibration JSON -> print normalized values")
    ap.add_argument("--ping", type=int, metavar="N", default=None,
                    help="measure RTT with N round-trips (needs `soarm-local latency-server`)")
    ap.add_argument("--interval", type=float, default=0.05)
    a = ap.parse_args()
    if a.ping is not None:
        return _ping(a.addr, a.ping, a.interval)
    return _selftest(a.addr, a.calib)


if __name__ == "__main__":
    raise SystemExit(main())
