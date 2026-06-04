#!/usr/bin/env python3
"""Local CLI (runs where the leader arm is plugged in; Linux or macOS).

Subcommands:
  probe           - verify the arm and read positions
  calibrate       - LeRobot-format calibration (homing + ranges); writes a JSON
  stream          - read Present_Position and stream it to the remote over TCP
  latency-server  - echo server for measuring round-trip latency through the tunnel

Keeps the Feetech serial protocol local; only joint values cross the network. The remote
side (soarm_remote_teleop.network_leader) reconstructs the exact LeRobot action.

Find your serial port:  Linux -> /dev/ttyACM* or /dev/ttyUSB* ;  macOS -> /dev/cu.usbmodem*
"""
from __future__ import annotations

import argparse
import json
import select
import socket
import struct
import sys
import time
from pathlib import Path

from .feetech_bus import (
    SoArmBus, RESOLUTION, HOMING_SIGN_BIT,
    ADDR_HOMING_OFFSET, ADDR_MIN_POSITION_LIMIT, ADDR_MAX_POSITION_LIMIT, ADDR_OPERATING_MODE,
)
from .protocol import (
    MOTORS, IDS, ID_TO_NAME,
    PRESENT_POSITION_SIGN_BIT, encode_sign_magnitude, decode_sign_magnitude, pack_frame,
)

HALF_TURN = RESOLUTION // 2 - 1  # 2047


def _present_decoded(bus: SoArmBus) -> dict[int, int]:
    return {i: decode_sign_magnitude(v, PRESENT_POSITION_SIGN_BIT)
            for i, v in bus.read_present_position().items()}


def cmd_probe(args):
    bus = SoArmBus(args.port, args.baud)
    try:
        print(f"Opened {args.port} @ {args.baud} baud")
        ok_all = True
        for name, i in MOTORS:
            ok, model, comm, err = bus.ping(i)
            print(f"  id {i:>2} {name:<14} {'OK' if ok else 'MISSING'}  "
                  f"{'model=' + str(model) if ok else f'comm={comm} err={err}'}")
            ok_all = ok_all and ok
        if not ok_all:
            print("\nNot all 6 motors responded — check the arm is the right one and powered.")
            return 1
        pos = bus.read_present_position()
        print("Present_Position (raw ticks):")
        for name, i in MOTORS:
            print(f"  {name:<14} = {pos[i]}")
        return 0
    finally:
        bus.close()


def cmd_calibrate(args):
    """Replicate LeRobot's leader calibration locally (the arm is here, so Homing_Offset
    writes happen over USB). Emits a LeRobot-format MotorCalibration JSON."""
    bus = SoArmBus(args.port, args.baud)
    full_turn = args.full_turn_motor or None
    try:
        print("Disabling torque so you can move the arm by hand...")
        bus.disable_torque()
        for i in IDS:
            bus.write1(i, ADDR_OPERATING_MODE, 0)

        # reset_calibration: homing=0, min=0, max=4095
        for i in IDS:
            bus.write2(i, ADDR_HOMING_OFFSET, 0)
            bus.write2(i, ADDR_MIN_POSITION_LIMIT, 0)
            bus.write2(i, ADDR_MAX_POSITION_LIMIT, RESOLUTION - 1)

        # EEPROM commits slowly; verify the homing reset actually landed before reading.
        for _ in range(5):
            time.sleep(0.1)
            stale = {ID_TO_NAME[i]: bus.read_reg2(i, ADDR_HOMING_OFFSET)
                     for i in IDS if bus.read_reg2(i, ADDR_HOMING_OFFSET) != 0}
            if not stale:
                break
            for i in IDS:
                if bus.read_reg2(i, ADDR_HOMING_OFFSET) != 0:
                    bus.write2(i, ADDR_HOMING_OFFSET, 0)
        else:
            print(f"\nERROR: could not reset Homing_Offset to 0: {stale}")
            return 1

        input("\nMove the arm to the MIDDLE of its range of motion and press ENTER...")
        actual = _present_decoded(bus)
        oob = {ID_TO_NAME[i]: actual[i] for i in IDS if not (0 <= actual[i] <= RESOLUTION - 1)}
        if oob:
            print(f"\nERROR: implausible centered reading (expected 0..4095): {oob}")
            print("Power-cycle the arm (clears multi-turn encoder state) and re-run.")
            return 1

        homing = {i: actual[i] - HALF_TURN for i in IDS}
        bad = {ID_TO_NAME[i]: homing[i] for i in IDS if abs(homing[i]) > 2047}
        if bad:
            print(f"\nERROR: these joints were not centered (|homing| > 2047): {bad}")
            print("Re-center those joints (away from the encoder wrap) and re-run.")
            return 1
        for i in IDS:
            bus.write2(i, ADDR_HOMING_OFFSET, encode_sign_magnitude(homing[i], HOMING_SIGN_BIT))

        sweep = [n for n, _ in MOTORS] if full_turn is None else \
                [n for n, _ in MOTORS if n != full_turn]
        print(f"\nMove these joints through their full range: {', '.join(sweep)}")
        if full_turn:
            print(f"('{full_turn}' is recorded as a full turn 0..4095)")
        print("Recording min/max. Press ENTER to stop...")
        pos = _present_decoded(bus)
        mins, maxes = dict(pos), dict(pos)
        while True:
            pos = _present_decoded(bus)
            for i in IDS:
                mins[i] = min(mins[i], pos[i])
                maxes[i] = max(maxes[i], pos[i])
            line = "  ".join(f"{ID_TO_NAME[i][:4]}:{mins[i]:>4}/{pos[i]:>4}/{maxes[i]:>4}" for i in IDS)
            print("\r" + line, end="", flush=True)
            if select.select([sys.stdin], [], [], 0.02)[0]:
                sys.stdin.readline()
                break
        print()

        if full_turn:
            fid = dict(MOTORS)[full_turn]
            mins[fid], maxes[fid] = 0, RESOLUTION - 1

        calibration = {
            name: {"id": i, "drive_mode": 0, "homing_offset": int(homing[i]),
                   "range_min": int(mins[i]), "range_max": int(maxes[i])}
            for name, i in MOTORS
        }
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(calibration, indent=4))
        print(f"\nCalibration written to {out}")
        print("Copy it to the remote's LeRobot calibration dir, named <id>.json "
              "(the id you pass as --teleop.id / --robot_id there).")
        return 0
    finally:
        bus.close()


def cmd_stream(args):
    bus = SoArmBus(args.port, args.baud)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port_tcp))
    srv.listen(1)
    period = 1.0 / args.fps
    print(f"Streaming on {args.host}:{args.port_tcp} @ {args.fps} Hz. Waiting for the remote...")
    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Remote connected: {addr}")
            seq = 0
            try:
                next_t = time.perf_counter()
                while True:
                    conn.sendall(pack_frame(seq, bus.read_present_position()))
                    seq += 1
                    next_t += period
                    sleep = next_t - time.perf_counter()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_t = time.perf_counter()
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print(f"Remote disconnected ({e}); waiting for reconnect...")
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\nStopping stream.")
    finally:
        srv.close()
        bus.close()
    return 0


def cmd_latency_server(args):
    """Opt-in diagnostic, separate from streaming. Echoes 8-byte pings for RTT measurement
    (run instead of `stream`; the remote runs `network_leader --ping`)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port_tcp))
    srv.listen(1)
    print(f"Latency echo server on {args.host}:{args.port_tcp}. Waiting for client...")

    def recvall(conn, n):
        buf = b""
        while len(buf) < n:
            c = conn.recv(n - len(buf))
            if not c:
                return None
            buf += c
        return buf

    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Client connected: {addr}")
            try:
                while True:
                    p = recvall(conn, 8)
                    if p is None:
                        break
                    conn.sendall(p)
            except OSError as e:
                print(f"Client disconnected ({e}); waiting...")
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\nStopping latency server.")
    finally:
        srv.close()
    return 0


def main():
    p = argparse.ArgumentParser(prog="soarm-local", description="SO-arm local node (leader side)")
    p.add_argument("--port", required=True,
                   help="Serial port of the arm (Linux: /dev/ttyACM* ; macOS: /dev/cu.usbmodem*)")
    p.add_argument("--baud", type=int, default=1_000_000)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("probe", help="Verify the arm and read positions")

    sc = sub.add_parser("calibrate", help="Calibrate the arm, emit LeRobot JSON")
    sc.add_argument("--out", default="leader_calibration.json")
    sc.add_argument("--full-turn-motor", default="wrist_roll",
                    help="Motor recorded as a full turn 0..4095 (set empty to sweep all)")

    sp = sub.add_parser("stream", help="Stream joint positions to the remote over TCP")
    sp.add_argument("--host", default="127.0.0.1")
    sp.add_argument("--port-tcp", type=int, default=5599)
    sp.add_argument("--fps", type=float, default=120.0)

    sl = sub.add_parser("latency-server", help="Echo server for tunnel latency measurement")
    sl.add_argument("--host", default="127.0.0.1")
    sl.add_argument("--port-tcp", type=int, default=5599)

    args = p.parse_args()
    return {
        "probe": cmd_probe, "calibrate": cmd_calibrate,
        "stream": cmd_stream, "latency-server": cmd_latency_server,
    }[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
