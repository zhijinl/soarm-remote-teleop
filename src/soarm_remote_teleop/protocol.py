"""Wire protocol + SO-arm motor table + sign-magnitude codecs.

Stdlib-only, shared by the local node and the remote adapters. SO-100 and SO-101 share the
same 6× Feetech STS3215 layout; the motor table is the single source of truth, and the
frame size is derived from it, so widening to a different DoF count is a one-line change.
"""
from __future__ import annotations

import struct
import time

# SO-100 / SO-101 motors in LeRobot order: (name, servo id).
MOTORS: tuple[tuple[str, int], ...] = (
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_roll", 5),
    ("gripper", 6),
)
IDS: list[int] = [i for _, i in MOTORS]
ID_TO_NAME: dict[int, str] = {i: n for n, i in MOTORS}

# Frame on the wire:  MAGIC + <timestamp_us: u64><seq: u32><one u16 per motor>
MAGIC = b"\xa5\x5a"
BODY_FMT = "<QI" + "H" * len(MOTORS)
BODY_SIZE = struct.calcsize(BODY_FMT)
FRAME_SIZE = len(MAGIC) + BODY_SIZE

# STS3215 encodes Present_Position and Goal_Position as sign-magnitude with bit 15.
PRESENT_POSITION_SIGN_BIT = 15
GOAL_POSITION_SIGN_BIT = 15


def encode_sign_magnitude(value: int, sign_bit: int) -> int:
    return ((1 << sign_bit) | (-value)) if value < 0 else value


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    sign = -1 if value & (1 << sign_bit) else 1
    return sign * (value & ((1 << sign_bit) - 1))


def pack_frame(seq: int, values_by_id: dict[int, int]) -> bytes:
    """Pack one frame; values_by_id maps servo id -> raw u16 register value."""
    ts_us = int(time.time() * 1e6)
    return MAGIC + struct.pack(
        BODY_FMT, ts_us, seq & 0xFFFFFFFF, *[values_by_id[i] & 0xFFFF for i in IDS]
    )


def parse_latest(buf: bytes) -> tuple[dict[int, int] | None, bytes]:
    """Parse all complete frames in `buf`, keeping only the most recent (drops backlog).

    Returns (latest_values_by_id_or_None, remaining_buf). Resynchronizes on MAGIC.
    """
    latest: dict[int, int] | None = None
    while len(buf) >= FRAME_SIZE:
        if buf[:2] != MAGIC:
            buf = buf[1:]
            continue
        fields = struct.unpack(BODY_FMT, buf[2:FRAME_SIZE])
        buf = buf[FRAME_SIZE:]
        pos = fields[2:]  # skip ts_us, seq
        latest = {IDS[k]: pos[k] for k in range(len(IDS))}
    return latest, buf
