"""Local Feetech STS3215 bus helpers (via scservo_sdk). Works on Linux and macOS.

Talks to the servos with the same scservo_sdk calls LeRobot uses internally, so raw ticks
are byte-identical to LeRobot's. Used by the local CLI to read/stream the leader and to
calibrate it.
"""
from __future__ import annotations

import time

import scservo_sdk as scs

from .protocol import IDS

DEFAULT_BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0          # Feetech STS -> protocol 0
RESOLUTION = 4096             # sts3215: 12-bit encoder
HOMING_SIGN_BIT = 11          # Homing_Offset sign-magnitude bit

# sts3215 control table
ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_HOMING_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
LEN_2 = 2


def _patched_set_packet_timeout(self, packet_length):  # noqa: N802
    """The PyPI scservo_sdk ships a buggy setPacketTimeout that under-waits, so slow
    responses (notably EEPROM writes) return 'no status packet'. LeRobot patches it the
    same way; we replicate it exactly."""
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (
        (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + 50
    )


class SoArmBus:
    """One bus class for both roles: opens the port, patches the packet timeout, and offers
    sync-read of Present_Position, sync-write of Goal_Position, torque + write helpers."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port_handler = scs.PortHandler(port)
        self.port_handler.setPacketTimeout = _patched_set_packet_timeout.__get__(
            self.port_handler, type(self.port_handler)
        )
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
        if not self.port_handler.openPort():
            raise IOError(f"Failed to open port {port}")
        if not self.port_handler.setBaudRate(baudrate):
            raise IOError(f"Failed to set baudrate {baudrate} on {port}")
        self.reader = scs.GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, LEN_2
        )
        for i in IDS:
            self.reader.addParam(i)

    # --- reads ---
    def ping(self, motor_id: int):
        model, comm, err = self.packet_handler.ping(self.port_handler, motor_id)
        return (comm == scs.COMM_SUCCESS and err == 0), model, comm, err

    def read_present_position(self) -> dict[int, int]:
        """Raw Present_Position register values (u16), byte-identical to lerobot's
        `_sync_read` output (before sign decode / normalize)."""
        comm = self.reader.txRxPacket()
        if comm != scs.COMM_SUCCESS:
            raise IOError(self.packet_handler.getTxRxResult(comm))
        out = {}
        for i in IDS:
            if not self.reader.isAvailable(i, ADDR_PRESENT_POSITION, LEN_2):
                raise IOError(f"sync read: data unavailable for id {i}")
            out[i] = self.reader.getData(i, ADDR_PRESENT_POSITION, LEN_2)
        return out

    def read_reg2(self, motor_id: int, addr: int) -> int:
        val, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, addr)
        return val

    # --- writes (with retry; EEPROM writes can be slow) ---
    def _write(self, fn, motor_id: int, addr: int, value: int, num_retry: int = 3):
        last = None
        for _ in range(1 + num_retry):
            comm, err = fn(self.port_handler, motor_id, addr, value)
            if comm == scs.COMM_SUCCESS and err == 0:
                return
            last = (comm, err)
            time.sleep(0.02)
        comm, err = last
        if comm != scs.COMM_SUCCESS:
            raise IOError(f"id {motor_id} addr {addr}: {self.packet_handler.getTxRxResult(comm)}")
        raise IOError(f"id {motor_id} addr {addr}: {self.packet_handler.getRxPacketError(err)}")

    def write2(self, motor_id: int, addr: int, value: int):
        self._write(self.packet_handler.write2ByteTxRx, motor_id, addr, value & 0xFFFF)

    def write1(self, motor_id: int, addr: int, value: int):
        self._write(self.packet_handler.write1ByteTxRx, motor_id, addr, value & 0xFF)

    # --- torque (calibration needs the arm limp so it can be moved by hand) ---
    def disable_torque(self):
        for i in IDS:
            self.write1(i, ADDR_TORQUE_ENABLE, 0)
            self.write1(i, ADDR_LOCK, 0)

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass
