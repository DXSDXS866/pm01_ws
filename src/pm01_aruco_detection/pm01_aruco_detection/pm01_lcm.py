"""Minimal LCM encoder for PM01 virtual gamepad messages."""

from io import BytesIO
import struct


class GamepadKeys:
    """LCM GamepadKeys message used by the PM01 virtual gamepad channel."""

    __slots__ = ["timestamp", "digital_states", "analog_states"]
    _HASH = 0xD6CAE60F8643A772

    def __init__(self) -> None:
        self.timestamp = 0
        self.digital_states = [0] * 12
        self.analog_states = [0.0] * 6

    @classmethod
    def _packed_fingerprint(cls) -> bytes:
        return struct.pack(">Q", cls._HASH)

    def encode(self) -> bytes:
        buffer = BytesIO()
        buffer.write(self._packed_fingerprint())
        buffer.write(struct.pack(">q", int(self.timestamp)))
        buffer.write(struct.pack(">12i", *self.digital_states[:12]))
        buffer.write(struct.pack(">6d", *self.analog_states[:6]))
        return buffer.getvalue()
