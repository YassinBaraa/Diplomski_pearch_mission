import json
from dataclasses import dataclass


@dataclass
class PerchPacket:
    error_x: float
    error_y: float
    tof_mm: float
    timestamp: float

    @staticmethod
    def from_json(data: bytes) -> 'PerchPacket':
        d = json.loads(data.decode('utf-8'))
        return PerchPacket(
            error_x=float(d['error_x']),
            error_y=float(d['error_y']),
            tof_mm=float(d['tof']),
            timestamp=float(d['timestamp']),
        )
