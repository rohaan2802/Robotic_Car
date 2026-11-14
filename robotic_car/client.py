"""Bounded HTTP commands and telemetry for the NodeMCU sketch."""

from dataclasses import dataclass
import math

import requests

COMMANDS = ("F", "B", "L", "R", "RL", "RR", "S")


@dataclass(frozen=True)
class Attitude:
    yaw: float
    pitch: float
    roll: float


class CarClient:
    def __init__(self, base="http://192.168.4.1", *, session=None):
        self.base = base.rstrip("/")
        self.session = session if session is not None else requests.Session()
        if session is None:
            self.session.trust_env = False

    def request(self, endpoint, **kwargs):
        response = self.session.get(self.base + endpoint, timeout=(1, 1), **kwargs)
        response.raise_for_status()
        return response

    def command(self, direction, milliseconds=250):
        if direction not in COMMANDS:
            raise ValueError("Unknown drive command.")
        if direction != "S" and (type(milliseconds) is not int or not 50 <= milliseconds <= 1000):
            raise ValueError("Motion duration must be an integer from 50 to 1000 ms.")
        params = {"cmd": direction}
        if direction != "S":
            params["ms"] = milliseconds
        self.request("/command", params=params)

    def distance(self):
        raw = self.request("/distance").text.strip()
        try:
            value = float(raw)
        except ValueError:
            return None
        return value if math.isfinite(value) and value > 0 else None

    def attitude(self):
        body = self.request("/mpu").json()
        yaw, pitch, roll = (float(body[f"estimated_{axis}"]) for axis in ("yaw", "pitch", "roll"))
        if not all(map(math.isfinite, (yaw, pitch, roll))):
            raise ValueError("IMU response contains non-finite values.")
        return Attitude(yaw % 360, pitch, roll)
