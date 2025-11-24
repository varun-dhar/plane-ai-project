"""
MAVLink interface for the plane RL project (Step 2A).

Provides a small wrapper around pymavlink so we can:
- connect to ArduPilot SITL
- read the current aircraft state
- optionally compute distance to a target point
"""

from typing import Optional, Dict, Any
from pymavlink import mavutil
import math


class MavlinkInterface:
    def __init__(
        self,
        connection_str: str = "tcp:127.0.0.1:5760",
        timeout_s: int = 30,
    ) -> None:
        self.connection_str = connection_str
        self.timeout_s = timeout_s
        self.master: Optional[mavutil.mavfile] = None

    # ------------------------ connection ------------------------

    def connect(self) -> None:
        """Connect to ArduPilot SITL if not already connected."""
        if self.master is not None:
            return

        print(f"[MAVLINK] Connecting to {self.connection_str} ...")
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.master.wait_heartbeat(timeout=self.timeout_s)
        print(
            f"[MAVLINK] Heartbeat from sys={self.master.target_system}, "
            f"comp={self.master.target_component}"
        )

    def _ensure_connected(self) -> None:
        if self.master is None:
            self.connect()

    # ------------------------ state reading ------------------------

    def get_state(
        self,
        target_lat: Optional[float] = None,
        target_lon: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Read the current aircraft state.

        Returns a dict with:
        - lat, lon (deg)
        - alt_m (m)
        - groundspeed_mps (m/s)
        - heading_deg (deg)
        - wind_speed_mps (m/s)
        - wind_dir_deg (deg)
        - fuel_remaining (0..1, rough estimate)
        - distance_to_target_m (m, or None if no target provided)
        """
        self._ensure_connected()

        needed = {
            "GLOBAL_POSITION_INT": None,
            "VFR_HUD": None,
            "WIND": None,
            "SYS_STATUS": None,
        }

        attempts = 0
        while attempts < 50 and any(v is None for v in needed.values()):
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                attempts += 1
                continue

            msg_type = msg.get_type()
            if msg_type in needed and needed[msg_type] is None:
                needed[msg_type] = msg
            attempts += 1

        gps = needed["GLOBAL_POSITION_INT"]
        hud = needed["VFR_HUD"]
        wind = needed["WIND"]
        sys_status = needed["SYS_STATUS"]

        # Position
        if gps is not None:
            lat_deg = gps.lat / 1e7
            lon_deg = gps.lon / 1e7
            alt_m = gps.relative_alt / 1000.0  # mm -> m
        else:
            lat_deg = 0.0
            lon_deg = 0.0
            alt_m = 0.0

        # Speed + heading
        if hud is not None:
            groundspeed_mps = float(hud.groundspeed)
            heading_deg = float(hud.heading)
        else:
            groundspeed_mps = 0.0
            heading_deg = 0.0

        # Wind
        if wind is not None:
            wind_speed_mps = float(wind.speed)
            wind_dir_deg = float(wind.direction)
        else:
            wind_speed_mps = 0.0
            wind_dir_deg = 0.0

        # Fuel estimate (using battery_remaining as a proxy)
        if sys_status is not None and sys_status.battery_remaining != -1:
            fuel_remaining = sys_status.battery_remaining / 100.0
        else:
            fuel_remaining = 1.0  # assume full if unknown

        # Distance to target (optional)
        if target_lat is not None and target_lon is not None:
            distance_to_target_m = self._haversine_distance_m(
                lat_deg, lon_deg, target_lat, target_lon
            )
        else:
            distance_to_target_m = None

        return {
            "lat": lat_deg,
            "lon": lon_deg,
            "alt_m": alt_m,
            "groundspeed_mps": groundspeed_mps,
            "heading_deg": heading_deg,
            "wind_speed_mps": wind_speed_mps,
            "wind_dir_deg": wind_dir_deg,
            "fuel_remaining": fuel_remaining,
            "distance_to_target_m": distance_to_target_
       }
