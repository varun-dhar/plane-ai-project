"""
Step 2B – Actions module.

Defines the basic actions the RL agent can take and helpers for:
- changing heading (N, S, E, W, HOLD)
- changing altitude (UP / DOWN)
"""

import enum
from typing import Optional
from pymavlink import mavutil
from .mavlink_interface import MavlinkInterface


class Action(enum.IntEnum):
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    CLIMB = 4
    DESCEND = 5
    HOLD = 6


def change_heading(iface: MavlinkInterface, action: Action) -> None:
    """Send a simple yaw command to adjust heading."""
    iface._ensure_connected()
    master = iface.connection
    assert master is not None

    heading_map = {
        Action.NORTH: 0,
        Action.EAST: 90,
        Action.SOUTH: 180,
        Action.WEST: 270,
    }

    if action not in heading_map:
        print(f"[ACTION] {action.name}: no heading change")
        return

    new_heading = heading_map[action]
    print(f"[ACTION] Turning to {new_heading}°")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        new_heading,
        10.0,  # yaw rate deg/s
        1,     # clockwise
        0, 0, 0, 0,
    )


def change_altitude(
    iface: MavlinkInterface,
    action: Action,
    delta_m: float = 20.0,
    min_alt_m: float = 50.0,
    max_alt_m: float = 400.0,
) -> Optional[float]:
    """Compute the next altitude target)"""
    if action not in (Action.NORTH, Action.SOUTH):
        return None

    state = iface.get_state()
    alt = state["alt_m"]
    new_alt = alt + delta_m if action == Action.NORTH else alt - delta_m
    new_alt = max(min_alt_m, min(max_alt_m, new_alt))
    print(f"[ACTION] Altitude {alt:.1f} → {new_alt:.1f} m ({action.name})")
    return new_alt
