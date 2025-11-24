"""
Step 2C – Reward functions.

Computes reward using:
- distance from route
- heading error
- speed error
- crash / stall / success bonuses
"""

import math
from typing import Tuple
from pyproj import Geod

geod = Geod(ellps="WGS84")


def route_geometry(start_lat, start_lon, end_lat, end_lon) -> Tuple[float, float]:
    """Return (distance_m, ideal_course_deg)."""
    fwd_az, _, dist = geod.inv(start_lon, start_lat, end_lon, end_lat)
    return dist, fwd_az


def angle_diff_deg(a: float, b: float) -> float:
    """Smallest signed difference [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def reward(
    plane_lat, plane_lon, plane_speed, plane_heading,
    start_lat, start_lon, end_lat, end_lon,
    desired_speed=20.0,
    crashed=False, stalled=False, reached=False,
) -> float:
    """Compute a simple scalar reward."""
    dist, ideal_course = route_geometry(start_lat, start_lon, end_lat, end_lon)
    _, _, offroute = geod.inv(start_lon, start_lat, plane_lon, plane_lat)
    heading_err = abs(angle_diff_deg(plane_heading, ideal_course))
    speed_err = abs(plane_speed - desired_speed)

    r = 0
    r -= 0.01 * offroute
    r -= 0.5 * heading_err
    r -= 0.1 * speed_err

    if crashed:
        r -= 500
    if stalled:
        r -= 200
    if reached:
        r += 1000

    return r
