"""
Step 2C – Reward functions.

Computes reward using:
- distance from route
- heading error
- speed error
- crash / stall / success bonuses
"""

#imports
import math
from typing import Tuple
from pyproj import Geod

geod = Geod(ellps="WGS84")

# reward function 
def reward(
		plane_speed, fuel_diff, dist_traveled,
		desired_speed=20.0, # manuel setting target speed
		crashed=False, stalled=False, reached=False,
) -> float:
	"""Compute a simple scalar reward."""
	speed_err = abs(plane_speed - desired_speed)
	efficiency = dist_traveled / fuel_diff if fuel_diff > 0 else 0

	#set reward 0 
	r = 0
	r -= 0.1 * speed_err # decrease with speed error
	r += 0.1 * efficiency #increase reward if efficient with path/fuel.

	# negative rewards
	if crashed:
		r -= 500
	if stalled:
		r -= 200

	#reaches goal
	if reached:
		r += 1000

	return r
