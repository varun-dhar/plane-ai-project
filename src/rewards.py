def reward(
		fuel_diff, dist_traveled,
		crashed=False, stalled=False, reached=False,
) -> float:
	"""
	Reward function

	Computes reward using:
	- efficiency
	- crash / stall / success bonuses
	"""
	efficiency = dist_traveled / fuel_diff if fuel_diff > 0 else 0

	r = 0
	r += 0.1 * efficiency  # increase reward if efficient with path/fuel.

	# negative rewards
	if crashed:
		r -= 500
	if stalled:
		r -= 200

	# reaches goal
	if reached:
		r += 1000

	return r
