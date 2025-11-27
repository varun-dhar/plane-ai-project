from typing import Any

import numpy as np
import gymnasium as gym
from gymnasium.core import ObsType
from gz.sim import TestFixture
import math
import pyproj
import rewards
import os

from mavlink_interface import MavlinkInterface


class GazeboPlaneEnv(gym.Env):
	def __init__(self):
		home = os.getenv("HOME")
		self.fixture = TestFixture(f'{home}/sitl_models/Gazebo/worlds/vtail_runway.sdf')
		self.fixture.finalize()

		self.server = self.fixture.server()
		self.server.run(False, 0, False)

		self.plane = MavlinkInterface()
		self.observation_space = gym.spaces.Dict({
			"airspeed": gym.spaces.Box(5., 100., shape=(1,), dtype=np.float32),
			"altitude": gym.spaces.Box(5., 3050., shape=(1,), dtype=np.float32),
			"fuel_remaining": gym.spaces.Box(0., 1., shape=(1,), dtype=np.float32),
			"wind_dir": gym.spaces.Box(0., 2 * np.pi, shape=(1,), dtype=np.float32),
			"wind_speed": gym.spaces.Box(0., 20., shape=(1,), dtype=np.float32),
		})
		self.action_space = gym.spaces.Box(5., 100., shape=(2,), dtype=np.float32)
		self.dest_lat: float = 0
		self.dest_lon: float = 0
		self.start_lat: float = 0
		self.start_lon: float = 0
		self.positions: list[tuple[float, float]] = []
		self.fuel_left: list[float] = []
		self.wgs84_geod = pyproj.Geod(ellps='WGS84')
		self.plane.takeoff()

	def _get_obs(self):
		return self.plane.get_state()

	@staticmethod
	def _obs_to_user(obs: dict):
		return dict(filter(lambda item: item[0] not in ('latitude', 'longitude', 'pitch'), obs.items()))

	def reset(
			self,
			*,
			seed: int | None = None,
			options: dict[str, Any] | None = None,
	) -> tuple[ObsType, dict[str, Any]]:
		super().reset(seed=seed)
		#self.server.reset_all()
		obs = self._get_obs()
		max_dist_km = 500
		max_lat_deg = max_dist_km / 111.111
		lat_delta = self.np_random.uniform(-max_lat_deg, max_lat_deg)
		max_lon_deg = max_lat_deg * math.cos(math.radians(lat_delta)) - lat_delta * 111.111
		lon_delta = self.np_random.uniform(-abs(max_lon_deg), abs(max_lon_deg))
		self.start_lat = obs['latitude']
		self.start_lon = obs['longitude']
		self.positions = [(self.start_lat, self.start_lon)]
		self.dest_lat = self.start_lat + lat_delta
		self.dest_lon = self.start_lon + lon_delta
		self.plane.reset_battery()
		self.fuel_left = [obs['fuel_remaining']]
		self.plane.set_waypoint(self.dest_lat, self.dest_lon, 10)
		self.plane.set_wind(self.np_random.uniform(0., 360.), self.np_random.uniform(5., 100.), self.np_random.uniform(0., 12.85))

		return self._obs_to_user(obs), {}

	def step(self, action):
		airspeed = action[0]
		altitude = action[1]
		self.plane.set_speed_alt(airspeed, altitude)
		obs = self._get_obs()
		truncated = obs['altitude'] < 5 or obs['altitude'] > 3050
		plane_lat = obs['latitude']
		plane_lon = obs['longitude']
		_, _, distance = self.wgs84_geod.inv(plane_lon, plane_lat, self.dest_lon, self.dest_lat)
		done = truncated or distance < 10

		fuel_diff = obs['fuel_remaining'] - self.fuel_left[-1]
		_, _, dist_traveled = self.wgs84_geod.inv(plane_lon, plane_lat, self.positions[-1][1], self.positions[-1][0])
		reward = rewards.reward(airspeed, fuel_diff, dist_traveled, crashed=obs['altitude'] < 5,
								stalled=obs['pitch'] > 15,
								reached=distance < 10)
		self.positions.append((plane_lat, plane_lon))
		self.fuel_left.append(obs['fuel_remaining'])

		return self._obs_to_user(obs), reward, done, truncated, {}


gym.register(
	id="GazeboPlaneEnv",
	entry_point=GazeboPlaneEnv,
	max_episode_steps=300,  # Prevent infinite episodes
)


