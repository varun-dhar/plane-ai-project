"""
MAVLink interface for the plane RL project (Step 2A).

Provides a small wrapper around pymavlink so we can:
- connect to ArduPilot SITL
- read the current aircraft state
- optionally compute distance to a target point
"""

from typing import Optional, Dict, Any
from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import pyproj
import math


def verify_ack(connection: mavutil.mavtcp, cmd_id: int, failure_message: str, die: bool = True):
	ack: mavlink.MAVLink_command_ack_message = connection.recv_match(type='COMMAND_ACK',
																	 condition=f'COMMAND_ACK.command=={cmd_id}',
																	 blocking=True)
	if ack.result != mavlink.MAV_RESULT_ACCEPTED and die:
		raise Exception(f'{failure_message} {ack.result}')
	return ack.result == mavlink.MAV_RESULT_ACCEPTED


class MavlinkInterface:
	def __init__(
			self,
			connection_str: str = "tcp:127.0.0.1:5760",
			timeout_s: int = 30,
	) -> None:
		self.timeout_s = timeout_s
		self.connection: mavutil.mavtcp = mavutil.mavlink_connection(connection_str,
																	 planner_format=False,
																	 notimestamps=True,
																	 robust_parsing=True,
																	 dialect="ardupilotmega",
																	 )
		if self.connection.wait_heartbeat(timeout=self.timeout_s) is None:
			raise Exception("Could not connect to MAVProxy")
		self.wp = mavwp.MAVWPLoader()

		messages = {
			"SYS_STATUS": mavlink.MAVLINK_MSG_ID_SYS_STATUS,
			"GLOBAL_POSITION_INT": mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
			"VFR_HUD": mavlink.MAVLINK_MSG_ID_VFR_HUD,
			"WIND": mavlink.MAVLINK_MSG_ID_WIND,
		}
		for name, msg_id in messages.items():
			self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
												  mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
												  msg_id, int(0.1e6), 0, 0, 0, 0, 0, 0)
			verify_ack(self.connection, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, f'Failed to request {name}')

		self.msl_to_wgs84 = pyproj.Transformer.from_crs(4979, 5773, always_xy=True)
		self.waypoint_number = 0

	# ------------------------ state reading ------------------------

	def get_state(self) -> dict[str, float]:
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

		needed = {
			"GLOBAL_POSITION_INT": None,
			"VFR_HUD": None,
			"WIND": None,
			"SYS_STATUS": None,
			"ATTITUDE": None
		}

		attempts = 0
		while attempts < 50 and any(v is None for v in needed.values()):
			msg = self.connection.recv_match(blocking=True, timeout=0.1)
			if msg is None or msg.get_type() == "BAD_DATA":
				attempts += 1
				continue

			msg_type = msg.get_type()
			if msg_type in needed:
				needed[msg_type] = msg
			else:
				attempts += 1

		gps: mavlink.MAVLink_global_position_int_message = needed["GLOBAL_POSITION_INT"]
		hud: mavlink.MAVLink_vfr_hud_message = needed["VFR_HUD"]
		wind: mavlink.MAVLink_wind_message = needed["WIND"]
		sys_status: mavlink.MAVLink_sys_status_message = needed["SYS_STATUS"]
		attitude: mavlink.MAVLink_attitude_message = needed["ATTITUDE"]

		# Position
		if gps is not None:
			lat = gps.lat / 1e7
			lon = gps.lon / 1e7
			alt_m = gps.alt / 1000.0  # mm -> m
			_, _, alt_m = self.msl_to_wgs84.transform(lon, lat, alt_m)
		else:
			lat = 0.0
			lon = 0.0
			alt_m = 0.0

		# Speed + heading
		if hud is not None:
			groundspeed_mps = float(hud.airspeed)
		else:
			groundspeed_mps = 0.0

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

		if attitude is not None:
			pitch = math.degrees(attitude.pitch)
		else:
			pitch = 0

		return {
			"altitude": alt_m,
			"airspeed": groundspeed_mps,
			"wind_speed": wind_speed_mps,
			"wind_dir": wind_dir_deg,
			"fuel_remaining": fuel_remaining,
			"latitude": lat,
			"longitude": lon,
			"pitch": pitch
		}

	def takeoff(self):
		self.connection.mav: mavlink.MAVLink = self.connection.mav
		self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
											  mavlink.MAV_CMD_DO_SET_MODE, 0, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
											  mavlink.PLANE_MODE_TAKEOFF,
											  0, 0, 0, 0, 0)
		self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
											  mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
		# self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
		#									  mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
		self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
											  mavlink.MAV_CMD_DO_SET_MODE, 0, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
											  mavlink.PLANE_MODE_GUIDED,
											  0, 0, 0, 0, 0)

	def set_waypoint(self, lat: float, lon: float, alt: float = 10):
		self.connection.mav: mavlink.MAVLink = self.connection.mav
		self.connection.waypoint_clear_all_send()
		self.connection.waypoint_count_send(1)
		self.connection.mav.mission_item_int_send(self.connection.target_system, self.connection.target_component, 0,
												  mavlink.MAV_FRAME_GLOBAL,
												  mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 10, 0, 0,
												  int(lat / 1e7), int(lon / 1e7), alt / 1000, 0)

	def set_speed_alt(self, airspeed: float, alt: float):
		self.connection.mav: mavlink.MAVLink = self.connection.mav
		self.connection.mav.command_int_send(self.connection.target_system, self.connection.target_component,
											 mavlink.MAV_FRAME_GLOBAL,
											 mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, mavlink.SPEED_TYPE_AIRSPEED,
											 airspeed, -1, 0, 0, 0, 0)
		self.connection.mav.command_int_send(self.connection.target_system, self.connection.target_component,
											 mavlink.MAV_FRAME_GLOBAL,
											 mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 0, 0,
											 alt, mavlink.MAV_FRAME_GLOBAL, 0, 0, 0, 0, 0)

	def set_wind(self, direction: float, speed: float):
		self.connection.mav: mavlink.MAVLink = self.connection.mav
		self.connection.mav.param_set_send(self.connection.target_system, self.connection.target_component,
										   'SIM_WIND_SPD'.encode('ascii'), speed, mavlink.MAV_PARAM_TYPE_REAL32)
		self.connection.mav.param_set_send(self.connection.target_system, self.connection.target_component,
										   'SIM_WIND_DIR'.encode('ascii'), direction, mavlink.MAV_PARAM_TYPE_REAL32)

	def reset_battery(self):
		self.connection.mav: mavlink.MAVLink = self.connection.mav
		self.connection.mav.param_set_send(self.connection.target_system, self.connection.target_component,
										   'SIM_BATT_VOLTAGE'.encode('ascii'), 12.6, mavlink.MAV_PARAM_TYPE_REAL32)
