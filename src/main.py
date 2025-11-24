from gz.sim import TestFixture, World, world_entity
import random
from pyproj import Geod


class Action(enum.IntEnum):
	HOLD = 0b1000
	CLIMB = 0b0001
	CLIMB_LEFT = 0b0101
	CLIMB_RIGHT = 0b0011
	DESCEND_LEFT = 0b0100
	DESCEND_RIGHT = 0b0010
	DESCEND = 0b0000
	LEFT = 0b0100
	RIGHT = 0b0010


Q_table = defaultdict(lambda: [0] * len(Action))
update_table = defaultdict(lambda: [0] * len(Action))

episode_rewards = []
reward_running_avg = []

total_reward = 0

num_episodes = 10000


def Q_learning(gamma=0.9, epsilon=1, decay_rate=0.999):
	"""
	Run Q-learning algorithm for a specified number of episodes.

    Parameters:
    - num_episodes (int): Number of episodes to run.
    - gamma (float): Discount factor.
    - epsilon (float): Exploration rate.
    - decay_rate (float): Rate at which epsilon decays. Epsilon should be decayed as epsilon = epsilon * decay_rate after each episode.

    Returns:
    - Q_table (dict): Dictionary containing the Q-values for each state-action pair.
    """
	episode_reward = 0
	obs, reward, done, info = env.reset()
	while not done:
		state_id = hash(obs)

		if random.random() < epsilon:
			action = env.action_space.sample()
		else:
			action = max(list(Action), key=lambda j: Q_table[state_id][j])

		obs, reward, done, info = env.step(action)
		eta = 1 / (1 + update_table[state_id][action])
		new_state_id = hash(obs)
		v = max(Q_table[new_state_id])
		Q_table[state_id][action] = (1 - eta) * Q_table[state_id][action] + eta * (reward + gamma * v)
		update_table[state_id][action] += 1
		episode_reward += reward
	epsilon *= decay_rate
	episode_rewards.append(episode_reward)
	total_reward += episode_reward
	reward_running_avg.append(total_reward / float(i + 1))


fixture = TestFixture('sitl_models/Gazebo/worlds/vtail_runway.sdf')


def find_closest_point(point, points):
	if len(points) == 1:
		return points[0]
	geod = Geod(ellps='WGS84')
	_, _, dist1 = geod.inv(point[0], point[1], points[len(points) // 2][0], points[len(points) // 2][1])
	_, _, dist2 = geod.inv(point[0], point[1], points[len(points) // 2 - 1][0], points[len(points) // 2 - 1][1])
	if dist1 > dist2:
		return find_closest_point(point, points[len(points) // 2-1:])
	return find_closest_point(point, points[:len(points) // 2 - 1])

def on_pre_update_cb(_info, _ecm):
	global pre_iterations
	global first_iteration
	pre_iterations += 1
	if first_iteration:
		first_iteration = False
		world_e = world_entity(_ecm)
		print('World entity is ', world_e)
		w = World(world_e)
		modelEntity = w.model_by_name(_ecm, 'mini_talon_vtail')
	plane_pos = None
	max_dist_km = 500
	max_lat_deg = max_dist_km / 111.111
	lat_delta = random.uniform(0, max_lat_deg)
	max_lon_deg = max_lat_deg * math.cos(math.radians(lat_delta)) - lat_delta * 111.111
	lon_delta = random.uniform(0, max_lon_deg)
	geod = Geod(ellps='WGS84')
	points = geod.npts(start_lat, start_lon, end_lat, end_lon)
	closest_lat, closest_lon = find_closest_point((plane_lat, plane_lon), points)
	fwd_azimuth, _, _ = geod.inv(closest_lat, closest_lon, end_lat, end_lon)
	heading_error = fwd_azimuth - plane_heading
	max_alt_m = 4300
	min_alt_m = 150
	alt = random.uniform(min_alt_m, max_alt_m)
	alt_error = alt - plane_alt


def on_update_cb(_info, _ecm):
	global iterations
	iterations += 1


def on_post_update_cb(_info, _ecm):
	global post_iterations
	post_iterations += 1
	if _info.sim_time.seconds == 1:
		print('Post update sim time: ', _info.sim_time)


fixture.on_post_update(on_post_udpate_cb)
fixture.on_update(on_udpate_cb)
fixture.on_pre_update(on_pre_udpate_cb)
fixture.finalize()

server = fixture.server()
server.run(True, 1000, False)
