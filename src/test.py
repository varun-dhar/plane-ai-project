import torch
from torch import nn
from torchrl.envs import (Compose, DoubleToFloat, ObservationNorm, StepCounter,
						  TransformedEnv, CatTensors)
from torchrl.envs.libs.gym import GymEnv
from torchrl.envs.utils import check_env_specs, ExplorationType, set_exploration_type
from torchrl.modules import ProbabilisticActor, TanhNormal
from tensordict.nn.distributions import NormalParamExtractor
import gazebo_gym
from collections import defaultdict
from matplotlib import pyplot as plt
from tensordict.nn import TensorDictModule
from tqdm import tqdm
import sys

device = (
	torch.device(0)
	if torch.cuda.is_available()
	else torch.device("cpu")
)
device = torch.device('cpu')
print(device)

num_cells = 2

base_env = GymEnv("GazeboPlaneEnv", device=device)

env = TransformedEnv(
	base_env,
	Compose(
		CatTensors(in_keys=["airspeed", "altitude", "fuel_remaining", "wind_dir", "wind_speed"], out_key="observation"),
		# normalize observations
		ObservationNorm(in_keys=["observation"]),
		DoubleToFloat(),
		StepCounter(),
	),
)

env.transform[1].init_stats(num_iter=1000, reduce_dim=0, cat_dim=0)

actor_net = nn.Sequential(
	nn.LazyLinear(num_cells, device=device),
	nn.Tanh(),
	nn.LazyLinear(num_cells, device=device),
	nn.Tanh(),
	nn.LazyLinear(num_cells, device=device),
	nn.Tanh(),
	nn.LazyLinear(2 * env.action_spec.shape[-1], device=device),
	NormalParamExtractor(),
)
actor_net.load_state_dict(torch.load('ppo_actor_weights.pt', weights_only=True, map_location=device))
actor_net.eval()

policy_module = TensorDictModule(
	actor_net, in_keys=["observation"], out_keys=["loc", "scale"]
)

policy_module = ProbabilisticActor(
	module=policy_module,
	spec=env.action_spec,
	in_keys=["loc", "scale"],
	distribution_class=TanhNormal,
	distribution_kwargs={
		"low": env.action_spec.space.low,
		"high": env.action_spec.space.high,
	},
	return_log_prob=True,
	# we'll need the log-prob for the numerator of the importance weights
)

logs = defaultdict(list)
for _ in tqdm(range(100)):
    with set_exploration_type(ExplorationType.DETERMINISTIC), torch.no_grad():
        # execute a rollout with the trained policy
        eval_rollout = env.rollout(1000, policy_module)
        logs["eval reward"].append(eval_rollout["next", "reward"].mean().item())
        logs["eval reward (sum)"].append(
            eval_rollout["next", "reward"].sum().item()
        )
        logs["eval step_count"].append(eval_rollout["step_count"].max().item())
        eval_str = (
            f"eval cumulative reward: {logs['eval reward (sum)'][-1]: 4.4f} "
            f"(init: {logs['eval reward (sum)'][0]: 4.4f}), "
            f"eval step-count: {logs['eval step_count'][-1]}"
        )
        del eval_rollout

plt.figure()
plt.plot(logs["eval reward"])
plt.title("Return (test)")
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.savefig('return-test.png')

plt.figure()
plt.plot(logs["eval step_count"])
plt.title("Max step count (test)")
plt.xlabel("Episode")
plt.ylabel("Steps")
plt.savefig('step-count-test.png')
