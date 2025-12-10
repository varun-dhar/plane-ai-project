import torch
from tensordict.nn import NormalParamExtractor
from torch import nn
from torchrl.envs import TransformedEnv


class Actor(nn.Module):
	'''
	Actor Network - performs actions given state from the environment
	'''
	def __init__(self, num_cells: int, env: TransformedEnv, device: torch.Device):
		super().__init__()
		self.actor_stack = nn.Sequential(
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(2 * env.action_spec.shape[-1], device=device),
			NormalParamExtractor(),
		)

	def forward(self, x: torch.Tensor) -> torch.Tensor:
		return self.actor_stack(x)
