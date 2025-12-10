import torch
from torch import nn


class Critic(nn.Module):
	'''
	Critic network - determines value of actions based on state and action taken
	'''
	def __init__(self, num_cells: int, device: torch.Device) -> None:
		super().__init__()
		self.critic_stack = nn.Sequential(
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(num_cells, device=device),
			nn.Tanh(),
			nn.LazyLinear(1, device=device),
		)

	def forward(self, x: torch.Tensor) -> torch.Tensor:
		return self.critic_stack(x)
