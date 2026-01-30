"""
Minimal TD3 agent loader wrapper copied from ai_rescuebot's td3_agent.
Used to load actor network for inference when launching the packaged controller.
"""
import torch
import numpy as np
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, state_size, action_size, hidden_size=512):
        super().__init__()
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

    def forward(self, x):
        x = F.relu(self.fa1(x))
        x = F.relu(self.fa2(x))
        return torch.tanh(self.fa3(x))


class TD3:
    def __init__(self, device='cpu'):
        self.device = device
        self.actor = None

    def get_action(self, state_tensor, is_training=False, step=0):
        if self.actor is None:
            raise RuntimeError('Actor not loaded')
        with torch.no_grad():
            action = self.actor(state_tensor)
        return action.cpu().numpy()[0]

    def load_model(self, path):
        checkpoint = torch.load(path, map_location=self.device)

        # find actor state dict in common checkpoint formats
        state_dict = None
        if isinstance(checkpoint, dict):
            for key in ('actor', 'actor_state_dict', 'model_state_dict', 'model_state'):
                if key in checkpoint:
                    state_dict = checkpoint[key]
                    break
        if state_dict is None:
            # maybe the checkpoint is the actor state_dict itself
            state_dict = checkpoint

        # normalize keys (remove possible prefixes like 'module.' or 'actor.')
        def normalize(sd):
            new = {}
            for k, v in sd.items():
                nk = k
                if nk.startswith('module.'):
                    nk = nk[len('module.'):]
                if nk.startswith('actor.'):
                    nk = nk[len('actor.'):]
                new[nk] = v
            return new

        state_dict = normalize(state_dict)

        # infer network sizes from state_dict
        # look for first layer weight 'fa1.weight'
        if 'fa1.weight' in state_dict:
            fa1_w = state_dict['fa1.weight']
        else:
            # try alternative key names
            fa1_w = None
            for k in state_dict.keys():
                if k.endswith('fa1.weight'):
                    fa1_w = state_dict[k]
                    break

        if fa1_w is None:
            raise RuntimeError('Cannot infer actor architecture from checkpoint')

        # fa1_w shape is [hidden_size, input_dim]
        hidden_size = fa1_w.shape[0]
        input_dim = fa1_w.shape[1]

        # infer action dim from fa3.weight
        fa3_w = None
        if 'fa3.weight' in state_dict:
            fa3_w = state_dict['fa3.weight']
        else:
            for k in state_dict.keys():
                if k.endswith('fa3.weight'):
                    fa3_w = state_dict[k]
                    break

        if fa3_w is None:
            # fallback: assume action dim 2
            action_dim = 2
        else:
            action_dim = fa3_w.shape[0]

        # create actor with inferred sizes
        actor = Actor(input_dim, action_dim, hidden_size).to(self.device)

        try:
            actor.load_state_dict(state_dict)
        except Exception:
            # try matching keys by stripping potential prefixes
            filtered = {k.replace('module.', ''): v for k, v in state_dict.items()}
            actor.load_state_dict(filtered)

        self.actor = actor
