"""
TD3 (Twin Delayed Deep Deterministic Policy Gradient) implementation.
Based on turtlebot3_drlnav repository.
"""
import numpy as np
import copy
import torch
import torch.nn as nn
import torch.nn.functional as F
from abc import ABC, abstractmethod

from .settings import (
    ACTION_SIZE, HIDDEN_SIZE, BATCH_SIZE, BUFFER_SIZE,
    DISCOUNT_FACTOR, LEARNING_RATE, TAU, POLICY_NOISE,
    POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY, NUM_SCAN_SAMPLES
)


class Network(nn.Module, ABC):
    """Base network class."""
    
    def __init__(self, name):
        super(Network, self).__init__()
        self.name = name
        self.visual = None
        
    def init_weights(self, m):
        """Initialize weights with uniform distribution."""
        if isinstance(m, nn.Linear):
            nn.init.xavier_uniform_(m.weight)
            nn.init.constant_(m.bias, 0.01)
            
    def attach_visual(self, visual):
        """Attach visualization (optional)."""
        self.visual = visual


class Actor(Network):
    """Actor network for TD3."""
    
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)
        
        # Define layers
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)
        
        self.apply(super().init_weights)
        
    def forward(self, states, visualize=False):
        """Forward pass through actor network."""
        x1 = torch.relu(self.fa1(states))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))
        
        # Optional visualization
        if visualize and self.visual:
            self.visual.update_layers(states, action, [x1, x2], 
                                     [self.fa1.bias, self.fa2.bias])
        
        return action


class Critic(Network):
    """Critic network for TD3 with twin Q-functions."""
    
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)
        
        # Q1 network
        self.l1 = nn.Linear(state_size, int(hidden_size / 2))
        self.l2 = nn.Linear(action_size, int(hidden_size / 2))
        self.l3 = nn.Linear(hidden_size, hidden_size)
        self.l4 = nn.Linear(hidden_size, 1)
        
        # Q2 network
        self.l5 = nn.Linear(state_size, int(hidden_size / 2))
        self.l6 = nn.Linear(action_size, int(hidden_size / 2))
        self.l7 = nn.Linear(hidden_size, hidden_size)
        self.l8 = nn.Linear(hidden_size, 1)
        
        self.apply(super().init_weights)
        
    def forward(self, states, actions):
        """Forward pass through both Q-networks."""
        # Q1
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l3(x))
        x1 = self.l4(x)
        
        # Q2
        xs = torch.relu(self.l5(states))
        xa = torch.relu(self.l6(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l7(x))
        x2 = self.l8(x)
        
        return x1, x2
        
    def Q1_forward(self, states, actions):
        """Forward pass through Q1 network only."""
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l3(x))
        x1 = self.l4(x)
        return x1


class OUNoise:
    """Ornstein-Uhlenbeck process for exploration noise."""
    
    def __init__(self, action_space, max_sigma=0.1, min_sigma=0.1, 
                 decay_period=8000000):
        self.action_space = action_space
        self.max_sigma = max_sigma
        self.min_sigma = min_sigma
        self.decay_period = decay_period
        
    def get_noise(self, step):
        """Generate noise that decays over time."""
        sigma = self.max_sigma - (self.max_sigma - self.min_sigma) * min(1.0, step / self.decay_period)
        return np.random.normal(0, sigma, self.action_space)


class TD3:
    """TD3 agent implementation."""
    
    def __init__(self, device='cpu'):
        self.device = device
        
        # Network structure
        self.state_size = NUM_SCAN_SAMPLES + 4  # scan + goal_dist + goal_angle + prev_actions
        self.action_size = ACTION_SIZE
        self.hidden_size = HIDDEN_SIZE
        
        # Hyperparameters
        self.batch_size = BATCH_SIZE
        self.buffer_size = BUFFER_SIZE
        self.discount_factor = DISCOUNT_FACTOR
        self.learning_rate = LEARNING_RATE
        self.tau = TAU
        
        # TD3 specific parameters
        self.policy_noise = POLICY_NOISE
        self.noise_clip = POLICY_NOISE_CLIP
        self.policy_freq = POLICY_UPDATE_FREQUENCY
        
        # Exploration noise
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, 
                            min_sigma=0.1, decay_period=8000000)
        
        # Training iteration counter
        self.iteration = 0
        self.last_actor_loss = 0
        
        # Create networks
        self.actor = Actor('actor', self.state_size, self.action_size, 
                          self.hidden_size).to(device)
        self.actor_target = Actor('target_actor', self.state_size, 
                                  self.action_size, self.hidden_size).to(device)
        self.actor_optimizer = torch.optim.AdamW(self.actor.parameters(), 
                                                 lr=self.learning_rate)
        
        self.critic = Critic('critic', self.state_size, self.action_size, 
                            self.hidden_size).to(device)
        self.critic_target = Critic('target_critic', self.state_size, 
                                    self.action_size, self.hidden_size).to(device)
        self.critic_optimizer = torch.optim.AdamW(self.critic.parameters(), 
                                                  lr=self.learning_rate)
        
        # Initialize target networks
        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)
        
        # Loss function
        self.loss_function = F.smooth_l1_loss
        
    def get_action(self, state, is_training, step, visualize=False):
        """Get action from actor network."""
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        action = self.actor(state, visualize)
        
        if is_training:
            noise = torch.from_numpy(copy.deepcopy(self.noise.get_noise(step))).to(self.device)
            action = torch.clamp(torch.add(action, noise), -1.0, 1.0)
            
        return action.detach().cpu().data.numpy().tolist()
        
    def get_action_random(self):
        """Get random action for exploration."""
        # Random linear velocity [-1, 1] (Mapped to [0, MAX] by env when backward disabled)
        action_linear = np.random.uniform(-1.0, 1.0)
        action_angular = np.random.uniform(-1.0, 1.0)
        return [action_linear, action_angular]
        
    def train(self, state, action, reward, state_next, done):
        """Train the TD3 agent."""
        # Add noise to target policy
        noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
        action_next = (self.actor_target(state_next) + noise).clamp(-1.0, 1.0)
        
        # Compute target Q-values
        Q1_next, Q2_next = self.critic_target(state_next, action_next)
        Q_next = torch.min(Q1_next, Q2_next)
        Q_target = reward + (1 - done) * self.discount_factor * Q_next
        
        # Get current Q-values
        Q1, Q2 = self.critic(state, action)
        
        # Compute critic loss
        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)
        
        # Optimize critic
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=2.0, norm_type=2)
        self.critic_optimizer.step()
        
        # Delayed policy updates
        if self.iteration % self.policy_freq == 0:
            # Compute actor loss
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()
            
            # Optimize actor
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
            self.actor_optimizer.step()
            
            # Soft update target networks
            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)
            
            self.last_actor_loss = loss_actor.mean().detach().cpu()
            
        self.iteration += 1
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]
        
    def hard_update(self, target, source):
        """Copy parameters from source to target network."""
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(param.data)
            
    def soft_update(self, target, source, tau):
        """Soft update of target network parameters."""
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)
            
    def save_model(self, path):
        """Save model weights."""
        torch.save({
            'actor': self.actor.state_dict(),
            'actor_target': self.actor_target.state_dict(),
            'critic': self.critic.state_dict(),
            'critic_target': self.critic_target.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
            'iteration': self.iteration
        }, path)
        
    def load_model(self, path):
        """Load model weights."""
        checkpoint = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(checkpoint['actor'])
        self.actor_target.load_state_dict(checkpoint['actor_target'])
        self.critic.load_state_dict(checkpoint['critic'])
        self.critic_target.load_state_dict(checkpoint['critic_target'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
        self.iteration = checkpoint.get('iteration', 0)
