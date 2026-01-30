"""
Replay buffer for storing and sampling experience tuples.
"""
import numpy as np
import random
from collections import deque


class ReplayBuffer:
    """Experience replay buffer for off-policy RL algorithms."""
    
    def __init__(self, size):
        """
        Initialize replay buffer.
        
        Args:
            size: Maximum size of buffer (FIFO when full)
        """
        self.buffer = deque(maxlen=size)
        self.max_size = size
        
    def sample(self, batch_size):
        """
        Sample a batch of experiences from buffer.
        
        Args:
            batch_size: Number of samples to return
            
        Returns:
            Tuple of numpy arrays: (states, actions, rewards, next_states, dones)
        """
        batch_size = min(batch_size, self.get_length())
        batch = random.sample(self.buffer, batch_size)
        
        states = np.float32([sample[0] for sample in batch])
        actions = np.float32([sample[1] for sample in batch])
        rewards = np.float32([sample[2] for sample in batch])
        next_states = np.float32([sample[3] for sample in batch])
        dones = np.float32([sample[4] for sample in batch])
        
        return states, actions, rewards, next_states, dones
        
    def get_length(self):
        """Return current size of buffer."""
        return len(self.buffer)
        
    def add_sample(self, state, action, reward, next_state, done):
        """
        Add a new experience to buffer.
        
        Args:
            state: Current state
            action: Action taken
            reward: Reward received
            next_state: Next state
            done: Episode done flag
        """
        transition = (state, action, reward, next_state, done)
        self.buffer.append(transition)
        
    def clear(self):
        """Clear the buffer."""
        self.buffer.clear()
