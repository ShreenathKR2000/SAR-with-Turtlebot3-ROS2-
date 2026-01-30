#!/usr/bin/env python3
"""
TD3 Training Script for TurtleBot3 in SAR Room World.
Main training loop managing episodes, data collection, and model checkpointing.
"""
import os
import sys
import time
import argparse
from datetime import datetime

# Ensure we use the virtual environment Python with PyTorch
venv_python = '/opt/venv/bin/python3'
if sys.executable != venv_python and os.path.exists(venv_python):
    print(f"Switching to virtual environment Python: {venv_python}")
    os.execv(venv_python, [venv_python] + sys.argv)
import torch
import rclpy

from .td3_agent import TD3
from .replay_buffer import ReplayBuffer
from .sar_environment import SAREnvironment
from .plotter import TrainingPlotter
from .settings import (
    OBSERVE_STEPS, MODEL_STORE_INTERVAL, GRAPH_DRAW_INTERVAL,
    BATCH_SIZE, OUTCOME_LABELS
)


class TD3Trainer:
    """Main training class for TD3 agent."""
    
    def __init__(self, checkpoint_dir='./checkpoints/td3_sar', 
                 load_model=None, start_episode=0):
        """
        Initialize trainer.
        
        Args:
            checkpoint_dir: Directory to save checkpoints
            load_model: Path to model to load (None for new training)
            start_episode: Episode number to start from (for resuming)
        """
        # Setup directories
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.checkpoint_dir = checkpoint_dir
        self.session_dir = os.path.join(checkpoint_dir, f'session_{timestamp}')
        os.makedirs(self.session_dir, exist_ok=True)
        
        # GPU/CPU setup
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f'Using device: {self.device}')
        
        # Initialize components
        self.agent = TD3(device=self.device)
        self.replay_buffer = ReplayBuffer(self.agent.buffer_size)
        self.plotter = TrainingPlotter(self.session_dir)
        
        # Training state
        self.episode = start_episode
        self.total_steps = 0
        self.observe_steps = OBSERVE_STEPS
        
        # Load model if specified
        if load_model and os.path.exists(load_model):
            print(f'Loading model from {load_model}')
            self.agent.load_model(load_model)
            
            # Try to load replay buffer and plot data
            buffer_path = os.path.join(os.path.dirname(load_model), 'replay_buffer.pkl')
            if os.path.exists(buffer_path):
                import pickle
                with open(buffer_path, 'rb') as f:
                    self.replay_buffer.buffer = pickle.load(f)
                print(f'Loaded replay buffer with {self.replay_buffer.get_length()} samples')
                
                # Update total_steps to match buffer length so we don't restart observation
                if self.replay_buffer.get_length() > 0:
                    self.total_steps = self.replay_buffer.get_length()
                    print(f'Updated total_steps to {self.total_steps} from replay buffer')
                
            plot_data_path = os.path.join(os.path.dirname(load_model), 'plot_data.npz')
            if os.path.exists(plot_data_path):
                self.plotter.load_data(plot_data_path)
                print('Loaded plot data')
        
        # Create log file
        self.log_file = open(os.path.join(self.session_dir, 'training_log.csv'), 'w')
        self.log_file.write('episode,reward,outcome,duration,steps,total_steps,'
                           'buffer_size,avg_critic_loss,avg_actor_loss\n')
        self.log_file.flush()
        
        print(f'Session directory: {self.session_dir}')
        print(f'Starting from episode: {self.episode}')
        
    def train_episode(self, env, episode_num):
        """
        Train for one episode.
        
        Args:
            env: SAREnvironment instance
            
        Returns:
            dict: Episode statistics
        """
        state = env.reset(episode_num)
        
        episode_reward = 0
        episode_steps = 0
        loss_critic_sum = 0
        loss_actor_sum = 0
        episode_start = time.time()
        
        done = False
        action_prev = [0.0, 0.0]
        
        while not done and rclpy.ok():
            # Select action
            if self.total_steps < self.observe_steps:
                # Random exploration
                action = self.agent.get_action_random()
            else:
                # Policy action with exploration noise
                action = self.agent.get_action(state, is_training=True, 
                                               step=self.total_steps, visualize=False)
            
            # Execute action
            next_state, reward, done, info = env.step(action)
            
            episode_reward += reward
            episode_steps += 1
            self.total_steps += 1
            
            # Store transition
            self.replay_buffer.add_sample(state, action, [reward], next_state, [float(done)])
            
            # Train agent
            if self.replay_buffer.get_length() >= BATCH_SIZE:
                batch = self.replay_buffer.sample(BATCH_SIZE)
                states, actions, rewards, next_states, dones = batch
                
                states_t = torch.from_numpy(states).to(self.device)
                actions_t = torch.from_numpy(actions).to(self.device)
                rewards_t = torch.from_numpy(rewards).to(self.device)
                next_states_t = torch.from_numpy(next_states).to(self.device)
                dones_t = torch.from_numpy(dones).to(self.device)
                
                loss_critic, loss_actor = self.agent.train(states_t, actions_t, rewards_t, 
                                                           next_states_t, dones_t)
                loss_critic_sum += float(loss_critic)
                loss_actor_sum += float(loss_actor)
            
            state = next_state
            action_prev = action
            
            # Spin to process callbacks and allow simulation to update
            # Multiple spins to ensure odometry and scan updates are received
            for _ in range(5):
                rclpy.spin_once(env, timeout_sec=0.01)
            
        episode_duration = time.time() - episode_start
        
        return {
            'reward': episode_reward,
            'steps': episode_steps,
            'duration': episode_duration,
            'outcome': info['outcome'],
            'distance_traveled': info['distance_traveled'],
            'loss_critic_sum': loss_critic_sum,
            'loss_actor_sum': loss_actor_sum
        }
        
    def save_checkpoint(self, episode):
        """
        Save model checkpoint and training data.
        
        Args:
            episode: Current episode number
        """
        checkpoint_path = os.path.join(self.session_dir, f'model_ep{episode}.pt')
        self.agent.save_model(checkpoint_path)
        
        # Save replay buffer
        import pickle
        buffer_path = os.path.join(self.session_dir, f'replay_buffer_ep{episode}.pkl')
        with open(buffer_path, 'wb') as f:
            pickle.dump(self.replay_buffer.buffer, f)
            
        # Save plot data
        plot_data_path = os.path.join(self.session_dir, f'plot_data_ep{episode}.npz')
        self.plotter.save_data(plot_data_path)
        
        # Also save as "latest"
        latest_model = os.path.join(self.session_dir, 'model_latest.pt')
        latest_buffer = os.path.join(self.session_dir, 'replay_buffer.pkl')
        latest_plot = os.path.join(self.session_dir, 'plot_data.npz')
        
        self.agent.save_model(latest_model)
        with open(latest_buffer, 'wb') as f:
            pickle.dump(self.replay_buffer.buffer, f)
        self.plotter.save_data(latest_plot)
        
        print(f'Saved checkpoint at episode {episode}')
        
    def train(self, num_episodes=10000):
        """
        Main training loop.
        
        Args:
            num_episodes: Total number of episodes to train
        """
        # Initialize ROS2 and environment
        if not rclpy.ok():
            rclpy.init()
            
        env = SAREnvironment()
        
        # Give environment time to initialize
        time.sleep(2.0)
        
        print(f'\nStarting TD3 training for {num_episodes} episodes')
        print(f'Observation phase: {self.observe_steps} steps')
        print('='*70)
        
        try:
            while self.episode < num_episodes and rclpy.ok():
                self.episode += 1
                
                # Train one episode
                stats = self.train_episode(env, self.episode)
                
                # Log results
                if self.total_steps < self.observe_steps:
                    print(f"Observe: {self.total_steps}/{self.observe_steps} steps")
                else:
                    avg_critic_loss = stats['loss_critic_sum'] / stats['steps'] if stats['steps'] > 0 else 0
                    avg_actor_loss = stats['loss_actor_sum'] / stats['steps'] if stats['steps'] > 0 else 0
                    
                    print(f"Ep: {self.episode:<5} "
                          f"R: {stats['reward']:<8.0f} "
                          f"Outcome: {OUTCOME_LABELS[stats['outcome']]:<18} "
                          f"Steps: {stats['steps']:<5} "
                          f"Total: {self.total_steps:<7} "
                          f"Time: {stats['duration']:<6.2f}s")
                    
                    # Update plot data
                    self.plotter.update_data(
                        self.episode, stats['steps'], self.total_steps,
                        stats['outcome'], stats['reward'],
                        stats['loss_critic_sum'], stats['loss_actor_sum']
                    )
                    
                    # Log to file
                    self.log_file.write(f"{self.episode},{stats['reward']},{stats['outcome']},"
                                       f"{stats['duration']},{stats['steps']},{self.total_steps},"
                                       f"{self.replay_buffer.get_length()},{avg_critic_loss},"
                                       f"{avg_actor_loss}\n")
                    self.log_file.flush()
                    
                    # Draw plots
                    if self.episode % GRAPH_DRAW_INTERVAL == 0:
                        self.plotter.draw_plots(self.episode)
                        success_rate = self.plotter.get_success_count()
                        avg_reward = self.plotter.get_reward_average()
                        print(f"  --> Last {GRAPH_DRAW_INTERVAL} eps: "
                              f"Success: {success_rate}/{GRAPH_DRAW_INTERVAL}, "
                              f"Avg Reward: {avg_reward:.1f}")
                    
                    # Save checkpoint
                    if self.episode % MODEL_STORE_INTERVAL == 0:
                        self.save_checkpoint(self.episode)
                        
        except KeyboardInterrupt:
            print('\n\nTraining interrupted by user')
            
        finally:
            # Save final checkpoint
            print('\nSaving final checkpoint...')
            self.save_checkpoint(self.episode)
            
            # Cleanup
            self.log_file.close()
            env.destroy_node()
            rclpy.shutdown()
            
        print(f'\nTraining completed: {self.episode} episodes, {self.total_steps} total steps')
        print(f'Session saved to: {self.session_dir}')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='TD3 Training for SAR Room Navigation')
    parser.add_argument('--episodes', type=int, default=10000,
                       help='Number of episodes to train (default: 10000)')
    parser.add_argument('--checkpoint-dir', type=str, default='./checkpoints/td3_sar',
                       help='Directory for checkpoints (default: ./checkpoints/td3_sar)')
    parser.add_argument('--load-model', type=str, default=None,
                       help='Path to model to load for continuing training')
    parser.add_argument('--start-episode', type=int, default=0,
                       help='Starting episode number (for resuming training)')
    
    args = parser.parse_args()
    
    # Create trainer and start training
    trainer = TD3Trainer(
        checkpoint_dir=args.checkpoint_dir,
        load_model=args.load_model,
        start_episode=args.start_episode
    )
    
    trainer.train(num_episodes=args.episodes)


if __name__ == '__main__':
    main()
