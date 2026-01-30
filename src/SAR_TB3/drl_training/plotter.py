"""
Live plotting for training metrics.
Creates matplotlib plots showing success rate, outcomes, losses, and rewards.
"""
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from .settings import (
    SUCCESS, GRAPH_DRAW_INTERVAL, GRAPH_AVERAGE_REWARD,
    OUTCOME_LABELS
)

# Try TkAgg first, fallback to Agg if display not available
try:
    matplotlib.use('TkAgg')
    print("Using TkAgg backend for interactive plotting")
except Exception as e:
    print(f"TkAgg not available ({e}), using Agg backend (plots saved to file only)")
    matplotlib.use('Agg')


class TrainingPlotter:
    """Live training visualization with matplotlib."""
    
    def __init__(self, save_dir):
        """
        Initialize plotter.
        
        Args:
            save_dir: Directory to save plot images
        """
        self.interactive = matplotlib.get_backend() == 'TkAgg'
        
        if self.interactive:
            try:
                plt.ion()  # Interactive mode
                plt.show()
                print("Matplotlib interactive mode enabled - plots will appear in a window")
            except Exception as e:
                print(f"Could not enable interactive mode: {e}")
                print("Plots will be saved to file only")
                self.interactive = False
        else:
            print("Non-interactive mode - plots will be saved to file only")
        
        self.save_dir = save_dir
        self.legend_labels = list(OUTCOME_LABELS.values())
        self.legend_colors = ['b', 'g', 'r', 'c', 'm', 'y']
        
        self.outcome_histories = []
        
        # Data storage
        self.global_steps = 0
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        
        # Create figure with 4 subplots
        self.fig, self.ax = plt.subplots(2, 2, figsize=(18.5, 10.5))
        
        titles = [
            'Outcomes per Episode',
            'Avg Critic Loss per Episode',
            'Avg Actor Loss per Episode',
            'Avg Reward over 10 Episodes'
        ]
        
        for i in range(4):
            ax = self.ax[int(i/2)][int(i%2!=0)]
            ax.set_title(titles[i], fontsize=14, fontweight='bold')
            ax.set_xlabel('Episode', fontsize=12)
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax.grid(True, alpha=0.3)
            
        self.ax[0][0].set_ylabel('Count', fontsize=12)
        self.ax[0][1].set_ylabel('Loss', fontsize=12)
        self.ax[1][0].set_ylabel('Loss', fontsize=12)
        self.ax[1][1].set_ylabel('Reward', fontsize=12)
        
        self.legend_set = False
        
    def update_data(self, episode, step, global_steps, outcome, reward_sum, 
                    loss_critic_sum, loss_actor_sum):
        """
        Update data with new episode results.
        
        Args:
            episode: Current episode number
            step: Steps in this episode
            global_steps: Total steps across all episodes
            outcome: Episode outcome (SUCCESS, COLLISION, etc.)
            reward_sum: Total reward for episode
            loss_critic_sum: Sum of critic losses
            loss_actor_sum: Sum of actor losses
        """
        self.global_steps = global_steps
        self.data_outcome_history.append(outcome)
        self.data_rewards.append(reward_sum)
        self.data_loss_critic.append(loss_critic_sum / step if step > 0 else 0)
        self.data_loss_actor.append(loss_actor_sum / step if step > 0 else 0)
        
    def draw_plots(self, episode):
        """
        Draw/update all plots.
        
        Args:
            episode: Current episode number
        """
        if episode == 0 or len(self.data_outcome_history) == 0:
            return
            
        # Clear all axes
        for i in range(4):
            ax = self.ax[int(i/2)][int(i%2!=0)]
            ax.clear()
            
        num_episodes = len(self.data_outcome_history)
        xaxis = np.array(range(1, num_episodes + 1))
        
        # Plot 1: Outcome history (stacked area chart)
        # Rebuild outcome histories from data each time
        outcome_histories = [[0] * num_episodes for _ in range(6)]
        
        for idx, outcome in enumerate(self.data_outcome_history):
            if 0 <= outcome < 6:  # Validate outcome is in valid range
                for i in range(6):
                    if i < outcome:
                        outcome_histories[i][idx] = outcome_histories[i][idx-1] if idx > 0 else 0
                    elif i == outcome:
                        outcome_histories[i][idx] = (outcome_histories[i][idx-1] if idx > 0 else 0) + 1
                    else:
                        outcome_histories[i][idx] = outcome_histories[i][idx-1] if idx > 0 else 0
                
        if num_episodes > 0:
            for i, outcome_history in enumerate(outcome_histories):
                if any(outcome_history):  # Only plot if there's data
                    self.ax[0][0].plot(xaxis, outcome_history, 
                                      color=self.legend_colors[i], 
                                      label=self.legend_labels[i],
                                      linewidth=2)
            if not self.legend_set:
                self.ax[0][0].legend(loc='upper left')
                self.legend_set = True
        
        # Add success rate line on secondary y-axis
        ax_twin = self.ax[0][0].twinx()
        success_rate = []
        for idx in range(num_episodes):
            total_so_far = max(1, idx + 1)
            success_so_far = outcome_histories[1][idx] if len(outcome_histories) > 1 else 0
            rate = (success_so_far / total_so_far) * 100.0
            success_rate.append(rate)
        
        ax_twin.plot(xaxis, success_rate, 'b--', linewidth=2, label='Success Rate %')
        ax_twin.set_ylabel('Success Rate (%)', fontsize=12, color='blue')
        ax_twin.tick_params(axis='y', labelcolor='blue')
        ax_twin.set_ylim([0, 100])
        ax_twin.legend(loc='upper right')
                
        self.ax[0][0].set_title('Outcomes per Episode & Success Rate', fontsize=14, fontweight='bold')
        self.ax[0][0].set_xlabel('Episode', fontsize=12)
        self.ax[0][0].set_ylabel('Count', fontsize=12)
        self.ax[0][0].grid(True, alpha=0.3)
        
        # Plot 2: Critic loss
        if len(self.data_loss_critic) > 0:
            y = np.array(self.data_loss_critic)
            self.ax[0][1].plot(xaxis, y, 'b-', linewidth=1.5)
            self.ax[0][1].set_title('Avg Critic Loss per Episode', fontsize=14, fontweight='bold')
            self.ax[0][1].set_xlabel('Episode', fontsize=12)
            self.ax[0][1].set_ylabel('Loss', fontsize=12)
            self.ax[0][1].grid(True, alpha=0.3)
            
        # Plot 3: Actor loss
        if len(self.data_loss_actor) > 0:
            y = np.array(self.data_loss_actor)
            self.ax[1][0].plot(xaxis, y, 'r-', linewidth=1.5)
            self.ax[1][0].set_title('Avg Actor Loss per Episode', fontsize=14, fontweight='bold')
            self.ax[1][0].set_xlabel('Episode', fontsize=12)
            self.ax[1][0].set_ylabel('Loss', fontsize=12)
            self.ax[1][0].grid(True, alpha=0.3)
            
        # Plot 4: Average reward over last N episodes
        num_data_points = len(self.data_rewards)
        count = int(num_data_points / GRAPH_AVERAGE_REWARD)
        if count > 0 and num_data_points >= GRAPH_AVERAGE_REWARD:
            xaxis_avg = np.array(range(GRAPH_AVERAGE_REWARD, count * GRAPH_AVERAGE_REWARD + 1, GRAPH_AVERAGE_REWARD))
            averages = []
            for i in range(count):
                avg_sum = 0
                for j in range(GRAPH_AVERAGE_REWARD):
                    idx = i * GRAPH_AVERAGE_REWARD + j
                    if idx < num_data_points:
                        avg_sum += self.data_rewards[idx]
                averages.append(avg_sum / GRAPH_AVERAGE_REWARD)
            y = np.array(averages)
            self.ax[1][1].plot(xaxis_avg, y, 'g-', linewidth=2, marker='o')
            self.ax[1][1].set_title(f'Avg Reward over {GRAPH_AVERAGE_REWARD} Episodes', 
                                   fontsize=14, fontweight='bold')
            self.ax[1][1].set_xlabel('Episode', fontsize=12)
            self.ax[1][1].set_ylabel('Reward', fontsize=12)
            self.ax[1][1].grid(True, alpha=0.3)
            
        plt.tight_layout()
        
        # Save figure first (always works)
        save_path = os.path.join(self.save_dir, '_training_figure.png')
        plt.savefig(save_path, dpi=100, bbox_inches='tight')
        
        # Update interactive display if available
        if self.interactive:
            try:
                plt.draw()
                plt.pause(0.001)  # Very short pause to update
                self.fig.canvas.flush_events()
            except Exception as e:
                # Silently fail if display update doesn't work
                pass
        
    def get_success_count(self):
        """Get number of successes in last GRAPH_DRAW_INTERVAL episodes."""
        if len(self.data_outcome_history) < GRAPH_DRAW_INTERVAL:
            recent = self.data_outcome_history
        else:
            recent = self.data_outcome_history[-GRAPH_DRAW_INTERVAL:]
        return recent.count(SUCCESS)
        
    def get_reward_average(self):
        """Get average reward over last GRAPH_DRAW_INTERVAL episodes."""
        if len(self.data_rewards) < GRAPH_DRAW_INTERVAL:
            recent = self.data_rewards
        else:
            recent = self.data_rewards[-GRAPH_DRAW_INTERVAL:]
        return sum(recent) / len(recent) if len(recent) > 0 else 0.0
        
    def save_data(self, path):
        """Save plot data to file."""
        np.savez(path,
                 global_steps=self.global_steps,
                 outcome_history=self.data_outcome_history,
                 rewards=self.data_rewards,
                 loss_critic=self.data_loss_critic,
                 loss_actor=self.data_loss_actor)
                 
    def load_data(self, path):
        """Load plot data from file."""
        data = np.load(path)
        self.global_steps = int(data['global_steps'])
        self.data_outcome_history = list(data['outcome_history'])
        self.data_rewards = list(data['rewards'])
        self.data_loss_critic = list(data['loss_critic'])
        self.data_loss_actor = list(data['loss_actor'])
