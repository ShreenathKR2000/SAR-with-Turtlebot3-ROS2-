#!/usr/bin/env python3
"""
Laser Scan Adapter Node.
Adapts different laser scan sizes to a standardized format for TD3 models.

Handles conversion between:
- Real TurtleBot3 LDS-01: 223 samples
- Gazebo simulation: 360 samples
- TD3 model expectation: 24 bins (flexible to input size)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from sensor_msgs.msg import LaserScan


class LaserScanAdapter(Node):
    """
    Adapts laser scan data between different resolutions.
    
    Real TurtleBot3 (LDS-01) publishes 223 samples at ~360 degrees.
    Gazebo simulation publishes 360 samples.
    
    This node can:
    1. Pass through unchanged (if target_samples = 0)
    2. Interpolate to target number of samples
    3. Downsample to target number of samples
    """
    
    def __init__(self):
        super().__init__('laser_scan_adapter')
        
        # Parameters
        self.declare_parameter('input_scan_topic', '/scan_raw')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('target_samples', 360)  # 0 = passthrough, >0 = resample
        self.declare_parameter('interpolation_method', 'linear')  # 'linear' or 'nearest'
        
        input_topic = self.get_parameter('input_scan_topic').value
        output_topic = self.get_parameter('output_scan_topic').value
        self.target_samples = self.get_parameter('target_samples').value
        self.interp_method = self.get_parameter('interpolation_method').value
        
        # QoS for laser scan (best effort for real-time data)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            scan_qos
        )
        
        # Publisher
        self.scan_pub = self.create_publisher(
            LaserScan,
            output_topic,
            scan_qos
        )
        
        self.get_logger().info(f'Laser Scan Adapter initialized')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Target samples: {self.target_samples} (0=passthrough)')
        self.get_logger().info(f'  Interpolation: {self.interp_method}')
        
        self.first_msg = True
    
    def interpolate_scan(self, ranges, target_size):
        """
        Interpolate laser scan to target number of samples.
        
        Args:
            ranges: Input range array
            target_size: Desired number of samples
            
        Returns:
            Interpolated range array
        """
        if len(ranges) == target_size:
            return ranges
        
        # Create indices
        input_indices = np.linspace(0, len(ranges) - 1, len(ranges))
        output_indices = np.linspace(0, len(ranges) - 1, target_size)
        
        # Interpolate
        if self.interp_method == 'linear':
            interpolated = np.interp(output_indices, input_indices, ranges)
        else:  # nearest
            nearest_indices = np.round(output_indices).astype(int)
            nearest_indices = np.clip(nearest_indices, 0, len(ranges) - 1)
            interpolated = ranges[nearest_indices]
        
        return interpolated
    
    def scan_callback(self, msg):
        """Process incoming scan and republish."""
        if self.first_msg:
            self.get_logger().info(
                f'Received first scan: {len(msg.ranges)} samples, '
                f'angle range: {msg.angle_min:.2f} to {msg.angle_max:.2f} rad'
            )
            self.first_msg = False
        
        # If target_samples is 0 or matches input, pass through
        if self.target_samples == 0 or self.target_samples == len(msg.ranges):
            self.scan_pub.publish(msg)
            return
        
        # Create output message (copy metadata)
        output_msg = LaserScan()
        output_msg.header = msg.header
        output_msg.angle_min = msg.angle_min
        output_msg.angle_max = msg.angle_max
        output_msg.range_min = msg.range_min
        output_msg.range_max = msg.range_max
        output_msg.time_increment = msg.time_increment * len(msg.ranges) / self.target_samples
        output_msg.scan_time = msg.scan_time
        
        # Adjust angle_increment for new sample count
        output_msg.angle_increment = (msg.angle_max - msg.angle_min) / (self.target_samples - 1)
        
        # Convert to numpy and handle inf values
        ranges = np.array(msg.ranges, dtype=np.float32)
        intensities = np.array(msg.intensities, dtype=np.float32) if msg.intensities else None
        
        # Replace inf with max_range for interpolation
        inf_mask = np.isinf(ranges)
        ranges[inf_mask] = msg.range_max
        
        # Interpolate ranges
        output_ranges = self.interpolate_scan(ranges, self.target_samples)
        
        # Interpolate intensities if present
        if intensities is not None and len(intensities) == len(ranges):
            output_intensities = self.interpolate_scan(intensities, self.target_samples)
            output_msg.intensities = output_intensities.tolist()
        
        output_msg.ranges = output_ranges.tolist()
        
        # Publish
        self.scan_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
