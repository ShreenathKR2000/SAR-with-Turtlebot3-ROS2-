#!/usr/bin/env python3
"""
Packaged Autonomous Explorer (copied from ai_rescuebot)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        self.declare_parameter('enable_td3_control', True)
        self.declare_parameter('exploration_timeout', 60.0)
        self.declare_parameter('retry_on_failure', True)
        self.declare_parameter('max_retries', 3)

        self.use_td3 = self.get_parameter('enable_td3_control').value
        self.timeout = self.get_parameter('exploration_timeout').value
        self.retry_on_failure = self.get_parameter('retry_on_failure').value
        self.max_retries = self.get_parameter('max_retries').value

        self.current_frontier = None
        self.exploration_active = False
        self.retry_count = 0
        self.map_received = False

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.frontier_sub = self.create_subscription(PoseStamped, '/frontier_goal', self.frontier_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_action_client.wait_for_server()
        self.get_logger().info('Nav2 action server ready')

        self.status_timer = self.create_timer(5.0, self.publish_status)
        self.get_logger().info('Packaged Autonomous Explorer initialized')
        self.publish_status_msg('Ready - waiting for map and frontiers')

    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info('Received first map from SLAM')
            self.map_received = True

    def frontier_callback(self, msg):
        self.current_frontier = msg
        if not self.exploration_active:
            self.start_exploration(msg)

    def start_exploration(self, goal_pose):
        if not self.map_received:
            self.get_logger().warn('No map received yet, cannot navigate')
            return
        self.exploration_active = True
        self.retry_count = 0
        self.get_logger().info(f'Starting exploration to frontier: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        send_goal_future = self.nav_action_client.send_goal_async(nav_goal, feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        self.publish_status_msg(f'Navigating to frontier ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self.exploration_active = False
            self.publish_status_msg('Goal rejected by Nav2')
            return
        self.get_logger().info('Nav2 goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        pass

    def nav_result_callback(self, future):
        result = future.result()
        self.exploration_active = False
        if result.status == 4:
            self.get_logger().info('Successfully reached frontier!')
            self.publish_status_msg('Frontier reached - searching for next frontier')
            self.retry_count = 0
        elif result.status == 5:
            self.get_logger().warn('Navigation was canceled')
            self.publish_status_msg('Navigation canceled')
        elif result.status == 6:
            self.get_logger().warn('Navigation aborted')
            if self.retry_on_failure and self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Retrying... (attempt {self.retry_count}/{self.max_retries})')
                self.publish_status_msg(f'Retrying navigation (attempt {self.retry_count})')
                self.create_timer(2.0, self.retry_navigation, oneshot=True)
            else:
                self.get_logger().warn('Max retries reached or retry disabled, waiting for new frontier')
                self.publish_status_msg('Navigation failed - waiting for new frontier')
        else:
            self.get_logger().warn(f'Navigation ended with status: {result.status}')
            self.publish_status_msg(f'Navigation status: {result.status}')

    def retry_navigation(self):
        if self.current_frontier is not None and not self.exploration_active:
            self.start_exploration(self.current_frontier)

    def publish_status(self):
        if self.exploration_active:
            status = 'EXPLORING'
        elif not self.map_received:
            status = 'WAITING_FOR_MAP'
        else:
            status = 'IDLE'
        self.publish_status_msg(status)

    def publish_status_msg(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
