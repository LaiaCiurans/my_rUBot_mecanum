#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator


class TrafficSignNavigation(Node):

    def __init__(self):
        super().__init__('traffic_sign_navigation')

        #self.declare_parameter('use_sim_time', True)
        self.declare_parameter('initial_pose', [-0.5, 0.0, 0.0])
        self.declare_parameter('inspection_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('left_waypoint', [0.5, 1.0, 0.0])
        self.declare_parameter('right_waypoint', [0.5, -1.0, 0.0])
        self.declare_parameter('final_pose', [1.0, 0.0, 1.57])
        self.declare_parameter('stop_duration', 3.0)

        self.initial_pose = self.get_parameter('initial_pose').value
        self.inspection_pose = self.get_parameter('inspection_pose').value
        self.left_waypoint = self.get_parameter('left_waypoint').value
        self.right_waypoint = self.get_parameter('right_waypoint').value
        self.final_pose = self.get_parameter('final_pose').value
        self.stop_duration = self.get_parameter('stop_duration').value

        self.navigator = BasicNavigator()
        self.detected_sign = None
        self.accept_sign = False
        self.sign_armed = True

        self.subscription = self.create_subscription(
            String, '/traffic_sign', self.sign_callback, 10
        )

    def sign_callback(self, msg):
        sign = msg.data.strip().lower()

        # A "none" message rearms the node for the next sign.
        if sign == 'none':
            self.sign_armed = True
            self.get_logger().info('Ready to accept the next sign.')
            return

        if sign not in ('left', 'right', 'stop'):
            return

        if self.accept_sign and self.sign_armed and self.detected_sign is None:
            self.detected_sign = sign
            self.sign_armed = False
            self.get_logger().info(f'Received traffic sign: {sign}')

    def make_pose(self, pose_values):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_values[0])
        pose.pose.position.y = float(pose_values[1])

        yaw = float(pose_values[2])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def go_to(self, pose_values, description):
        self.get_logger().info(f'Navigating to {description}: {pose_values[0]}, {pose_values[1]}')
        self.navigator.goToPose(self.make_pose(pose_values))

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        self.get_logger().info(f'Navigation to {description} finished: {result}')
        return result

    def wait_for_sign(self):
        self.detected_sign = None
        self.accept_sign = True
        self.get_logger().info('Waiting for a new sign. Remove the previous sign first.')

        while rclpy.ok() and self.detected_sign is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.accept_sign = False
        return self.detected_sign

    def run(self):
        self.navigator.setInitialPose(self.make_pose(self.initial_pose))
        self.get_logger().info('Publishing Initial Pose')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready for use!')

        self.go_to(self.inspection_pose, 'inspection pose')

        while rclpy.ok():
            sign = self.wait_for_sign()
            if sign is None:
                break

            if sign == 'left':
                self.go_to(self.left_waypoint, 'left waypoint')
            elif sign == 'right':
                self.go_to(self.right_waypoint, 'right waypoint')
            elif sign == 'stop':
                self.get_logger().info(f'STOP sign: waiting {self.stop_duration} seconds')
                time.sleep(float(self.stop_duration))

            self.go_to(self.final_pose, 'final pose')
            self.go_to(self.inspection_pose, 'inspection pose')
            self.get_logger().info('Waiting for the next sign. Publish "none" before the next sign.')


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNavigation()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
