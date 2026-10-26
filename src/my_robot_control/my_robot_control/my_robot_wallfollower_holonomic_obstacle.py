#!/usr/bin/env python3
"""Holonomic obstacle wall follower for the mecanum rUBot.

/scan (LaserScan) + /odom (Odometry) -> state machine -> /cmd_vel (Twist)

States (named after the LiDAR region that drives them):
    RIGHT        follow the right wall (vx, vy distance, wz alignment)
    FRONT        wall ahead: move left (+vy) and measure lateral progress
    FRONT_RIGHT  short obstacle: diagonal +vx/+vy | long wall: rotate left
    BACK_RIGHT   finish the bypass: diagonal +vx/-vy until RIGHT is recovered

Short obstacle sequence:
    RIGHT -> FRONT -> FRONT_RIGHT -> RIGHT (angular.z = 0, passing flag)
          -> BACK_RIGHT -> RIGHT
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicObstacleWallFollower(Node):
    """Reactive holonomic wall follower organised as a state machine."""

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        # Desired distance between the robot and the right wall (m).
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        # Maximum lateral distance used to classify a short obstacle (m).
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        # Forward velocity (m/s).
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.20).value
        )
        # Maximum lateral velocity (m/s).
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.20).value
        )
        # Turn speed and maximum angular correction (rad/s).
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 1.00).value
        )
        # Maximum LiDAR distance considered (m).
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )
        # Proportional gain for lateral-distance correction.
        self.lateral_gain = float(
            self.declare_parameter('lateral_gain', 2.0).value
        )
        # Proportional gain for angular alignment.
        self.alignment_gain = float(
            self.declare_parameter('alignment_gain', 2.0).value
        )

        self.RIGHT = 'RIGHT'
        self.FRONT = 'FRONT'
        self.FRONT_RIGHT = 'FRONT_RIGHT'
        self.BACK_RIGHT = 'BACK_RIGHT'

        self.diagonal_factor = 0.5
        self.region_confirmations = 3
        self.robust_rays = 3
        self.scan_timeout = 0.5          # s without /scan -> stop

        self.state = self.RIGHT
        self.obstacle_is_long = False
        self.passing_short_obstacle = False
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.right_angle = None
        self.scan_stamp = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info('Holonomic obstacle wall follower started.')

    def publish_command(self):
        """Publish the last command (zero if /scan is missing or stale)."""
        if self.scan_stamp is None:
            self.publisher.publish(Twist())
            return
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        if age > self.scan_timeout:
            self.get_logger().warn(
                '/scan is stale: stopping.', throttle_duration_sec=2.0
            )
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        """Store planar pose (x, y, yaw) from odometry."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Yaw from the quaternion (no tf_transformations dependency).
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        self.odom_pose = (position.x, position.y, yaw)

    # ------------------------------------------------------------------
    # LiDAR processing
    # ------------------------------------------------------------------
    def region_measurements(self, scan):
        """Return robust distances and angles for the four LiDAR regions.

        Result: {region: (distance_m, angle_deg)}; (math.inf, None) if the
        region has no valid ray.
        """
        # TODO 1: one list per region, each valid ray stored as
        # (distance, angle_deg).
        rays = {
            self.FRONT: [],
            self.FRONT_RIGHT: [],
            self.RIGHT: [],
            self.BACK_RIGHT: [],
        }

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

            if -20.0 <= deg <= 20.0:
                rays[self.FRONT].append((r, deg))
            elif -70.0 <= deg < -20.0:
                rays[self.FRONT_RIGHT].append((r, deg))
            elif -110.0 <= deg < -70.0:
                rays[self.RIGHT].append((r, deg))
            elif -160.0 <= deg < -110.0:
                rays[self.BACK_RIGHT].append((r, deg))

        # TODO 2: mean distance and angle of the nearest rays per region.
        measurements = {}
        for name, values in rays.items():
            if not values:
                measurements[name] = (math.inf, None)
                continue
            nearest = sorted(values)[:self.robust_rays]
            distance = sum(v[0] for v in nearest) / len(nearest)
            angle_deg = sum(v[1] for v in nearest) / len(nearest)
            measurements[name] = (distance, angle_deg)
        return measurements

    def closest_region(self, measurements):
        """Return the closest region inside detection_limit (or None)."""
        # TODO 3: ignore regions that are not below detection_limit.
        best_name = None
        best_distance = math.inf
        for name, (distance, _) in measurements.items():
            if distance < self.detection_limit and distance < best_distance:
                best_name = name
                best_distance = distance
        return best_name

    def confirmed_region(self, measurements):
        """Confirm a region change over several consecutive scans."""
        # TODO 4: the closest region must repeat region_confirmations times
        # before it replaces stable_region.
        closest = self.closest_region(measurements)
        if closest == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = closest
            self.region_candidate_count = 1
        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = self.region_candidate
        return self.stable_region

    def lateral_progress(self):
        """Return travel along the robot's initial lateral axis."""
        # TODO 5
        if self.odom_pose is None or self.front_start_pose is None:
            return 0.0
        x0, y0, yaw0 = self.front_start_pose
        x, y, _ = self.odom_pose
        dx = x - x0
        dy = y - y0
        return abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    # ------------------------------------------------------------------
    # State behaviours. Each one fills `cmd` in place; a zero cmd = stop.
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
            self.state = new_state

    def start_front(self):
        """Enter FRONT saving the odometry pose; False if no odom yet."""
        if self.odom_pose is None:
            self.get_logger().warn(
                'Frontal wall but no /odom yet: waiting.',
                throttle_duration_sec=2.0,
            )
            return False
        self.front_start_pose = self.odom_pose
        self.obstacle_is_long = False
        self.passing_short_obstacle = False
        self.set_state(self.FRONT)
        return True

    def follow_right(self, measurements, closest, cmd):
        """Follow the wall using lateral and angular correction."""
        # TODO 6
        if closest == self.FRONT:
            if self.start_front():
                self.follow_front(measurements, closest, cmd)
            return

        if self.passing_short_obstacle and closest == self.BACK_RIGHT:
            self.set_state(self.BACK_RIGHT)
            self.follow_back_right(measurements, closest, cmd)
            return

        distance, angle_deg = measurements[self.RIGHT]
        if angle_deg is None or distance >= self.detection_limit:
            self.get_logger().warn(
                'Right region lost: stopping.', throttle_duration_sec=2.0
            )
            return

        self.right_angle = angle_deg
        cmd.linear.x = self.forward_speed
        # Too far -> move right (-vy); too close -> move left (+vy).
        cmd.linear.y = clamp(
            -self.lateral_gain * (distance - self.follow_distance),
            -self.lateral_speed, self.lateral_speed,
        )
        if self.passing_short_obstacle:
            cmd.angular.z = 0.0
        else:
            # Move the nearest right point towards -90 degrees.
            cmd.angular.z = clamp(
                self.alignment_gain * math.radians(angle_deg + 90.0),
                -self.turn_speed, self.turn_speed,
            )

    def follow_front(self, measurements, closest, cmd):
        """Move laterally and classify the obstacle as short or long."""
        # TODO 7
        if self.odom_pose is None or self.front_start_pose is None:
            return

        progress = self.lateral_progress()

        if closest == self.FRONT:
            if progress >= self.max_front_length:
                self.get_logger().info(
                    f'Long wall (lateral progress {progress:.2f} m): turning.'
                )
                self.obstacle_is_long = True
                self.set_state(self.FRONT_RIGHT)
                self.follow_front_right(measurements, closest, cmd)
                return
            distance, angle_deg = measurements[self.FRONT]
            cmd.linear.x = clamp(
                self.lateral_gain * (distance - self.follow_distance),
                -self.forward_speed, self.forward_speed,
            )
            cmd.linear.y = self.lateral_speed          # move left
            cmd.angular.z = clamp(
                self.alignment_gain * math.radians(angle_deg),
                -self.turn_speed, self.turn_speed,
            )
            return

        if closest is None:
            return   # nothing valid: stop

        # Front cleared before max_front_length: short obstacle.
        self.get_logger().info(
            f'Short obstacle (lateral progress {progress:.2f} m): bypass.'
        )
        self.passing_short_obstacle = True
        if closest == self.FRONT_RIGHT:
            self.set_state(self.FRONT_RIGHT)
            self.follow_front_right(measurements, closest, cmd)
        elif closest == self.BACK_RIGHT:
            self.set_state(self.BACK_RIGHT)
            self.follow_back_right(measurements, closest, cmd)
        else:
            self.set_state(self.RIGHT)
            self.follow_right(measurements, closest, cmd)

    def follow_front_right(self, measurements, closest, cmd):
        """Move diagonally around the front-right side."""
        # TODO 8
        if self.obstacle_is_long:
            # Long wall: rotate left until it is in the RIGHT region.
            if closest == self.RIGHT:
                self.get_logger().info('New right wall found.')
                self.obstacle_is_long = False
                self.passing_short_obstacle = False
                self.set_state(self.RIGHT)
                self.follow_right(measurements, closest, cmd)
                return
            cmd.angular.z = self.turn_speed
            return

        if closest == self.RIGHT:
            self.set_state(self.RIGHT)         # passing flag stays True
            self.follow_right(measurements, closest, cmd)
        elif closest == self.BACK_RIGHT:
            self.set_state(self.BACK_RIGHT)
            self.follow_back_right(measurements, closest, cmd)
        elif closest == self.FRONT:
            if self.start_front():
                self.follow_front(measurements, closest, cmd)
        elif closest == self.FRONT_RIGHT:
            # Slow diagonal +vx/+vy, angular.z = 0.
            cmd.linear.x = self.diagonal_factor * self.forward_speed
            cmd.linear.y = self.diagonal_factor * self.lateral_speed

    def follow_back_right(self, measurements, closest, cmd):
        """Complete the bypass and recover the right wall."""
        # TODO 9
        if closest == self.RIGHT:
            self.get_logger().info('Right wall recovered.')
            self.passing_short_obstacle = False
            self.set_state(self.RIGHT)
            self.follow_right(measurements, closest, cmd)
        elif closest == self.FRONT:
            if self.start_front():
                self.follow_front(measurements, closest, cmd)
        elif closest is not None:
            # Slow diagonal +vx/-vy, angular.z = 0.
            cmd.linear.x = self.diagonal_factor * self.forward_speed
            cmd.linear.y = -self.diagonal_factor * self.lateral_speed

    def scan_callback(self, scan):
        """Read the LiDAR and calculate the next velocity command."""
        # TODO 10
        self.scan_stamp = self.get_clock().now()
        measurements = self.region_measurements(scan)
        closest = self.confirmed_region(measurements)
        cmd = Twist()

        if closest is None:
            self.get_logger().warn(
                'No valid LiDAR region: stopping.',
                throttle_duration_sec=2.0,
            )
        elif self.state == self.RIGHT:
            self.follow_right(measurements, closest, cmd)
        elif self.state == self.FRONT:
            self.follow_front(measurements, closest, cmd)
        elif self.state == self.FRONT_RIGHT:
            self.follow_front_right(measurements, closest, cmd)
        elif self.state == self.BACK_RIGHT:
            self.follow_back_right(measurements, closest, cmd)

        self.command = cmd

    def stop(self):
        """Publish a final zero command."""
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = HolonomicObstacleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
