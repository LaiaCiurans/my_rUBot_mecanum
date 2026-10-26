#!/usr/bin/env python3
"""Improved holonomic wall follower for the mecanum rUBot.

/scan (LaserScan) + /odom (Odometry) -> state machine -> /cmd_vel (Twist)

States:
    FOLLOW_RIGHT        follow the right wall (vx, vy distance, wz alignment)
    FOLLOW_FRONT        frontal wall: move left (+vy) and measure lateral
                        progress with odometry
    TURN_TO_RIGHT       long wall: rotate left until it is in the right region
    BYPASS_FRONT_RIGHT  short obstacle: slow diagonal +vx/+vy until RIGHT
    BYPASS_RIGHT        slow +vx with proportional vy until BACK_RIGHT
    BYPASS_BACK_RIGHT   slow diagonal +vx/-vy until the right wall is back

Short obstacle:
    FOLLOW_RIGHT -> FOLLOW_FRONT -> BYPASS_FRONT_RIGHT -> BYPASS_RIGHT
                 -> BYPASS_BACK_RIGHT -> FOLLOW_RIGHT
Long wall:
    FOLLOW_RIGHT -> FOLLOW_FRONT -> TURN_TO_RIGHT -> FOLLOW_RIGHT
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicWallFollowerImproved(Node):
    """Reactive holonomic wall follower organised as a state machine."""

    # LiDAR regions
    FRONT = 'front'
    FRONT_RIGHT = 'front_right'
    RIGHT = 'right'
    BACK_RIGHT = 'back_right'

    # Controller states
    S_FOLLOW_RIGHT = 'FOLLOW_RIGHT'
    S_FOLLOW_FRONT = 'FOLLOW_FRONT'
    S_TURN_TO_RIGHT = 'TURN_TO_RIGHT'
    S_BYPASS_FRONT_RIGHT = 'BYPASS_FRONT_RIGHT'
    S_BYPASS_RIGHT = 'BYPASS_RIGHT'
    S_BYPASS_BACK_RIGHT = 'BYPASS_BACK_RIGHT'

    def __init__(self):
        super().__init__('holonomic_wall_follower_improved')

        # Desired distance to the wall (m).
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value)
        # Maximum frontal-obstacle length (m), measured with odometry.
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value)
        # Normal forward velocity (m/s).
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.30).value)
        # Maximum magnitude of vy (m/s).
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.40).value)
        # Proportional gain for the distance correction.
        self.distance_gain = float(
            self.declare_parameter('distance_gain', 0.80).value)
        # Proportional gain for the angular correction.
        self.angle_gain = float(
            self.declare_parameter('angle_gain', 1.20).value)
        # Angular velocity saturation (rad/s).
        self.max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 1.00).value)
        # Maximum relevant LiDAR distance (m).
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value)
        # Back-right distance factor that triggers a stronger -vy.
        self.back_right_limit_factor = float(
            self.declare_parameter('back_right_limit_factor', 1.50).value)

        # Derived values (no extra parameters needed).
        self.front_threshold = 1.5 * self.follow_distance
        self.bypass_vx = 0.5 * self.forward_speed
        self.bypass_vy = 0.25 * self.lateral_speed

        self.region_confirmations = 3
        self.robust_rays = 3
        self.scan_timeout = 0.5  # s without /scan -> stop

        self.state = self.S_FOLLOW_RIGHT
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.scan_stamp = None

        # Debounce of the closest region and of the frontal obstacle.
        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None
        self.front_candidate = False
        self.front_candidate_count = 0
        self.front_blocked = False

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info('Improved holonomic wall follower started.')

    # ------------------------------------------------------------------
    # Publishing and odometry
    # ------------------------------------------------------------------
    def publish_command(self):
        """Publish the last command (zero if /scan is missing or stale)."""
        if self.scan_stamp is None:
            self.publisher.publish(Twist())
            return
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        if age > self.scan_timeout:
            self.get_logger().warn(
                '/scan is stale: stopping.', throttle_duration_sec=2.0)
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        """Store planar pose (x, y, yaw) from odometry."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    # ------------------------------------------------------------------
    # LiDAR processing
    # ------------------------------------------------------------------
    def region_measurements(self, scan):
        """Return {region: (distance_m, angle_deg)}; (inf, None) if empty."""
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
        """Closest region below detection_limit (or None)."""
        best_name = None
        best_distance = math.inf
        for name, (distance, _) in measurements.items():
            if distance < self.detection_limit and distance < best_distance:
                best_name = name
                best_distance = distance
        return best_name

    def confirmed_region(self, measurements):
        """The closest region must repeat several scans to be accepted."""
        closest = self.closest_region(measurements)
        if closest == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = closest
            self.region_candidate_count = 1
        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = self.region_candidate
        return self.stable_region

    def confirmed_front_blocked(self, measurements):
        """Frontal obstacle: front distance < 1.5 * follow_distance."""
        raw = measurements[self.FRONT][0] < self.front_threshold
        if raw == self.front_candidate:
            self.front_candidate_count += 1
        else:
            self.front_candidate = raw
            self.front_candidate_count = 1
        if self.front_candidate_count >= self.region_confirmations:
            self.front_blocked = self.front_candidate
        return self.front_blocked

    def lateral_progress(self):
        """Travel along the robot's initial lateral axis (m)."""
        if self.odom_pose is None or self.front_start_pose is None:
            return 0.0
        x0, y0, yaw0 = self.front_start_pose
        x, y, _ = self.odom_pose
        dx = x - x0
        dy = y - y0
        return abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
            self.state = new_state

    def start_front(self):
        """Enter FOLLOW_FRONT saving the odometry pose; False if no /odom."""
        if self.odom_pose is None:
            self.get_logger().warn(
                'Frontal wall but no /odom yet: waiting.',
                throttle_duration_sec=2.0)
            return False
        self.front_start_pose = self.odom_pose
        self.set_state(self.S_FOLLOW_FRONT)
        return True

    def follow_right(self, m, closest, cmd):
        """Normal right-wall following."""
        if self.front_blocked:
            if self.start_front():
                self.follow_front(m, closest, cmd)
            return

        distance, angle_deg = m[self.RIGHT]
        if angle_deg is None or distance >= self.detection_limit:
            self.get_logger().warn(
                'Right region lost: stopping.', throttle_duration_sec=2.0)
            return

        cmd.linear.x = self.forward_speed
        # Too far -> move right (-vy); too close -> move left (+vy).
        cmd.linear.y = -self.distance_gain * (distance - self.follow_distance)
        # Keep the nearest right point centred around -90 degrees.
        cmd.angular.z = self.angle_gain * math.radians(angle_deg + 90.0)

    def follow_front(self, m, closest, cmd):
        """Move left along the frontal wall; short obstacle or long wall."""
        if self.odom_pose is None or self.front_start_pose is None:
            return

        progress = self.lateral_progress()

        if self.front_blocked:
            if progress >= self.max_front_length:
                self.get_logger().info(
                    f'Long wall (lateral progress {progress:.2f} m).')
                self.set_state(self.S_TURN_TO_RIGHT)
                self.turn_to_right(m, closest, cmd)
                return
            distance, angle_deg = m[self.FRONT]
            if angle_deg is None:
                return
            cmd.linear.x = self.distance_gain * (
                distance - self.follow_distance)
            cmd.linear.y = self.lateral_speed          # move left
            cmd.angular.z = self.angle_gain * math.radians(angle_deg)
            return

        # Front cleared before max_front_length: short obstacle.
        self.get_logger().info(
            f'Short obstacle (lateral progress {progress:.2f} m): bypass.')
        self.set_state(self.S_BYPASS_FRONT_RIGHT)
        self.bypass_front_right(m, closest, cmd)

    def turn_to_right(self, m, closest, cmd):
        """Rotate left until the wall is in the right region."""
        if closest == self.RIGHT and not self.front_blocked:
            self.get_logger().info('New right wall found.')
            self.set_state(self.S_FOLLOW_RIGHT)
            self.follow_right(m, closest, cmd)
            return
        cmd.angular.z = self.max_angular_speed

    def bypass_front_right(self, m, closest, cmd):
        """Slow diagonal +vx/+vy until RIGHT is the closest region."""
        if self.front_blocked:
            if self.start_front():
                self.follow_front(m, closest, cmd)
            return
        if closest == self.RIGHT:
            self.set_state(self.S_BYPASS_RIGHT)
            self.bypass_right(m, closest, cmd)
            return
        cmd.linear.x = self.bypass_vx
        cmd.linear.y = self.bypass_vy
        # angular.z stays 0.

    def bypass_right(self, m, closest, cmd):
        """Slow +vx with proportional vy until BACK_RIGHT is closest."""
        if self.front_blocked:
            if self.start_front():
                self.follow_front(m, closest, cmd)
            return
        if closest == self.BACK_RIGHT:
            self.set_state(self.S_BYPASS_BACK_RIGHT)
            self.bypass_back_right(m, closest, cmd)
            return
        distance, _ = m[self.RIGHT]
        cmd.linear.x = self.bypass_vx
        if distance < self.detection_limit:
            cmd.linear.y = -self.distance_gain * (
                distance - self.follow_distance)
        # angular.z stays 0.

    def bypass_back_right(self, m, closest, cmd):
        """Slow diagonal +vx/-vy until the right wall is recovered."""
        if self.front_blocked:
            if self.start_front():
                self.follow_front(m, closest, cmd)
            return
        if closest == self.RIGHT:
            self.get_logger().info('Right wall recovered.')
            self.set_state(self.S_FOLLOW_RIGHT)
            self.follow_right(m, closest, cmd)
            return
        vy = self.bypass_vy
        distance, _ = m[self.BACK_RIGHT]
        limit = self.back_right_limit_factor * self.follow_distance
        if distance < self.detection_limit and distance > limit:
            vy += self.distance_gain * (distance - limit)
        cmd.linear.x = self.bypass_vx
        cmd.linear.y = -clamp(vy, self.bypass_vy, self.lateral_speed)
        # angular.z stays 0.

    def scan_callback(self, scan):
        """Read the LiDAR and calculate the next velocity command."""
        self.scan_stamp = self.get_clock().now()
        m = self.region_measurements(scan)
        closest = self.confirmed_region(m)
        self.confirmed_front_blocked(m)
        cmd = Twist()

        if closest is None:
            self.get_logger().warn(
                'No valid LiDAR region: stopping.', throttle_duration_sec=2.0)
        elif self.state == self.S_FOLLOW_RIGHT:
            self.follow_right(m, closest, cmd)
        elif self.state == self.S_FOLLOW_FRONT:
            self.follow_front(m, closest, cmd)
        elif self.state == self.S_TURN_TO_RIGHT:
            self.turn_to_right(m, closest, cmd)
        elif self.state == self.S_BYPASS_FRONT_RIGHT:
            self.bypass_front_right(m, closest, cmd)
        elif self.state == self.S_BYPASS_RIGHT:
            self.bypass_right(m, closest, cmd)
        elif self.state == self.S_BYPASS_BACK_RIGHT:
            self.bypass_back_right(m, closest, cmd)

        # All velocity outputs are bounded.
        cmd.linear.x = clamp(
            cmd.linear.x, -self.forward_speed, self.forward_speed)
        cmd.linear.y = clamp(
            cmd.linear.y, -self.lateral_speed, self.lateral_speed)
        cmd.angular.z = clamp(
            cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.command = cmd

    def stop(self):
        """Publish a final zero command (several times to be sure)."""
        self.command = Twist()
        self.timer.cancel()
        for _ in range(5):
            try:
                self.publisher.publish(Twist())
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = HolonomicWallFollowerImproved()
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