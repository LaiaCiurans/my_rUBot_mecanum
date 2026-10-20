import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class RobotWallFollower(Node):
    def __init__(self):
        super().__init__('robot_wall_follower_holonomic_node')

        # Parameters
        self.declare_parameter('follow_distance', 0.35)
        self.declare_parameter('forward_speed', 0.30)
        self.declare_parameter('turn_speed', 0.50)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('right_detection_limit', 1.0)
        self.declare_parameter('lateral_gain', 0.5)
        self.declare_parameter('alignment_gain', 0.5)
        self.declare_parameter('max_lateral_speed', 0.40)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('back_right_limit_factor', 1.50)

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.right_detection_limit = float(
            self.get_parameter('right_detection_limit').value
        )
        self.k_lateral = float(self.get_parameter('lateral_gain').value)
        self.k_alignment = float(self.get_parameter('alignment_gain').value)
        self.max_lateral_speed = float(
            self.get_parameter('max_lateral_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.back_right_limit_factor = float(
            self.get_parameter('back_right_limit_factor').value
        )

        self.cmd = Twist()
        self._state_action = 'Idle'
        self._last_action_logged = None
        self._shutting_down = False

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.info_timer = self.create_timer(1.0, self.log_info)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self.get_logger().info(
            'rUBot holonomic wall follower with lateral and alignment correction.'
        )

    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()

        for timer in [self.info_timer, self.cmd_timer]:
            try:
                timer.cancel()
            except Exception:
                pass

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    def cmd_publish_timer_cb(self):
        if not self._shutting_down:
            self.publisher.publish(self.cmd)

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        front = []
        front_right = []
        right = []
        back_right = []

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = angle_min + i * angle_inc

            if -20 <= angle <= 20:
                front.append(distance)
            elif -70 <= angle < -20:
                front_right.append(distance)
            elif -110 <= angle < -70:
                right.append(distance)
            elif -160 <= angle < -110:
                back_right.append(distance)

        # Mean of the three nearest rays: less sensitive to one noisy sample.
        def robust_min(values):
            nearest = sorted(values)[:3]
            return sum(nearest) / len(nearest) if nearest else float('inf')

        min_front = robust_min(front)
        min_front_right = robust_min(front_right)
        min_right = robust_min(right)
        min_back_right = robust_min(back_right)

        twist = Twist()
        action = ''

        # RULE 1: FRONT obstacle -> turn left
        if min_front < self.follow_distance:
            twist.angular.z = min(
                self.max_angular_speed, 2.0 * self.v_ang
            )
            action = f'FRONT {min_front:.2f} m -> turn LEFT'

        # RULE 2: FRONT-RIGHT obstacle -> turn left
        elif min_front_right < self.follow_distance:
            twist.angular.z = min(
                self.max_angular_speed, 2.0 * self.v_ang
            )
            action = f'FRONT-RIGHT {min_front_right:.2f} m -> turn LEFT'

        # RULE 3: follow the RIGHT wall with lateral and angular correction
        elif min_right < self.right_detection_limit:
            distance_error = min_right - self.follow_distance

            if abs(distance_error) <= self.tol:
                distance_error = 0.0

            # Positive y moves left: away from a close right wall.
            twist.linear.x = self.v_lin
            twist.linear.y = max(
                -self.max_lateral_speed,
                min(self.max_lateral_speed, -self.k_lateral * distance_error),
            )

            # If the front is closer than the back, turn left, and vice versa.
            if math.isfinite(min_front_right) and math.isfinite(min_back_right):
                alignment_error = min_front_right - min_back_right
                twist.angular.z = max(
                    -self.max_angular_speed,
                    min(
                        self.max_angular_speed,
                        -self.k_alignment * alignment_error,
                    ),
                )

            action = (
                f'RIGHT {min_right:.2f} m -> '
                f'vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f}, '
                f'w={twist.angular.z:.2f}'
            )

        # RULE 4: end of wall -> turn right around it
        elif (
            min_right >= self.right_detection_limit
            and math.isfinite(min_back_right)
            and min_back_right < min_right
        ):
            back_right_limit = (
                self.back_right_limit_factor * self.follow_distance
            )
            distance_excess = max(
                0.0, min_back_right - back_right_limit
            )
            twist.linear.x = self.v_lin * 0.1
            twist.linear.y = max(
                -self.max_lateral_speed,
                -self.k_lateral * distance_excess,
            )
            twist.angular.z = max(-self.max_angular_speed, -2.0 * self.v_ang)
            action = (
                f'BACK-RIGHT {min_back_right:.2f} m -> '
                f'vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f}, '
                f'strong RIGHT turn'
            )

        self.cmd = twist

        if action != self._last_action_logged:
            self.get_logger().info(action if action else 'No action (stopped).')
            self._last_action_logged = action

        self._state_action = action if action else 'Stopped (no wall detected)'

    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = RobotWallFollower()

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
