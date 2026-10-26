import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


class NavigationNode(Node):
    """A small proportional controller for the planar delivery demo."""

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('normal_speed', 0.90)
        self.declare_parameter('approach_speed', 0.70)
        self.declare_parameter('push_speed', 0.28)
        self.declare_parameter('reverse_speed', 0.45)
        self.declare_parameter('recovery_reverse_speed', 0.24)
        self.declare_parameter('recovery_turn_speed', 0.75)
        self.declare_parameter('recovery_reverse_duration', 1.3)
        self.declare_parameter('recovery_turn_duration', 1.2)
        self.normal_speed = self.get_parameter('normal_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.push_speed = self.get_parameter('push_speed').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.recovery_reverse_speed = self.get_parameter(
            'recovery_reverse_speed').value
        self.recovery_turn_speed = self.get_parameter(
            'recovery_turn_speed').value
        self.recovery_reverse_duration = self.get_parameter(
            'recovery_reverse_duration').value
        self.recovery_turn_duration = self.get_parameter(
            'recovery_turn_duration').value
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(String, '/delivery/target_pose', self.on_target, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.bumper_sub = self.create_subscription(
            Contacts, '/bumper/base', self.on_bumper, 10)
        self.left_wheel_bumper_sub = self.create_subscription(
            Contacts, '/bumper/left_wheel',
            lambda msg: self.on_bumper(msg, 'left'), 10)
        self.right_wheel_bumper_sub = self.create_subscription(
            Contacts, '/bumper/right_wheel',
            lambda msg: self.on_bumper(msg, 'right'), 10)
        self.arrival_pub = self.create_publisher(String, '/delivery/arrival', 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target = None
        self.arrived = False
        self.recovery_phase = None
        self.recovery_until_ns = 0
        self.recovery_cooldown_until_ns = 0
        self.recovery_turn_sign = 1.0
        self.timer = self.create_timer(0.05, self.control)

    def on_target(self, msg: String):
        try:
            target = json.loads(msg.data)
            previous_label = self.target.get('label') if self.target else None
            self.target = target
            self.arrived = False
            if target.get('label') != previous_label:
                self.recovery_phase = None
        except json.JSONDecodeError:
            self.get_logger().warning('invalid target pose')

    @staticmethod
    def package_contact(msg: Contacts):
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('_package' in name for name in names):
                return True
        return False

    def on_bumper(self, msg: Contacts, contact_side=None):
        if not self.target or self.recovery_phase is not None:
            return
        if self.target.get('mode') not in ('approach', 'reposition'):
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.recovery_cooldown_until_ns:
            return
        if not self.package_contact(msg):
            return

        if contact_side == 'left':
            self.recovery_turn_sign = -1.0
        elif contact_side == 'right':
            self.recovery_turn_sign = 1.0
        else:
            desired = math.atan2(
                float(self.target['y']) - self.y,
                float(self.target['x']) - self.x,
            )
            error = math.atan2(
                math.sin(desired - self.yaw), math.cos(desired - self.yaw)
            )
            if abs(error) > 0.15:
                self.recovery_turn_sign = 1.0 if error > 0.0 else -1.0
            else:
                # Alternate sides when the package is exactly ahead, so a
                # second contact cannot repeat the same failed manoeuvre.
                self.recovery_turn_sign *= -1.0
        self.recovery_phase = 'reverse'
        self.recovery_until_ns = now_ns + int(
            self.recovery_reverse_duration * 1e9)
        self.get_logger().warning(
            'package contact while repositioning'
            f' ({contact_side or "base"}); backing up and turning'
        )

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control(self):
        command = Twist()
        if not self.target or self.arrived:
            self.cmd_pub.publish(command)
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.recovery_phase == 'reverse':
            if now_ns < self.recovery_until_ns:
                command.linear.x = -self.recovery_reverse_speed
                self.cmd_pub.publish(command)
                return
            self.recovery_phase = 'turn'
            self.recovery_until_ns = now_ns + int(
                self.recovery_turn_duration * 1e9)
        if self.recovery_phase == 'turn':
            if now_ns < self.recovery_until_ns:
                command.angular.z = (
                    self.recovery_turn_sign * self.recovery_turn_speed
                )
                self.cmd_pub.publish(command)
                return
            self.recovery_phase = None
            self.recovery_cooldown_until_ns = now_ns + int(1.0e9)
            self.get_logger().info('package avoidance recovery completed')
        dx = float(self.target['x']) - self.x
        dy = float(self.target['y']) - self.y
        distance = math.hypot(dx, dy)
        mode = self.target.get('mode', 'navigate')
        arrival_distance = (
            0.08 if mode == 'push'
            else 0.10 if mode in ('approach', 'reposition')
            else 0.20
        )
        if distance < arrival_distance:
            self.arrived = True
            self.arrival_pub.publish(String(data=json.dumps(self.target)))
            self.cmd_pub.publish(command)
            self.get_logger().info(f"arrived at {self.target.get('label', 'target')}")
            return
        desired = math.atan2(dy, dx)
        error = math.atan2(math.sin(desired - self.yaw), math.cos(desired - self.yaw))
        drive_direction = 1.0
        if mode == 'reposition' and abs(error) > math.pi / 2.0:
            # Point the rear of the robot at the staging waypoint. This lets
            # the robot retreat from a package without turning into it.
            reverse_heading = desired + math.pi
            error = math.atan2(
                math.sin(reverse_heading - self.yaw),
                math.cos(reverse_heading - self.yaw),
            )
            drive_direction = -1.0
        angular_limit = 0.45 if mode == 'push' else 0.9
        command.angular.z = max(
            -angular_limit, min(angular_limit, 1.8 * error)
        )
        heading_tolerance = 0.20 if mode == 'push' else 0.35
        if abs(error) < heading_tolerance:
            speed_limit = (
                self.push_speed if mode == 'push'
                else self.reverse_speed if drive_direction < 0.0
                else self.approach_speed if mode in ('approach', 'reposition')
                else self.normal_speed
            )
            distance_gain = 0.45 if mode == 'push' else 0.70
            command.linear.x = drive_direction * min(
                speed_limit, distance_gain * distance
            )
        self.cmd_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
