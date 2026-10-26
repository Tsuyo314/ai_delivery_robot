import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class DeliveryManagerNode(Node):
    """Manage the delivery state machine and bridge configured targets to the planner."""

    def __init__(self):
        super().__init__('delivery_manager_node')
        self.declare_parameter('targets', [
            'red,1.8,1.6,red_station,-1.8,1.6,1.0',
            'blue,2.0,-1.5,blue_station,-1.8,-1.5,0.8',
            'green,-1.4,0.0,green_station,1.7,0.0,1.2',
        ])
        self.declare_parameter('physical_delivery', False)
        self.declare_parameter('package_approach_offset', 0.52)
        self.declare_parameter('package_staging_offset', 0.85)
        self.declare_parameter('package_alignment_offset', 0.70)
        self.declare_parameter('push_contact_offset', 0.34)
        self.declare_parameter('delivery_tolerance', 0.18)
        self.declare_parameter('fine_adjust_radius', 0.55)
        self.declare_parameter('max_push_step', 1.20)
        self.declare_parameter('post_delivery_retreat', 0.55)
        self.declare_parameter('obstacle_centers', ['0.0,0.9'])
        self.declare_parameter('obstacle_half_sizes', ['0.5,0.09'])
        self.declare_parameter('route_clearance', 0.40)
        self.physical_delivery = self.get_parameter('physical_delivery').value
        self.approach_offset = self.get_parameter('package_approach_offset').value
        self.staging_offset = self.get_parameter('package_staging_offset').value
        self.alignment_offset = self.get_parameter(
            'package_alignment_offset').value
        self.push_contact_offset = self.get_parameter('push_contact_offset').value
        self.delivery_tolerance = self.get_parameter('delivery_tolerance').value
        self.fine_adjust_radius = self.get_parameter('fine_adjust_radius').value
        self.max_push_step = self.get_parameter('max_push_step').value
        self.post_delivery_retreat = self.get_parameter('post_delivery_retreat').value
        self.obstacle_centers = [
            tuple(float(value) for value in entry.split(','))
            for entry in self.get_parameter('obstacle_centers').value
        ]
        self.obstacle_half_sizes = [
            tuple(float(value) for value in entry.split(','))
            for entry in self.get_parameter('obstacle_half_sizes').value
        ]
        self.route_clearance = self.get_parameter('route_clearance').value
        self.targets = self.parse_targets(self.get_parameter('targets').value)
        self.remaining = {item['color']: item for item in self.targets}
        self.state = 'searching'
        self.current = None
        self.push_attempt = 0
        self.package_positions = {}
        self.staging_target = None
        self.alignment_target = None
        self.push_goal = None
        self.push_direction = None
        self.correction_axis = None
        self.retreat_target = None
        self.clearance_target = None
        self.robot_position = (0.0, 0.0)
        self.candidate_pub = self.create_publisher(String, '/delivery/candidates', 10)
        self.pose_pub = self.create_publisher(String, '/delivery/target_pose', 10)
        self.status_pub = self.create_publisher(String, '/delivery/status', 10)
        self.decision_sub = self.create_subscription(String, '/delivery/decision', self.on_decision, 10)
        self.arrival_sub = self.create_subscription(String, '/delivery/arrival', self.on_arrival, 10)
        self.perception_sub = self.create_subscription(String, '/delivery/perception', self.on_perception, 10)
        self.package_pose_sub = self.create_subscription(
            TFMessage, '/world/delivery_world/dynamic_pose/info',
            self.on_package_poses, 10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_robot_odom, 10
        )
        self.last_perception = self.get_clock().now()
        self.perception_confidence = {}
        self.timer = self.create_timer(1.0, self.tick)

    @staticmethod
    def parse_targets(values):
        parsed = []
        for value in values:
            color, px, py, station, sx, sy, priority = value.split(',')
            parsed.append({
                'color': color, 'x': float(px), 'y': float(py),
                'station': station, 'station_x': float(sx), 'station_y': float(sy),
                'priority': float(priority), 'confidence': 1.0,
            })
        return parsed

    def on_perception(self, msg: String):
        self.last_perception = self.get_clock().now()
        self.perception_confidence = {}
        for token in msg.data.split(';'):
            if not token:
                continue
            fields = token.split(':')
            if len(fields) != 4:
                continue
            color = fields[0]
            if color in self.remaining:
                confidence = max(0.0, min(1.0, float(fields[3])))
                self.perception_confidence[color] = max(
                    confidence, self.perception_confidence.get(color, 0.0)
                )

    def on_package_poses(self, msg: TFMessage):
        """Cache world positions of dynamic package models."""
        package_names = {
            f'{color}_package': color for color in self.remaining
        }
        for transform in msg.transforms:
            frame = transform.child_frame_id.strip('/').replace('::', '/')
            for model_name, color in package_names.items():
                if frame == model_name or frame.endswith('/' + model_name):
                    translation = transform.transform.translation
                    self.package_positions[color] = (
                        float(translation.x), float(translation.y)
                    )
                    break

    def on_robot_odom(self, msg: Odometry):
        self.robot_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def tick(self):
        # Keep the current state observable even when `ros2 topic echo` is
        # started after a state transition.
        self.publish_status()
        if self.current and self.state in (
                'going_to_clearance', 'going_to_staging',
                'going_to_alignment', 'going_to_package',
                'pushing_package', 'going_to_station',
                'retreating_from_station'):
            self.publish_current_target()
        if not self.remaining and self.state != 'retreating_from_station':
            self.state = 'completed'
            self.publish_status()
            return
        if self.state == 'searching':
            candidates = []
            for item in self.remaining.values():
                candidate = dict(item)
                candidate['distance'] = math.hypot(item['x'], item['y'])
                # Camera detections increase confidence; when the camera is not
                # currently looking at a package, keep it as a lower-confidence
                # fallback so the demo can still finish deterministically.
                candidate['confidence'] = self.perception_confidence.get(
                    item['color'], 0.2
                )
                candidates.append(candidate)
            self.candidate_pub.publish(String(data=json.dumps(candidates)))
            self.state = 'waiting_for_plan'
            self.publish_status()

    def on_decision(self, msg: String):
        if self.state != 'waiting_for_plan':
            return
        selected = json.loads(msg.data)
        color = selected['color']
        if color not in self.remaining:
            self.state = 'searching'
            return
        self.current = self.remaining[color]
        self.push_attempt = 0
        self.staging_target = None
        self.alignment_target = None
        self.clearance_target = None
        self.push_goal = None
        self.push_direction = None
        self.plan_next_push()
        self.state = (
            'going_to_staging' if self.physical_delivery
            else 'going_to_package'
        )
        self.publish_current_target()
        self.publish_status()

    def on_arrival(self, msg: String):
        if not self.current:
            return
        if self.state == 'going_to_clearance':
            self.state = 'going_to_staging'
            self.publish_current_target()
            self.publish_status()
        elif self.state == 'going_to_staging':
            self.state = 'going_to_alignment'
            self.publish_current_target()
            self.publish_status()
        elif self.state == 'going_to_alignment':
            self.state = 'going_to_package'
            self.publish_current_target()
            self.publish_status()
        elif self.state == 'going_to_package' and self.physical_delivery:
            self.state = 'pushing_package'
            self.publish_current_target()
            self.publish_status()
        elif self.state == 'going_to_package':
            self.state = 'going_to_station'
            self.publish_current_target()
            self.publish_status()
        elif self.state == 'pushing_package':
            distance = self.package_delivery_distance()
            if distance is not None and distance <= self.delivery_tolerance:
                self.complete_current_delivery()
            else:
                self.push_attempt += 1
                previous_direction = self.push_direction
                contact_aligned = self.robot_aligned_behind_package()
                self.plan_next_push()
                direction_aligned = (
                    previous_direction is not None
                    and self.push_direction is not None
                    and previous_direction[0] * self.push_direction[0]
                    + previous_direction[1] * self.push_direction[1] > 0.985
                )
                if (contact_aligned and direction_aligned
                        and self.correction_axis == 'segment'):
                    self.get_logger().info(
                        f'push segment {self.push_attempt} complete; package '
                        'remains aligned, continuing without another staging '
                        'manoeuvre'
                    )
                    self.state = 'pushing_package'
                else:
                    detail = (
                        'pose unavailable' if distance is None
                        else f'{distance:.2f} m from station'
                    )
                    self.get_logger().warning(
                        f"{self.current['color']} package needs realignment "
                        f'({detail}); recovery attempt {self.push_attempt}'
                    )
                    self.state = 'going_to_staging'
                    self.staging_target = None
                    self.alignment_target = None
                    self.clearance_target = None
                self.publish_current_target()
                self.publish_status()
        elif self.state == 'going_to_station':
            self.complete_current_delivery()
        elif self.state == 'retreating_from_station':
            self.finish_delivery_retreat()

    def complete_current_delivery(self):
        if not self.current:
            return
        color = self.current['color']
        self.remaining.pop(color, None)
        self.get_logger().info(f'delivered {color} package')
        if self.physical_delivery:
            package_position = self.package_positions.get(
                color,
                (self.current['station_x'], self.current['station_y']),
            )
            direction = self.push_direction or (1.0, 0.0)
            retreat_offset = (
                self.push_contact_offset + self.post_delivery_retreat
            )
            self.retreat_target = (
                package_position[0] - direction[0] * retreat_offset,
                package_position[1] - direction[1] * retreat_offset,
            )
            self.state = 'retreating_from_station'
            self.correction_axis = 'retreat'
            self.publish_current_target()
            self.publish_status()
            return
        self.finish_delivery_retreat()

    def finish_delivery_retreat(self):
        self.current = None
        self.staging_target = None
        self.alignment_target = None
        self.push_goal = None
        self.push_direction = None
        self.correction_axis = None
        self.retreat_target = None
        self.clearance_target = None
        self.state = 'searching'
        self.publish_status()

    def package_delivery_distance(self):
        if not self.current:
            return None
        position = self.package_positions.get(self.current['color'])
        if position is None:
            return None
        return math.hypot(
            position[0] - self.current['station_x'],
            position[1] - self.current['station_y'],
        )

    def robot_aligned_behind_package(self):
        """Check that the robot still has safe, centred contact for pushing."""
        if not self.current or self.push_direction is None:
            return False
        position = self.package_positions.get(self.current['color'])
        if position is None:
            return False
        dx = position[0] - self.robot_position[0]
        dy = position[1] - self.robot_position[1]
        ux, uy = self.push_direction
        forward = dx * ux + dy * uy
        lateral = abs(dx * uy - dy * ux)
        return 0.20 <= forward <= 0.65 and lateral <= 0.14

    def plan_next_push(self):
        """Choose a direct push or one-axis correction from the latest pose."""
        if not self.current:
            return
        station = (
            self.current['station_x'], self.current['station_y']
        )
        position = self.package_positions.get(
            self.current['color'],
            (self.current['x'], self.current['y']),
        )

        dx = station[0] - position[0]
        dy = station[1] - position[1]
        distance = math.hypot(dx, dy)
        if distance > self.fine_adjust_radius:
            # Long uninterrupted pushes let the package slip sideways while
            # the robot continues to the station. Advance in short verified
            # segments, then re-read the package pose before the next push.
            length = max(distance, 1e-6)
            step = min(self.max_push_step, distance)
            self.push_goal = (
                position[0] + dx / length * step,
                position[1] + dy / length * step,
            )
            self.correction_axis = 'segment'
        elif abs(dx) >= abs(dy):
            # Preserve Y and correct only X. The following retry can then
            # approach another face of the package to correct Y if needed.
            self.push_goal = (station[0], position[1])
            self.correction_axis = 'x'
        else:
            self.push_goal = (position[0], station[1])
            self.correction_axis = 'y'
        goal_dx = self.push_goal[0] - position[0]
        goal_dy = self.push_goal[1] - position[1]
        goal_length = max(math.hypot(goal_dx, goal_dy), 1e-6)
        self.push_direction = (
            goal_dx / goal_length, goal_dy / goal_length
        )
        self.get_logger().info(
            f"planned push axis={self.correction_axis} "
            f"goal=({self.push_goal[0]:.2f}, {self.push_goal[1]:.2f})"
        )

    @staticmethod
    def segment_intersects_box(start, end, center, half_size):
        """Return whether a 2-D segment crosses an axis-aligned box."""
        t_min, t_max = 0.0, 1.0
        for axis in range(2):
            delta = end[axis] - start[axis]
            lower = center[axis] - half_size[axis]
            upper = center[axis] + half_size[axis]
            if abs(delta) < 1e-9:
                if start[axis] < lower or start[axis] > upper:
                    return False
                continue
            first = (lower - start[axis]) / delta
            second = (upper - start[axis]) / delta
            if first > second:
                first, second = second, first
            t_min = max(t_min, first)
            t_max = min(t_max, second)
            if t_min > t_max:
                return False
        return True

    def obstacle_avoidance_waypoint(self, start, goal):
        """Choose a visible expanded-box corner when the direct path is blocked."""
        for center, size in zip(
                self.obstacle_centers, self.obstacle_half_sizes):
            expanded = (
                size[0] + self.route_clearance,
                size[1] + self.route_clearance,
            )
            if not self.segment_intersects_box(start, goal, center, expanded):
                continue

            padding = 0.08
            corners = [
                (center[0] + sx * (expanded[0] + padding),
                 center[1] + sy * (expanded[1] + padding))
                for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)
            ]
            visible = [
                point for point in corners
                if not self.segment_intersects_box(start, point, center, expanded)
                and not self.segment_intersects_box(point, goal, center, expanded)
            ]
            if visible:
                return min(
                    visible,
                    key=lambda point: (
                        math.hypot(point[0] - start[0], point[1] - start[1])
                        + math.hypot(goal[0] - point[0], goal[1] - point[1])
                    ),
                )
        return None

    def publish_current_target(self):
        if not self.current:
            return
        if self.physical_delivery and self.state == 'retreating_from_station':
            target = {
                'label': f"{self.current['color']}_station_retreat",
                'x': self.retreat_target[0],
                'y': self.retreat_target[1],
                'mode': 'reposition',
            }
        elif self.physical_delivery:
            package_x, package_y = self.package_positions.get(
                self.current['color'],
                (self.current['x'], self.current['y']),
            )
            if self.push_goal is None:
                self.push_goal = (
                    self.current['station_x'], self.current['station_y']
                )
            if self.push_direction is None:
                dx = self.push_goal[0] - package_x
                dy = self.push_goal[1] - package_y
                length = max(math.hypot(dx, dy), 1e-6)
                self.push_direction = (dx / length, dy / length)
            ux, uy = self.push_direction
            behind_x = package_x - ux * self.approach_offset
            behind_y = package_y - uy * self.approach_offset
            alignment_x = behind_x - ux * self.alignment_offset
            alignment_y = behind_y - uy * self.alignment_offset

            if self.state == 'going_to_staging':
                if self.staging_target is None:
                    # Evaluate both sides of the package and choose the one
                    # farther from known obstacles. Keep it fixed for this
                    # attempt so repeated target publication cannot oscillate.
                    candidates = [
                        (
                            alignment_x - uy * self.staging_offset,
                            alignment_y + ux * self.staging_offset,
                        ),
                        (
                            alignment_x + uy * self.staging_offset,
                            alignment_y - ux * self.staging_offset,
                        ),
                    ]
                    self.alignment_target = (alignment_x, alignment_y)
                    if self.obstacle_centers:
                        self.staging_target = max(
                            candidates,
                            key=lambda point: min(
                                math.hypot(point[0] - obstacle[0],
                                           point[1] - obstacle[1])
                                for obstacle in self.obstacle_centers
                            ),
                        )
                    else:
                        self.staging_target = candidates[0]
                    self.clearance_target = self.obstacle_avoidance_waypoint(
                        self.robot_position, self.staging_target
                    )
                    if self.clearance_target is not None:
                        self.state = 'going_to_clearance'
                        self.get_logger().info(
                            'route blocked; using clearance waypoint '
                            f'({self.clearance_target[0]:.2f}, '
                            f'{self.clearance_target[1]:.2f})'
                        )
                if self.state == 'going_to_clearance':
                    target = {
                        'label': f"{self.current['color']}_clearance",
                        'x': self.clearance_target[0],
                        'y': self.clearance_target[1],
                        'mode': 'reposition',
                    }
                else:
                    target = {
                        'label': f"{self.current['color']}_staging",
                        'x': self.staging_target[0],
                        'y': self.staging_target[1],
                        'mode': 'reposition',
                    }
            elif self.state == 'going_to_clearance':
                target = {
                    'label': f"{self.current['color']}_clearance",
                    'x': self.clearance_target[0],
                    'y': self.clearance_target[1],
                    'mode': 'reposition',
                }
            elif self.state == 'going_to_alignment':
                target = {
                    'label': f"{self.current['color']}_alignment",
                    'x': self.alignment_target[0],
                    'y': self.alignment_target[1],
                    'mode': 'reposition',
                }
            elif self.state == 'going_to_package':
                target = {
                    'label': f"{self.current['color']}_push_start",
                    'x': behind_x, 'y': behind_y,
                    'mode': 'approach',
                }
            else:
                target = {
                    'label': (
                        self.current['station']
                        if self.correction_axis == 'direct'
                        else f"{self.current['station']}_{self.correction_axis}_adjust"
                    ),
                    'x': self.push_goal[0] - ux * self.push_contact_offset,
                    'y': self.push_goal[1] - uy * self.push_contact_offset,
                    'mode': 'push',
                }
        elif self.state == 'going_to_package':
            target = {
                'label': f"{self.current['color']}_package",
                'x': self.current['x'], 'y': self.current['y'],
            }
        else:
            target = {
                'label': self.current['station'],
                'x': self.current['station_x'], 'y': self.current['station_y'],
            }
        self.pose_pub.publish(String(data=json.dumps(target)))

    def publish_status(self):
        delivery_distance = self.package_delivery_distance()
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state,
            'remaining': sorted(self.remaining.keys()),
            'current': self.current['color'] if self.current else None,
            'push_attempt': self.push_attempt,
            'correction_axis': self.correction_axis,
            'push_goal': (
                [round(self.push_goal[0], 3), round(self.push_goal[1], 3)]
                if self.push_goal is not None else None
            ),
            'package_distance_to_station': (
                round(delivery_distance, 3)
                if delivery_distance is not None else None
            ),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
