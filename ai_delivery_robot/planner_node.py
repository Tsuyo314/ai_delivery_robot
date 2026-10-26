import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlannerNode(Node):
    """Choose the next delivery target using an explainable AI score."""

    def __init__(self):
        super().__init__('planner_node')
        self.sub = self.create_subscription(String, '/delivery/candidates', self.on_candidates, 10)
        self.pub = self.create_publisher(String, '/delivery/decision', 10)

    def on_candidates(self, msg: String):
        try:
            candidates = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('invalid candidate message')
            return
        if not candidates:
            return
        for candidate in candidates:
            candidate['score'] = (
                2.0 * float(candidate.get('priority', 1.0))
                + 1.5 * float(candidate.get('confidence', 1.0))
                - 0.8 * float(candidate.get('distance', 0.0))
            )
        selected = max(candidates, key=lambda item: item['score'])
        self.pub.publish(String(data=json.dumps(selected)))
        self.get_logger().info(
            f"AI selected {selected['color']} score={selected['score']:.2f} "
            f"distance={selected.get('distance', 0.0):.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
