import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ai_delivery_robot.color_model import hsv_feature, predict_color


class PerceptionNode(Node):
    """Detect colored blobs and publish a compact JSON-like line."""

    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.detection_pub = self.create_publisher(String, '/delivery/perception', 10)
        self.last_log = self.get_clock().now()
        default_model = os.path.join(
            get_package_share_directory('ai_delivery_robot'),
            'models', 'color_classifier.yml',
        )
        self.declare_parameter('color_model_path', default_model)
        model_path = self.get_parameter('color_model_path').value
        self.model = cv2.ml.RTrees_load(model_path)
        if self.model is None or not self.model.isTrained():
            raise RuntimeError(f'failed to load trained color model: {model_path}')
        self.get_logger().info(f'loaded trained color model: {model_path}')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'camera conversion failed: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width = frame.shape[:2]
        # This mask only locates saturated objects. Their color is determined
        # by the trained model below, not by hand-written per-color thresholds.
        mask = cv2.inRange(hsv, np.array((0, 70, 50)), np.array((179, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_by_color = {}

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 80.0:
                continue
            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                continue

            region = np.zeros((height, width), dtype=np.uint8)
            cv2.drawContours(region, [contour], -1, 255, thickness=cv2.FILLED)
            pixels = hsv[region != 0]
            if pixels.size == 0:
                continue
            hue_angles = pixels[:, 0].astype(np.float32) * np.pi / 90.0
            mean_hue = (
                np.arctan2(np.sin(hue_angles).mean(), np.cos(hue_angles).mean())
                % (2.0 * np.pi)
            ) * 90.0 / np.pi
            mean_saturation = float(pixels[:, 1].mean())
            mean_value = float(pixels[:, 2].mean())
            color, confidence = predict_color(
                self.model,
                hsv_feature(mean_hue, mean_saturation, mean_value),
            )

            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
            angle = (cx - width / 2.0) / max(1.0, width / 2.0)
            distance = 1.0 + 2.0 * (1.0 - cy / max(1.0, height))
            detection = (confidence, angle, distance)
            if color not in best_by_color or confidence > best_by_color[color][0]:
                best_by_color[color] = detection

        detections = [
            f'{color}:{angle:.3f}:{distance:.3f}:{confidence:.3f}'
            for color, (confidence, angle, distance) in sorted(best_by_color.items())
        ]

        out = String()
        out.data = ';'.join(detections)
        self.detection_pub.publish(out)
        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds > 1_000_000_000:
            self.get_logger().info(f'model inference={out.data or "none"}')
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
