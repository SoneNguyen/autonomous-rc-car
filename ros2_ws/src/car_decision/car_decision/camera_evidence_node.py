"""HSV color classifier and low-rate preview publisher for the dashboard."""

from __future__ import annotations

import time

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import qos_profile_sensor_data

from .evidence_common import (
    classify_hsv_image,
    message_stamp_ns,
    value_of,
)
from .evidence_node_base import EvidenceNode, spin_node


class CameraEvidenceNode(EvidenceNode):
    def __init__(self) -> None:
        super().__init__("camera", "/camera/image_raw", "sensor_msgs/msg/Image")
        camera = self.sensor_profile["presets"]["rc_car"]["sensors"]["camera"]
        self._horizontal_fov = float(value_of(camera["horizontal_fov_rad"]))
        self._bridge = CvBridge()
        self._preview_publisher = self.create_publisher(
            CompressedImage, "/decision/preview/camera", 1
        )
        self._processing_decimation = 2  # Keep HSV work at 7.5 Hz from the 15 Hz source.
        self._source_frame_count = 0
        self._last_preview_wall_ns = 0
        self._preview_period_ns = 200_000_000  # Five dashboard frames per second at most.
        self._source_topic = "/camera/image_raw"
        self._message_type = "sensor_msgs/msg/Image"
        self._subscription = self.create_subscription(
            Image, self._source_topic, self._on_image, qos_profile_sensor_data
        )

    def _empty_payload(self):
        return {"classifier": "hsv", "objects": []}

    def _on_image(self, message: Image) -> None:
        self._source_frame_count += 1
        if self._source_frame_count % self._processing_decimation:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.is_faulted(now_ns):
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            objects = classify_hsv_image(image, self.decision, self._horizontal_fov)
            self._publish_preview(message, image)
        except Exception as exc:
            self.get_logger().warning(f"camera frame could not be normalized: {exc}")
            return
        self.record_sample(
            {"classifier": "hsv", "objects": objects},
            message_stamp_ns(message.header, now_ns),
            message.header.frame_id,
        )

    def _publish_preview(self, source: Image, image_bgr) -> None:
        """Publish a small JPEG for the read-only dashboard, never the run log."""
        if self._preview_publisher.get_subscription_count() == 0:
            return
        now_ns = time.monotonic_ns()
        if now_ns - self._last_preview_wall_ns < self._preview_period_ns:
            return
        height, width = image_bgr.shape[:2]
        target_width = min(320, int(width))
        target_height = max(1, round(height * target_width / max(1, width)))
        preview = cv2.resize(image_bgr, (target_width, target_height), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ok:
            return
        message = CompressedImage()
        message.header = source.header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self._preview_publisher.publish(message)
        self._last_preview_wall_ns = now_ns

def main(args=None) -> None:
    spin_node(CameraEvidenceNode, args)
