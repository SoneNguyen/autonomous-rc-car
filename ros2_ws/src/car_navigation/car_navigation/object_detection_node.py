#!/usr/bin/env python3
"""Run the frozen ONNX image segmenter and publish Detection2DArray results."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import yaml

IMAGE_TOPIC = "/camera/image_raw"
DETECTIONS_TOPIC = "/detections"
CLASS_NAMES = {1: "cone", 2: "pedestrian", 3: "vehicle"}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _image_array(message: Image) -> np.ndarray:
    if message.encoding.lower() not in {"rgb8", "8uc3"}:
        raise ValueError(f"expected rgb8 input, got {message.encoding!r}")
    if message.width < 1 or message.height < 1 or message.step < message.width * 3:
        raise ValueError("RGB image layout is invalid")
    raw = np.frombuffer(message.data, dtype=np.uint8, count=message.step * message.height)
    return raw.reshape(message.height, message.step)[:, :message.width * 3].reshape(message.height, message.width, 3).copy()


def _boxes(probabilities: np.ndarray, width: int, height: int, threshold: float, minimum_pixels: int):
    labels = probabilities.argmax(axis=0)
    detections = []
    scale_x = width / probabilities.shape[2]
    scale_y = height / probabilities.shape[1]
    for class_index, class_name in CLASS_NAMES.items():
        binary = ((labels == class_index) & (probabilities[class_index] >= threshold)).astype(np.uint8)
        count, components, stats, _centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
        for component_id in range(1, count):
            x, y, comp_width, comp_height, area = map(int, stats[component_id])
            if area < minimum_pixels:
                continue
            mask = components == component_id
            score = float(probabilities[class_index][mask].mean())
            x0, y0 = max(0, int(math.floor(x * scale_x))), max(0, int(math.floor(y * scale_y)))
            x1 = min(width - 1, int(math.ceil((x + comp_width) * scale_x)) - 1)
            y1 = min(height - 1, int(math.ceil((y + comp_height) * scale_y)) - 1)
            detections.append((class_name, score, x0, y0, x1, y1))
    return detections


class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detection_node", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter("profile_path", "")
        self.declare_parameter("model_path", "")
        profile_path = Path(str(self.get_parameter("profile_path").value))
        self.model_path = Path(str(self.get_parameter("model_path").value))
        if not profile_path.is_file() or not self.model_path.is_file():
            raise FileNotFoundError("fusion requires the RC profile and a frozen ONNX model")
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        self.camera = profile["presets"]["rc_car"]["sensors"]["camera"]
        self.width = int(self.camera["width_px"]["value"])
        self.height = int(self.camera["height_px"]["value"])
        metadata_path = self.model_path.parent / "metadata.json"
        if not metadata_path.is_file():
            raise FileNotFoundError(f"frozen model metadata is missing: {metadata_path}")
        self.metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        expected_sha = self.metadata.get(
            "model_sha256",
            self.metadata.get("frozen_model_sha256", self.metadata.get("weights_sha256")),
        )
        if expected_sha and _sha256(self.model_path) != expected_sha:
            raise ValueError("ONNX model hash does not match its frozen metadata")
        threshold = self.metadata.get("decision_threshold", self.metadata.get("confidence_threshold"))
        if not isinstance(threshold, (int, float)) or not 0.0 < float(threshold) <= 1.0:
            raise ValueError("frozen model metadata needs a validation-selected confidence threshold")
        self.threshold = float(threshold)
        self.minimum_pixels = int(self.metadata.get("minimum_component_pixels", 3))
        model_size = self.metadata.get("input_size_px", [160, 120])
        self.input_width, self.input_height = int(model_size[0]), int(model_size[1])
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(self.model_path), sess_options=options, providers=["CPUExecutionProvider"])
        if self.session.get_providers()[0] != "CPUExecutionProvider":
            raise RuntimeError("ONNX Runtime CPU provider is required")
        self.publisher = self.create_publisher(Detection2DArray, DETECTIONS_TOPIC, qos_profile_sensor_data)
        self.latency_publisher = self.create_publisher(Float32, "/perception/latency_ms", 10)
        self.status_publisher = self.create_publisher(String, "/perception/status", 10)
        self.subscription = self.create_subscription(Image, IMAGE_TOPIC, self._on_image, qos_profile_sensor_data)
        self.status_publisher.publish(String(data="ready"))
        self.get_logger().info(f"loaded frozen ONNX model sha256={_sha256(self.model_path)} threshold={self.threshold:.6f}")

    def _on_image(self, message: Image) -> None:
        started = time.perf_counter()
        try:
            image = _image_array(message)
            resized = cv2.resize(image, (self.input_width, self.input_height), interpolation=cv2.INTER_AREA)
            tensor = (resized.astype(np.float32) / 255.0).transpose(2, 0, 1)[None, ...]
            logits = self.session.run(["class_logits"], {"image": tensor})[0][0]
            logits = logits - logits.max(axis=0, keepdims=True)
            exponent = np.exp(logits)
            probabilities = exponent / exponent.sum(axis=0, keepdims=True)
            detections = _boxes(probabilities, message.width, message.height, self.threshold, self.minimum_pixels)
            output = Detection2DArray()
            output.header = message.header
            for class_name, score, x0, y0, x1, y1 in detections:
                detection = Detection2D()
                detection.header = message.header
                detection.bbox.center.position.x = (x0 + x1) / 2.0
                detection.bbox.center.position.y = (y0 + y1) / 2.0
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = max(1.0, float(x1 - x0 + 1))
                detection.bbox.size_y = max(1.0, float(y1 - y0 + 1))
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)
                output.detections.append(detection)
            self.publisher.publish(output)
            latency_ms = max(0.0, (time.perf_counter() - started) * 1000.0)
            self.latency_publisher.publish(Float32(data=latency_ms))
            self.status_publisher.publish(String(data="ready"))
        except (ValueError, RuntimeError, cv2.error) as exc:
            self.status_publisher.publish(String(data="inference_error"))
            self.get_logger().error(f"image inference failed: {exc}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0

if __name__ == "__main__": raise SystemExit(main())
