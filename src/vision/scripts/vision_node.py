#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
# -*- coding: utf-8 -*-

import math
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision.msg import CamDetection, CamDetections

from cv_bridge import CvBridge
import cv2

import tf2_ros
from geometry_msgs.msg import PointStamped

# Optional: YOLO from ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class VisionNode(Node):
    """
    VisionNode

    - Subscribes:
        /camera/camera/color/image_raw                  (RGB)
        /camera/camera/aligned_depth_to_color/image_raw (Depth, aligned)
        /camera/camera/color/camera_info                (Camera intrinsics)

    - Publishes:
        /cam_detections (CamDetections)

    - Also opens an OpenCV window showing:
        - YOLO person detections
        - Distance (m) of each detected person
    """

    def __init__(self):
        super().__init__('vision_node')

        # ---------------- Parameters ----------------
        # Topics tuned for your current RealSense setup
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        # ✅ 修复：使用color光学坐标系，因为深度图是对齐到color的
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('global_frame', 'base_footprint')  # or 'base_link', 'map', etc.

        self.declare_parameter('yolo_model_path', 'yolo11n.pt')
        self.declare_parameter('yolo_conf_thresh', 0.3)
        # debug_mode: True = always publish fake detection (for testing)
        self.declare_parameter('debug_mode', False)
        # Decimate RGB callbacks — Gazebo RGBD ticks at 30 Hz but YOLO11n
        # on CPU takes 70-150 ms per frame. Running inference on every
        # frame saturated the rclpy executor and starved /cmd_vel_raw,
        # freezing the robot. stride=3 → ~10 Hz effective inference,
        # matches pedestrian_tracker's publish rate. Set stride=1 to
        # process every frame (old behaviour); real hardware with a
        # slower camera may not need decimation.
        self.declare_parameter('yolo_stride', 3)
        # YOLO inference device: 'cuda' (GPU), 'cpu', or specific GPU
        # index string like '0'. Default is 'cuda' — _resolve_yolo_device()
        # silently falls back to 'cpu' if torch.cuda is unavailable.
        self.declare_parameter('yolo_device', 'cuda')

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        cam_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.conf_thresh = float(self.get_parameter('yolo_conf_thresh').value)
        self.debug_mode = bool(self.get_parameter('debug_mode').value)
        self.yolo_stride = max(1, int(self.get_parameter('yolo_stride').value))
        self.yolo_device = self._resolve_yolo_device(
            str(self.get_parameter('yolo_device').value)
        )
        self._rgb_seen = 0
        # Skip cv2.imshow when we're clearly headless. On WSL2 or a
        # Docker container with no forwarded DISPLAY, cv2.waitKey can
        # block for several ms per frame or hang entirely — contributes
        # to the same executor starvation as YOLO.
        self._show_gui = bool(os.environ.get('DISPLAY', '').strip())

        # ---------------- YOLO model ----------------
        self.model = None
        if not self.debug_mode and YOLO_AVAILABLE:
            model_path = self.get_parameter('yolo_model_path').value
            self.get_logger().info(f'[VisionNode] Loading YOLO model: {model_path}')
            try:
                self.model = YOLO(model_path)
            except Exception as e:
                self.get_logger().error(f'[VisionNode] Failed to load YOLO model: {e}')
                self.model = None
        elif not self.debug_mode and not YOLO_AVAILABLE:
            self.get_logger().warn('[VisionNode] ultralytics not installed, no detections will be produced.')

        # ---------------- ROS utils ----------------
        self.bridge = CvBridge()
        self.cam_info = None
        self.last_depth_msg = None

        # Match RealSense "sensor data" QoS (BEST_EFFORT)
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_rgb = self.create_subscription(
            Image, rgb_topic, self.rgb_cb, qos_sensor)

        self.sub_depth = self.create_subscription(
            Image, depth_topic, self.depth_cb, qos_sensor)

        self.sub_info = self.create_subscription(
            CameraInfo, cam_info_topic, self.info_cb, qos_sensor)

        self.pub_dets = self.create_publisher(
            CamDetections, '/cam_detections', 10)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Heartbeat timer: prints once per second
        self.timer = self.create_timer(1.0, self.heartbeat_cb)

        mode_str = "DEBUG (fake detections)" if self.debug_mode else "FULL (YOLO + depth + TF + GUI)"
        self.get_logger().info(f'[VisionNode] Initialized in {mode_str} mode.')
        self.get_logger().info(f'[VisionNode] Subscribing to RGB: {rgb_topic}')
        self.get_logger().info(f'[VisionNode] Using YOLO device: {self.yolo_device}')

    # ==========================================================
    # Heartbeat
    # ==========================================================
    def heartbeat_cb(self):
        self.get_logger().info(
            f'heartbeat: cam_info={"yes" if self.cam_info else "no"}, '
            f'depth={"yes" if self.last_depth_msg else "no"}'
        )

    # ==========================================================
    # Callbacks
    # ==========================================================
    def info_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def depth_cb(self, msg: Image):
        self.last_depth_msg = msg

    def rgb_cb(self, rgb_msg: Image):
        # Decimate: only run the (expensive) YOLO + TF pipeline on every
        # `yolo_stride`-th frame. Keeps the subscription callback thread
        # responsive and /cmd_vel_raw publishing at 10 Hz.
        self._rgb_seen += 1
        if (self._rgb_seen - 1) % self.yolo_stride != 0:
            return

        # debug: confirm frames are arriving (throttled)
        # self.get_logger().info("rgb_cb: got RGB frame", throttle_duration_sec=5.0)

        # ------------------------------------------------------
        # DEBUG MODE: publish one fake detection every frame
        # ------------------------------------------------------
        if self.debug_mode:
            dets_msg = CamDetections()
            dets_msg.stamp = rgb_msg.header.stamp

            det = CamDetection()
            det.x = 0.0
            det.y = 0.0
            det.z = 0.0
            det.confidence = 1.0
            det.embedding = []

            dets_msg.detections.append(det)
            self.get_logger().info("rgb_cb: DEBUG -> publishing 1 fake detection")
            self.pub_dets.publish(dets_msg)

            # Optional: still show the raw RGB frame in debug mode
            try:
                if self._show_gui:
                    rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                    cv2.imshow("VisionNode - DEBUG", rgb_img)
                    cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'[VisionNode] cv_bridge RGB error (debug): {e}')

            return

        # ------------------------------------------------------
        # FULL MODE: YOLO + depth + TF + OpenCV display
        # ------------------------------------------------------
        if self.cam_info is None or self.last_depth_msg is None:
            self.get_logger().info("rgb_cb: waiting for cam_info or depth...")
            return

        if self.model is None:
            self.get_logger().warn("rgb_cb: YOLO model not loaded.")
            return  # YOLO not available

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[VisionNode] cv_bridge RGB error: {e}')
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(
                self.last_depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'[VisionNode] cv_bridge Depth error: {e}')
            return

        detections = self.run_yolo(rgb_img)
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        dets_msg = CamDetections()
        dets_msg.stamp = rgb_msg.header.stamp

        height, width = depth_img.shape[:2]

        # Draw something even if there are no detections
        if not detections:
            if self._show_gui:
                cv2.imshow("VisionNode", rgb_img)
                cv2.waitKey(1)
            self.get_logger().info("Processed frame, found 0 persons")
            self.pub_dets.publish(dets_msg)
            return

        for (xmin, ymin, xmax, ymax, conf) in detections:
            u = int((xmin + xmax) / 2.0)
            v = int((ymin + ymax) / 2.0)

            if u < 0 or v < 0 or u >= width or v >= height:
                continue

            Z = float(depth_img[v, u])

            # RealSense aligned depth is usually 16UC1 (mm)
            if self.last_depth_msg.encoding in ['16UC1', 'mono16']:
                Z = Z / 1000.0  # mm -> m

            if Z <= 0.0 or math.isnan(Z) or math.isinf(Z):
                continue

            # Back-project to camera coordinates
            Xc = (u - cx) * Z / fx
            Yc = (v - cy) * Z / fy
            Zc = Z

            # Euclidean distance from camera
            dist = math.sqrt(Xc * Xc + Yc * Yc + Zc * Zc)

            # ------------------------------------------------------------------
            # 1) Draw on the image (OpenCV GUI)
            # ------------------------------------------------------------------
            pt1 = (int(xmin), int(ymin))
            pt2 = (int(xmax), int(ymax))
            cv2.rectangle(rgb_img, pt1, pt2, (0, 255, 0), 2)

            # Mark the depth sample point (center of bbox)
            cv2.circle(rgb_img, (u, v), 4, (0, 0, 255), -1)

            label = f"person {conf:.2f}, {dist:.2f} m"
            label_pos = (pt1[0], max(pt1[1] - 10, 0))
            cv2.putText(rgb_img, label, label_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # ------------------------------------------------------------------
            # 2) Transform to global frame and publish CamDetection
            # ------------------------------------------------------------------
            pt_global = self.transform_point_camera_to_global(Xc, Yc, Zc)
            if pt_global is None:
                continue

            x_map, y_map, z_map = pt_global

            det = CamDetection()
            det.x = float(x_map)
            det.y = float(y_map)
            det.z = float(z_map)
            det.confidence = float(conf)
            det.embedding = []
            det.xmin = int(max(0, min(65535, int(xmin))))
            det.ymin = int(max(0, min(65535, int(ymin))))
            det.xmax = int(max(0, min(65535, int(xmax))))
            det.ymax = int(max(0, min(65535, int(ymax))))

            dets_msg.detections.append(det)

        # Show the annotated image (skipped on headless — see _show_gui)
        if self._show_gui:
            cv2.imshow("VisionNode", rgb_img)
            cv2.waitKey(1)

        self.get_logger().info(
            f"Processed frame, found {len(dets_msg.detections)} persons"
        )
        self.pub_dets.publish(dets_msg)

    # ==========================================================
    # YOLO helper & TF helpers
    # ==========================================================
    def _resolve_yolo_device(self, requested_device: str) -> str:
        device = str(requested_device).strip() or 'cpu'
        normalized = device.lower()
        wants_cuda = (
            normalized in ('cuda', 'gpu')
            or normalized.startswith('cuda:')
            or normalized.replace(',', '').isdigit()
        )
        if wants_cuda and (not TORCH_AVAILABLE or not torch.cuda.is_available()):
            self.get_logger().warn(
                f"[VisionNode] CUDA requested ({device}) but unavailable; falling back to CPU."
            )
            return 'cpu'
        return device

    def run_yolo(self, bgr_img):
        # Explicit device= keeps ultralytics on GPU when torch.cuda is
        # available — otherwise it silently falls back to CPU and runs
        # ~10× slower on the same weights.
        try:
            results = self.model.predict(
                source=bgr_img,
                imgsz=640,
                conf=self.conf_thresh,
                device=self.yolo_device,
                verbose=False
            )
        except ValueError as e:
            if self.yolo_device != 'cpu' and 'Invalid CUDA' in str(e):
                self.get_logger().warn(
                    '[VisionNode] Ultralytics rejected the configured CUDA device; retrying on CPU.'
                )
                self.yolo_device = 'cpu'
                results = self.model.predict(
                    source=bgr_img,
                    imgsz=640,
                    conf=self.conf_thresh,
                    device=self.yolo_device,
                    verbose=False
                )
            else:
                raise

        if not results:
            return []

        res = results[0]
        if res.boxes is None:
            return []

        detections = []
        for box in res.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            # Only keep "person" class (0)
            if cls_id != 0:
                continue
            xyxy = box.xyxy[0].cpu().numpy()
            xmin, ymin, xmax, ymax = xyxy.tolist()
            detections.append((xmin, ymin, xmax, ymax, conf))

        return detections

    def transform_point_camera_to_global(self, Xc, Yc, Zc):
        pt_cam = PointStamped()
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.point.x = float(Xc)
        pt_cam.point.y = float(Yc)
        pt_cam.point.z = float(Zc)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.camera_frame,
                rclpy.time.Time()  # latest available
            )
        except Exception as e:
            self.get_logger().warn(f'[VisionNode] TF lookup failed: {e}')
            return None

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        pc = np.array([Xc, Yc, Zc])
        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        pg = R @ pc + np.array([tx, ty, tz])

        return pg[0], pg[1], pg[2]

    @staticmethod
    def quaternion_to_rotation_matrix(qx, qy, qz, qw):
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        R = np.array([
            [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
            [2.0 * (xy + wz),           1.0 - 2.0 * (xx + zz),   2.0 * (yz - wx)],
            [2.0 * (xz - wy),           2.0 * (yz + wx),         1.0 - 2.0 * (xx + yy)]
        ])
        return R


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
