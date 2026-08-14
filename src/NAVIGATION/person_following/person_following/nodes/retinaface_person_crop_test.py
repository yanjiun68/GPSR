#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import onnxruntime as ort
except Exception:
    ort = None

from person_following.nodes.vision_node import RetinaFaceOnnx, _resolve_model_path
from person_following.utils.runtime_config import resolve_onnx_providers, resolve_yolo_device
from person_following.utils.yolo_runtime import resolve_yolo_import


class RetinaFacePersonCropTestNode(Node):
    def __init__(self):
        super().__init__('retinaface_person_crop_test')

        def _param_bool(v) -> bool:
            if isinstance(v, bool):
                return v
            if isinstance(v, (int, float)):
                return bool(v)
            if isinstance(v, str):
                return v.strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')
            return bool(v)

        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('debug_image_topic', '/person_following/retinaface_person_crop/debug_image')
        self.declare_parameter('yolo_model_path', 'yolo11n.pt')
        self.declare_parameter('yolo_conf_thresh', 0.6)
        self.declare_parameter('yolo_device', 'auto')
        self.declare_parameter('retinaface_enabled', True)
        self.declare_parameter('retinaface_onnx_path', 'models/RetinaFace_int.onnx')
        self.declare_parameter('retinaface_input_w', 640)
        self.declare_parameter('retinaface_input_h', 640)
        self.declare_parameter('retinaface_conf_thresh', 0.7)
        self.declare_parameter('retinaface_nms_iou', 0.4)
        self.declare_parameter('retinaface_out_scores', '')
        self.declare_parameter('retinaface_out_boxes', '')
        self.declare_parameter('retinaface_out_landmarks', '')
        self.declare_parameter('retinaface_providers', 'CUDAExecutionProvider,CPUExecutionProvider')
        self.declare_parameter('enable_gui', True)

        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.yolo_conf_thresh = float(self.get_parameter('yolo_conf_thresh').value)
        self.yolo_device_request = str(self.get_parameter('yolo_device').value)
        self.retinaface_enabled = _param_bool(self.get_parameter('retinaface_enabled').value)
        self.retinaface_onnx_path = _resolve_model_path(self.get_parameter('retinaface_onnx_path').value)
        self.retinaface_input_w = int(self.get_parameter('retinaface_input_w').value)
        self.retinaface_input_h = int(self.get_parameter('retinaface_input_h').value)
        self.retinaface_conf_thresh = float(self.get_parameter('retinaface_conf_thresh').value)
        self.retinaface_nms_iou = float(self.get_parameter('retinaface_nms_iou').value)
        self.retinaface_out_scores = str(self.get_parameter('retinaface_out_scores').value)
        self.retinaface_out_boxes = str(self.get_parameter('retinaface_out_boxes').value)
        self.retinaface_out_landmarks = str(self.get_parameter('retinaface_out_landmarks').value)
        self.retinaface_providers_request = self.get_parameter('retinaface_providers').value
        self.enable_gui = _param_bool(self.get_parameter('enable_gui').value)

        self.yolo_import = resolve_yolo_import()
        if not self.yolo_import.available:
            raise RuntimeError(f'[RetinaFacePersonCropTestNode] YOLO runtime unavailable: {self.yolo_import.reason}')

        self.bridge = CvBridge()
        self.window_name = 'RetinaFace Person Crop Test'

        self.yolo_resolution = resolve_yolo_device(self.yolo_device_request, self.yolo_import.torch_module)
        self.retinaface_resolution = resolve_onnx_providers(self.retinaface_providers_request, ort)

        yolo_model_path = _resolve_model_path(self.get_parameter('yolo_model_path').value)
        self.get_logger().info(
            f'[RetinaFacePersonCropTestNode] Loading YOLO model={yolo_model_path} device={self.yolo_resolution.resolved}'
        )
        self.model = self.yolo_import.yolo_class(yolo_model_path)

        self.retina: Optional[RetinaFaceOnnx] = None
        if self.retinaface_enabled and self.retinaface_resolution.enabled:
            self.get_logger().info(
                '[RetinaFacePersonCropTestNode] '
                f'Loading RetinaFace model={self.retinaface_onnx_path} providers={self.retinaface_resolution.resolved}'
            )
            self.retina = RetinaFaceOnnx(
                onnx_path=self.retinaface_onnx_path,
                providers=self.retinaface_resolution.resolved,
                input_w=self.retinaface_input_w,
                input_h=self.retinaface_input_h,
                conf_thresh=self.retinaface_conf_thresh,
                nms_iou=self.retinaface_nms_iou,
                out_scores=self.retinaface_out_scores,
                out_boxes=self.retinaface_out_boxes,
                out_landmarks=self.retinaface_out_landmarks,
                logger=self.get_logger(),
            )
            if not self.retina.enabled:
                self.retina = None
                self.get_logger().warn('[RetinaFacePersonCropTestNode] RetinaFace disabled after load failure')
        elif self.retinaface_enabled:
            self.get_logger().warn(
                '[RetinaFacePersonCropTestNode] '
                f'RetinaFace disabled: {self.retinaface_resolution.reason}'
            )
        else:
            self.get_logger().info('[RetinaFacePersonCropTestNode] RetinaFace disabled by parameter')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)
        self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info(
            '[RetinaFacePersonCropTestNode] Started '
            f'rgb_topic={self.rgb_topic} debug_image_topic={self.debug_image_topic} '
            f'retinaface={"enabled" if self.retina is not None else "disabled"}'
        )

    def rgb_cb(self, msg: Image):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'[RetinaFacePersonCropTestNode] RGB cv_bridge failed: {exc}')
            return

        annotated = rgb_img.copy()
        detections = self._run_yolo(rgb_img)

        for idx, (x1, y1, x2, y2, conf) in enumerate(detections):
            crop_bounds = self._clamp_bbox((x1, y1, x2, y2), annotated.shape[1], annotated.shape[0])
            if crop_bounds is None:
                continue
            px1, py1, px2, py2 = crop_bounds

            cv2.rectangle(annotated, (px1, py1), (px2, py2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f'person {conf:.2f}',
                (px1, max(py1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            face_count = 0
            best_face_score = 0.0
            if self.retina is not None:
                person_crop = rgb_img[py1:py2, px1:px2]
                if person_crop.size > 0:
                    boxes, scores, landmarks = self.retina.detect(person_crop)
                    face_count = int(boxes.shape[0])
                    best_face_score = float(scores.max()) if scores.size > 0 else 0.0
                    for face_box, face_score, lm in zip(boxes, scores, landmarks):
                        fx1, fy1, fx2, fy2 = face_box.tolist()
                        gx1 = int(round(px1 + fx1))
                        gy1 = int(round(py1 + fy1))
                        gx2 = int(round(px1 + fx2))
                        gy2 = int(round(py1 + fy2))
                        mapped = self._clamp_bbox((gx1, gy1, gx2, gy2), annotated.shape[1], annotated.shape[0])
                        if mapped is None:
                            continue
                        mx1, my1, mx2, my2 = mapped
                        cv2.rectangle(annotated, (mx1, my1), (mx2, my2), (0, 165, 255), 2)
                        cv2.putText(
                            annotated,
                            f'face {float(face_score):.2f}',
                            (mx1, min(my2 + 18, annotated.shape[0] - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (0, 165, 255),
                            2,
                        )
                        for pt in lm:
                            lx = int(round(px1 + float(pt[0])))
                            ly = int(round(py1 + float(pt[1])))
                            if 0 <= lx < annotated.shape[1] and 0 <= ly < annotated.shape[0]:
                                cv2.circle(annotated, (lx, ly), 2, (255, 255, 0), -1)

            self.get_logger().info(
                f'[RetinaFacePersonCropTestNode] person[{idx}] '
                f'bbox=({px1},{py1},{px2},{py2}) conf={conf:.2f} '
                f'faces={face_count} best_face_score={best_face_score:.2f}'
            )

        self._publish_debug_image(annotated, msg)

    def _run_yolo(self, bgr_img):
        results = self.model.predict(
            source=bgr_img,
            imgsz=640,
            conf=self.yolo_conf_thresh,
            verbose=False,
            device=self.yolo_resolution.resolved,
        )
        if not results:
            return []

        res = results[0]
        if res.boxes is None:
            return []

        detections = []
        for box in res.boxes:
            cls_id = int(box.cls[0].item())
            if cls_id != 0:
                continue
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
            detections.append((x1, y1, x2, y2, conf))
        return detections

    @staticmethod
    def _clamp_bbox(bbox_xyxy, img_w: int, img_h: int):
        x1, y1, x2, y2 = bbox_xyxy
        x1 = int(max(0, min(img_w - 1, round(x1))))
        y1 = int(max(0, min(img_h - 1, round(y1))))
        x2 = int(max(0, min(img_w, round(x2))))
        y2 = int(max(0, min(img_h, round(y2))))
        if x2 <= x1 + 2 or y2 <= y1 + 2:
            return None
        return x1, y1, x2, y2

    def _publish_debug_image(self, annotated, src_msg: Image):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = src_msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f'[RetinaFacePersonCropTestNode] Debug image publish failed: {exc}')

        if self.enable_gui:
            try:
                cv2.imshow(self.window_name, annotated)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().warn(f'[RetinaFacePersonCropTestNode] OpenCV GUI failed: {exc}')

    def destroy_node(self):
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RetinaFacePersonCropTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
