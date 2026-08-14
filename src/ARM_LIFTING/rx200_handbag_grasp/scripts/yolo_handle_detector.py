#!/usr/bin/env python3
"""
YOLO11 Bag Handle Detector Node

Detects bag handles using a custom-trained YOLO11 model.
Publishes detection results as vision_msgs/Detection2DArray.

Key Features:
- Uses CompressedImage to avoid cv_bridge OpenCV version conflict
- Compatible with ROS2 Humble + ultralytics (YOLO11)
- No virtual environment required

Parameters:
- model_path: Path to the trained YOLO model (in models/ directory or absolute path)
- confidence_threshold: Minimum confidence for valid detections (0.0-1.0)
- device: "cuda" for GPU, "cpu" for CPU inference
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
import cv2
import numpy as np
import os


class YoloHandleDetector(Node):
    """YOLO11 detector for bag handles."""

    def __init__(self):
        super().__init__('yolo_handle_detector')

        # ========================================
        # Parameters
        # ========================================
        self.declare_parameter('model_path', 'best.pt')  # Legacy/Custom
        self.declare_parameter('confidence_threshold', 0.1)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('annotated_image_topic', '/yolo/image_annotated/compressed')
        self.declare_parameter('mask_topic', '/yolo/mask')
        self.declare_parameter('detection_rate_hz', 10.0)
        self.declare_parameter('max_detections', 1)
        self.declare_parameter('one_per_class', False)
        
        # New Flexible Parameters
        self.declare_parameter('model_type', 'yoloworld') # yolo, yoloworld, sam
        self.declare_parameter('model_size', 's')         # n, s, m, l, x
        # Refined prompts for object detection
        # YOLO World works best with simple, common object names
        self.declare_parameter('target_classes', [
            'box',              # Simple and universal
            'cardboard box',    # Common packaging
            'package',          # Generic package
            'carton',           # Carton/box
            'container',        # Container
        ])
        self.declare_parameter('filter_target_classes', False)
        # Optional class name remap for custom YOLO models whose class names are numeric
        # (e.g. "5"), so downstream labels can be semantic names (e.g. "bag handle").
        self.declare_parameter('class_name_map', ['5:bag handle'])

        self.model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self.mask_topic = self.get_parameter('mask_topic').value
        rate_hz = self.get_parameter('detection_rate_hz').value
        self.max_detections = int(self.get_parameter('max_detections').value)
        self.one_per_class = bool(self.get_parameter('one_per_class').value)
        
        self.model_type = self.get_parameter('model_type').value
        self.model_size = self.get_parameter('model_size').value
        self.target_classes = self.get_parameter('target_classes').value
        self.filter_target_classes = bool(self.get_parameter('filter_target_classes').value)
        self.target_class_filter = {
            str(class_name).strip().lower().replace('_', ' ')
            for class_name in self.target_classes
        }
        self.class_name_map_raw = self.get_parameter('class_name_map').value
        self.class_name_map = self._parse_class_name_map(self.class_name_map_raw)

        # SAM Parameters
        self.declare_parameter('use_sam_segmentation', True)
        self.declare_parameter('sam_model_size', 't')
        
        self.use_sam = self.get_parameter('use_sam_segmentation').value
        self.sam_model_size = self.get_parameter('sam_model_size').value
        self.sam_model = None

        # ========================================
        # Initialize
        # ========================================
        self.model = None
        self.latest_image = None
        self.image_header = None

        # Load model
        # Load models
        self._load_model()
        if self.use_sam:
            self._load_sam()

        # ========================================
        # ROS Interface
        # ========================================
        self.image_sub = self.create_subscription(
            CompressedImage, image_topic, self._image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10
        )
        self.viz_pub = self.create_publisher(
            CompressedImage, self.annotated_image_topic, 10
        )
        # [NEW] Mask publisher for depth sampling
        self.mask_pub = self.create_publisher(
            Image, self.mask_topic, 10
        )

        # Store latest mask for grasp node
        self.latest_mask = None

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._detection_timer)

        self.get_logger().info(f'YOLO/SAM Detector initialized')
        self.get_logger().info(f'  Type: {self.model_type.upper()} ({self.model_size})')
        self.get_logger().info(f'  Model: {self.final_model_name}')
        self.get_logger().info(f'  Classes: {self.target_classes}')
        self.get_logger().info(f'  Filter target classes: {self.filter_target_classes}')
        self.get_logger().info(f'  Detections topic: {self.detections_topic}')
        self.get_logger().info(f'  Annotated image topic: {self.annotated_image_topic}')
        self.get_logger().info(f'  Mask topic: {self.mask_topic}')
        self.get_logger().info(
            f'  Max detections/frame: {self.max_detections} | one_per_class: {self.one_per_class}'
        )
        if self.class_name_map:
            self.get_logger().info(f'  Class remap: {self.class_name_map}')

    def _limit_detections(self, detections_meta):
        """Limit detections by confidence, optionally keeping only one per class."""
        if not detections_meta:
            return []

        # Highest confidence first.
        ordered = sorted(detections_meta, key=lambda d: d['conf'], reverse=True)

        if self.one_per_class:
            best_per_class = {}
            for det in ordered:
                cls_name = det['cls_name']
                if cls_name not in best_per_class:
                    best_per_class[cls_name] = det
            ordered = sorted(best_per_class.values(), key=lambda d: d['conf'], reverse=True)

        if self.max_detections > 0:
            ordered = ordered[:self.max_detections]

        return ordered

    def _parse_class_name_map(self, raw_entries):
        """Parse ['src:dst', ...] into a dict, ignoring malformed entries."""
        mapping = {}
        if not isinstance(raw_entries, (list, tuple)):
            return mapping
        for entry in raw_entries:
            if not isinstance(entry, str) or ':' not in entry:
                continue
            src, dst = entry.split(':', 1)
            src = src.strip()
            dst = dst.strip()
            if src and dst:
                mapping[src] = dst
        return mapping

    def _remap_class_name(self, cls_name, cls_id):
        """
        Remap class names for custom models.
        Priority:
        1) map by original class name string
        2) map by class id string
        """
        if not self.class_name_map:
            return cls_name
        key_name = str(cls_name)
        key_id = str(cls_id)
        if key_name in self.class_name_map:
            return self.class_name_map[key_name]
        if key_id in self.class_name_map:
            return self.class_name_map[key_id]
        return cls_name

    def _load_model(self):
        """Load Model based on type and size."""
        try:
            from ultralytics import YOLO, SAM

            self.final_model_name = ""

            if self.model_type == 'sam':
                # Map sizes to SAM2 models
                # sam2_t, sam2_s, sam2_b, sam2_l
                size_map = {'n': 't', 's': 's', 'm': 'b', 'l': 'l', 'x': 'l'}
                suffix = size_map.get(self.model_size, 'b')
                self.final_model_name = f'sam2_{suffix}.pt'
                
                self.get_logger().info(f'Loading SAM2 model: {self.final_model_name}')
                self.model = SAM(self.final_model_name)
                # SAM doesn't support set_classes
                
            elif self.model_type == 'yoloworld':
                # Map sizes to YOLO-World models
                # yolov8{sz}-worldv2.pt
                self.final_model_name = f'yolov8{self.model_size}-worldv2.pt'
                self.get_logger().info(f'Loading YOLO-World model: {self.final_model_name}')
                self.model = YOLO(self.final_model_name)
                self.model.set_classes(self.target_classes)
                
            else:
                # Custom/Legacy YOLO
                self.final_model_name = self.model_path
                self.get_logger().info(f'Loading Custom YOLO model: {self.final_model_name}')
                self.model = YOLO(self.final_model_name)

            # Move to device
            # Note: SAM models might handle device differently in ultralytics, but usually .to() works or it auto-detects
            # self.model.to(self.device) # ultralytics handles this automatically usually
            self.get_logger().info('Model loaded successfully')

        except ImportError:
            self.get_logger().error('ultralytics not installed!')
            self.model = None
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None

    def _image_callback(self, msg: CompressedImage):
        """Store latest compressed image."""
        self.latest_image = msg
        self.image_header = msg.header

    def _decompress_image(self, compressed_msg: CompressedImage) -> np.ndarray:
        """Decompress image without cv_bridge."""
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _compress_image(self, cv_image: np.ndarray) -> CompressedImage:
        """Compress image without cv_bridge."""
        msg = CompressedImage()
        msg.header = self.image_header
        msg.format = "jpeg"
        _, encoded = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        msg.data = encoded.tobytes()
        return msg

    def _detection_timer(self):
        """Run detection on latest image."""
        if self.latest_image is None or self.model is None:
            return

        try:
            # Decompress image (no cv_bridge needed!)
            cv_image = self._decompress_image(self.latest_image)

            # Run YOLO11 inference
            # Ensure RGB (OpenCV is BGR)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.model(rgb_image, conf=self.conf_thresh, verbose=False)

            # Build Detection2DArray message
            det_array = Detection2DArray()
            det_array.header = self.image_header

            annotated_image = cv_image.copy()

            # Prepare detections list for refinement
            detections_meta = []
            
            for result in results:
                if result.boxes is None:
                    continue

                for box in result.boxes:
                    # Extract bbox (xyxy format)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.model.names.get(cls_id, 'handle')
                    cls_name = self._remap_class_name(cls_name, cls_id)
                    if self.filter_target_classes and not self._is_target_class(cls_name):
                        continue
                    
                    # Store metadata for refinement
                    detections_meta.append({
                        'bbox_raw': [x1, y1, x2, y2],
                        'conf': conf,
                        'cls_name': cls_name,
                        'center': ((x1 + x2) / 2, (y1 + y2) / 2), # Default Center
                        'size': (x2 - x1, y2 - y1),               # Default Size
                        'refined': False
                    })
            
            # Run SAM Refinement (Modify detections_meta in place)
            detections_meta = self._limit_detections(detections_meta)
            if self.use_sam and len(detections_meta) > 0:
                self._refine_with_sam(rgb_image, detections_meta, results)

            for meta in detections_meta:
                 # Create Detection2D message
                det = Detection2D()
                det.header = self.image_header

                # Bounding box (center + size format)
                # Use refined center and size
                det.bbox.center.position.x = float(meta['center'][0])
                det.bbox.center.position.y = float(meta['center'][1])
                det.bbox.size_x = float(meta['size'][0])
                det.bbox.size_y = float(meta['size'][1])
                # Include orientation (angle in radians)
                det.bbox.center.theta = float(meta.get('angle', 0.0))

                # Hypothesis (class + confidence)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = meta['cls_name']
                hyp.hypothesis.score = meta['conf']
                det.results.append(hyp)

                det_array.detections.append(det)

                # Draw on annotated image
                # Re-extract logical coordinates for drawing
                cx, cy = meta['center']
                w, h = meta['size']
                
                # Draw Box (Green for refined, Yellow for raw if needed, but here we show final)
                # Compute top-left/bottom-right from center/size
                x1 = int(cx - w/2)
                y1 = int(cy - h/2)
                x2 = int(cx + w/2)
                y2 = int(cy + h/2)
                
                color = (0, 255, 0) # Green
                if meta['refined']:
                    color = (0, 255, 255) # Cyan for SAM refined
                    # Draw contour if available
                    if 'contour' in meta:
                         cv2.drawContours(annotated_image, [meta['contour']], -1, (255, 0, 255), 2)
                    # Draw centroid
                    cv2.circle(annotated_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                label = f"{meta['cls_name']} (SAM)" if meta['refined'] else f"{meta['cls_name']}"
                cv2.putText(annotated_image, f"{label} {meta['conf']:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Publish detections
            self.detection_pub.publish(det_array)

            # Publish visualization (compressed)
            viz_msg = self._compress_image(annotated_image)
            self.viz_pub.publish(viz_msg)

            # [NEW] Publish combined mask for depth sampling
            if len(detections_meta) > 0:
                self._publish_mask(cv_image.shape[:2], detections_meta)

            if len(det_array.detections) > 0:
                self.get_logger().debug(f'Detected {len(det_array.detections)} handle(s)')

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

    def _is_target_class(self, cls_name: str) -> bool:
        """Return True when a detected class should be kept by the optional filter."""
        normalized = str(cls_name).strip().lower().replace('_', ' ')
        return normalized in self.target_class_filter

    def _publish_mask(self, image_shape, detections_meta):
        """
        Publish combined binary mask of all detected objects.
        Used by grasp_node for mask-based depth sampling.

        Args:
            image_shape: (height, width) of the image
            detections_meta: List of detection metadata with contours
        """
        try:
            h, w = image_shape
            combined_mask = np.zeros((h, w), dtype=np.uint8)

            for meta in detections_meta:
                if meta.get('refined') and 'contour' in meta:
                    # Use SAM contour for precise mask
                    cv2.drawContours(combined_mask, [meta['contour']], -1, 255, -1)
                else:
                    # Fallback: Use bounding box
                    x1, y1, x2, y2 = meta['bbox_raw']
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    combined_mask[y1:y2, x1:x2] = 255

            # Store latest mask
            self.latest_mask = combined_mask

            # Publish mask as mono8 image
            mask_msg = Image()
            mask_msg.header = self.image_header
            mask_msg.height = h
            mask_msg.width = w
            mask_msg.encoding = 'mono8'
            mask_msg.is_bigendian = 0
            mask_msg.step = w
            mask_msg.data = combined_mask.tobytes()

            self.mask_pub.publish(mask_msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to publish mask: {e}')

    def _load_sam(self):
        """Load SAM model for checking."""
        try:
            from ultralytics import SAM
            # Map sizes to SAM2 models: t, s, b, l
            sam_model_name = f'sam2_{self.sam_model_size}.pt'
            self.get_logger().info(f'Loading SAM2 Refinement Model: {sam_model_name}...')
            self.sam_model = SAM(sam_model_name)
            self.get_logger().info('SAM2 Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load SAM model: {e}')
            self.use_sam = False

    def _refine_with_sam(self, image, detections, results):
        """
        Refine YOLO detections using SAM2 segmentation.
        
        Args:
            image: RGB image
            detections: List of detection dicts (mutable)
            results: YOLO results object (contains original boxes)
        """
        if not self.use_sam or self.sam_model is None or len(detections) == 0:
            return

        try:
            # IMPROVEMENT: Use Point Prompts (Center) instead of Box Prompts
            # This helps to segment only the surface under the center (Top) 
            # and avoid including side faces that are inside the box.
            
            points = []
            labels = []
            
            for det in detections:
                # Use Box Center as Prompt
                x1, y1, x2, y2 = det['bbox_raw']
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                points.append([cx, cy])
                labels.append(1) # 1 = Foreground
            
            # Predict masks using SAM with Points
            # ultralytics SAM call: results = model(image, points=points, labels=labels)
            # Note: For multiple objects, might need batch processing or loop
            # Ultralytics API for batch points is tricky, let's loop for safety and precision
            
            for i, det in enumerate(detections):
                 # Run SAM for each object individually with Point Prompt
                 res = self.sam_model(image, points=[points[i]], labels=[labels[i]], verbose=False)
                 
                 if len(res) == 0 or res[0].masks is None:
                     continue
                 
                 # Get mask (SAM can return multiple, we take the one with highest score index 0)
                 # data shape: [N, H, W]
                 mask = res[0].masks.data[0].cpu().numpy()
                 
                 # 1. Find contours of the mask
                 mask_uint8 = (mask * 255).astype(np.uint8)
                 contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                 if len(contours) == 0:
                    continue
                    
                 # Get largest contour
                 largest_contour = max(contours, key=cv2.contourArea)
                
                 # 2. Compute Centroid (Moments)
                 M = cv2.moments(largest_contour)
                 if M["m00"] != 0:
                    cX = M["m10"] / M["m00"]
                    cY = M["m01"] / M["m00"]
                    # Update detection
                    detections[i]['center'] = (cX, cY)
                    detections[i]['refined'] = True
                    
                 # 3. Compute Tight Bounding Box (Axis Aligned)
                 x, y, w, h = cv2.boundingRect(largest_contour)
                 detections[i]['bbox_tight'] = (x, y, x+w, y+h)
                 detections[i]['size'] = (float(w), float(h))
                 detections[i]['contour'] = largest_contour

                 # 4. Compute Rotated Rectangle (Orientation)
                 rect = cv2.minAreaRect(largest_contour)
                 # rect = ((cx, cy), (w, h), angle)
                 # angle is in degrees
                 angle_deg = rect[2]
                 
                 # Normalize angle to be consistent if needed (e.g., -90 to 90)
                 # For grasp alignment, we just want the principal axis
                 # Normalize angle to be consistent
                 # We want the angle of the SHORT edge (Width) for grasping
                 # OpenCV minAreaRect angle usually corresponds to the first side (rect[1][0])
                 # If rect[1][0] < rect[1][1], then angle is already for the short side.
                 # So we usually don't need to add 90.
                 
                 detections[i]['angle'] = float(np.radians(angle_deg)) # Store in radians

        except Exception as e:
            self.get_logger().warn(f'SAM refinement failed: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = YoloHandleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
