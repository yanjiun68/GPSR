#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def is_chair_occupied(chair_box, person_boxes, overlap_threshold=0.3):
    cx1, cy1, cx2, cy2 = chair_box
    chair_area = (cx2 - cx1) * (cy2 - cy1)
    
    for px1, py1, px2, py2 in person_boxes:
        x_left = max(cx1, px1)
        y_top = max(cy1, py1)
        x_right = min(cx2, px2)
        y_bottom = min(cy2, py2)
        
        if x_right > x_left and y_bottom > y_top:
            intersection_area = (x_right - x_left) * (y_bottom - y_top)
            overlap_ratio = intersection_area / float(chair_area)
            if overlap_ratio > overlap_threshold:
                return True
    return False

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('chair_status_detector')
    
    publisher_empty = node.create_publisher(String, 'empty_chairs', 10)
    publisher_occupied = node.create_publisher(String, 'occupied_chairs', 10)
    
    node.get_logger().info("Node started. Loading AI models...")

    # --- DUAL MODEL SETUP ---
    # 1. Your custom trained model for chairs (Update this path!)
    chair_model = YOLO('/home/upm-m15/workspace/nic/src/vision/models/detection_pt/chair.pt') 
    
    # 2. Standard Nano model just for finding people (fastest model to save CPU)
    person_model = YOLO('yolo11n.pt') 

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)

    node.get_logger().info("Models loaded and Camera started. Press 'q' to quit.")

    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            height, width, _ = color_image.shape
            left_boundary = width // 3
            right_boundary = 2 * (width // 3)

            cv2.line(color_image, (left_boundary, 0), (left_boundary, height), (0, 255, 255), 2)
            cv2.line(color_image, (right_boundary, 0), (right_boundary, height), (0, 255, 255), 2)

            # --- RUN BOTH MODELS ---
            chair_results = chair_model(color_image, stream=True, verbose=False)
            person_results = person_model(color_image, stream=True, verbose=False)
            
            chairs = []
            persons = []
            empty_chairs_data = []
            occupied_chairs_data = []

            # Extract Chairs from YOUR custom model
            # Note: Assuming class_id 0 is 'chair' in your custom training. 
            # If you trained it differently, change class_id == 0 below.
            for result in chair_results:
                for box in result.boxes:
                    if int(box.cls[0]) == 0 and float(box.conf[0]) > 0.60: 
                        chairs.append(tuple(map(int, box.xyxy[0])))

            # Extract Persons from the Standard Nano model (Class 0 is Person in COCO)
            for result in person_results:
                for box in result.boxes:
                    if int(box.cls[0]) == 0 and float(box.conf[0]) > 0.60: 
                        persons.append(tuple(map(int, box.xyxy[0])))

            # Evaluate every chair
            for chair_box in chairs:
                cx1, cy1, cx2, cy2 = chair_box
                center_x = int((cx1 + cx2) / 2)
                center_y = int((cy1 + cy2) / 2)

                if center_x < left_boundary:
                    position = "LEFT"
                elif center_x > right_boundary:
                    position = "RIGHT"
                else:
                    position = "CENTER"

                occupied = is_chair_occupied(chair_box, persons)

                y_min, y_max = max(0, center_y - 5), min(depth_image.shape[0], center_y + 5)
                x_min, x_max = max(0, center_x - 5), min(depth_image.shape[1], center_x + 5)
                depth_region = depth_image[y_min:y_max, x_min:x_max]
                valid_depths = depth_region[depth_region > 0]
                distance = np.median(valid_depths) * depth_scale if valid_depths.size > 0 else 0.0

                if occupied:
                    color = (0, 0, 255) 
                    label = f"Occupied {position}"
                    occupied_chairs_data.append(f"[{position}: Occupied]")
                else:
                    color = (0, 255, 0) 
                    label = f"Empty {position}: {distance:.2f}m"
                    empty_chairs_data.append(f"[{position}: {distance:.2f}m]")

                cv2.rectangle(color_image, (cx1, cy1), (cx2, cy2), color, 2)
                cv2.putText(color_image, label, (cx1, cy1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.circle(color_image, (center_x, center_y), 4, color, -1)

            # Draw Person boxes (Blue)
            for px1, py1, px2, py2 in persons:
                cv2.rectangle(color_image, (px1, py1), (px2, py2), (255, 0, 0), 1)

            if empty_chairs_data:
                msg_empty = String()
                msg_empty.data = " | ".join(empty_chairs_data)
                publisher_empty.publish(msg_empty)
                
            if occupied_chairs_data:
                msg_occupied = String()
                msg_occupied.data = " | ".join(occupied_chairs_data)
                publisher_occupied.publish(msg_occupied)

            cv2.imshow('RealSense Chair Status Detector', color_image)

            rclpy.spin_once(node, timeout_sec=0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")

    finally:
        node.get_logger().info("Shutting down...")
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()