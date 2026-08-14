#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def main(args=None):
    # 1. Initialize ROS 2 Node
    rclpy.init(args=args)
    node = rclpy.create_node('yolo_distance_detector')
    publisher = node.create_publisher(Bool, 'person_selected', 10)
    node.get_logger().info("Node started. Initializing camera and YOLO...")

    # 2. Initialize YOLOv8 (Using the nano model for speed)
    model = YOLO('yolo11m.pt') 

    # 3. Initialize RealSense directly in Python
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable Depth and Color streams (640x480 at 30 FPS is standard and fast)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the camera pipeline
    profile = pipeline.start(config)

    # Create an alignment object to align depth frames to color frames
    # This ensures the depth pixel perfectly matches the YOLO color pixel
    align_to = rs.stream.color
    align = rs.align(align_to)

    node.get_logger().info("Camera started. Press 'q' in the image window to quit.")

    try:
        while rclpy.ok():
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Run YOLOv8 inference on the color image
            results = model(color_image, stream=True, verbose=False)

            person_in_range = False

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Check if the detected class is 'person' (Class ID 0 in COCO dataset)
                    class_id = int(box.cls[0])
                    if class_id == 0: 
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # Calculate the center point of the bounding box
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # Get distance from the depth frame at the center point (in meters)
                        distance = depth_frame.get_distance(center_x, center_y)

                        # Default bounding box color: Green
                        color = (0, 255, 0)
                        
                        # If distance is valid and <= 3.5m, change color to Red and trigger boolean
                        if 0.0 < distance <= 3.5:
                            color = (0, 0, 255) # BGR for Red
                            person_in_range = True
                            
                        # Draw bounding box
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                        
                        # Draw label and distance
                        label = f"Person: {distance:.2f}m"
                        cv2.putText(color_image, label, (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        
                        # Draw a small circle at the center point where depth is measured
                        cv2.circle(color_image, (center_x, center_y), 4, color, -1)

            # Publish the boolean topic
            msg = Bool()
            msg.data = person_in_range
            publisher.publish(msg)

            # Show the image using OpenCV
            cv2.imshow('RealSense YOLOv8 Detection', color_image)

            # Process ROS 2 callbacks and handle OpenCV window events
            rclpy.spin_once(node, timeout_sec=0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")

    finally:
        # Clean up resources
        node.get_logger().info("Shutting down...")
        pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()