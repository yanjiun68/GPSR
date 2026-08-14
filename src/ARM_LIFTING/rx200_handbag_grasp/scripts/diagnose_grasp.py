#!/usr/bin/env python3
"""
Diagnostic script for RX200 Grasp System
检查TF树、节点状态和抓取配置

Usage:
    python3 diagnose_grasp.py
"""

import subprocess
import time
import sys


def run_cmd(cmd, timeout=5):
    """Run a command and return output."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.stdout.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "TIMEOUT", -1
    except Exception as e:
        return str(e), -1


def check_nodes():
    """Check running ROS2 nodes."""
    print("\n" + "=" * 60)
    print("1. RUNNING NODES")
    print("=" * 60)
    
    output, _ = run_cmd("ros2 node list 2>/dev/null")
    nodes = output.split('\n') if output else []
    
    required_nodes = [
        ('grasp_node', '/grasp_node'),
        ('move_group', '/move_group'),
        ('camera', '/camera/camera'),
        ('yolo_detector', '/yolo_detector'),
        ('robot_state_publisher', '/robot_state_publisher'),
        ('lift_driver', '/lift_serial_driver'),
    ]
    
    for name, pattern in required_nodes:
        found = any(pattern in n for n in nodes)
        status = "✅" if found else "❌ MISSING"
        print(f"  {name}: {status}")
    
    print(f"\n  All nodes:\n    " + "\n    ".join(nodes[:15]))
    return len(nodes) > 3


def check_tf_tree():
    """Check TF tree connectivity."""
    print("\n" + "=" * 60)
    print("2. TF TREE CHECK")
    print("=" * 60)
    
    # Key transform: lift_base_link -> camera_color_optical_frame
    frames_to_check = [
        ('lift_base_link', 'rx200/base_link'),
        ('rx200/base_link', 'rx200/camera_mount_link'),
        ('rx200/camera_mount_link', 'camera_link'),
        ('camera_link', 'camera_color_optical_frame'),
    ]
    
    all_ok = True
    for parent, child in frames_to_check:
        cmd = f"timeout 2 ros2 run tf2_ros tf2_echo {parent} {child} 2>&1 | head -5"
        output, code = run_cmd(cmd, timeout=5)
        if 'Translation' in output or 'At time' in output:
            print(f"  ✅ {parent} -> {child}")
        else:
            print(f"  ❌ {parent} -> {child}: NOT CONNECTED")
            all_ok = False
    
    # Full chain check
    cmd = "timeout 2 ros2 run tf2_ros tf2_echo lift_base_link camera_color_optical_frame 2>&1 | head -5"
    output, _ = run_cmd(cmd, timeout=5)
    if 'Translation' in output:
        print(f"\n  ✅ FULL TF CHAIN OK: lift_base_link -> camera_color_optical_frame")
    else:
        print(f"\n  ❌ FULL TF CHAIN BROKEN!")
        print(f"     Output: {output[:200]}")
        all_ok = False
    
    return all_ok


def check_topics():
    """Check required topics."""
    print("\n" + "=" * 60)
    print("3. REQUIRED TOPICS")
    print("=" * 60)
    
    topics_to_check = [
        '/joint_states',
        '/yolo/detections',
        '/camera/camera/color/image_raw',
        '/camera/camera/aligned_depth_to_color/image_raw',
        '/grasp/target_pose',
        '/move_action',
    ]
    
    output, _ = run_cmd("ros2 topic list 2>/dev/null")
    all_topics = output.split('\n') if output else []
    
    all_ok = True
    for topic in topics_to_check:
        found = topic in all_topics
        status = "✅" if found else "❌ MISSING"
        print(f"  {topic}: {status}")
        if not found:
            all_ok = False
    
    return all_ok


def check_grasp_pose():
    """Check if grasp pose is being published."""
    print("\n" + "=" * 60)
    print("4. GRASP POSE STATUS")
    print("=" * 60)
    
    cmd = "timeout 3 ros2 topic echo /grasp/target_pose --once 2>&1 | head -15"
    output, code = run_cmd(cmd, timeout=5)
    
    if 'position:' in output:
        print("  ✅ Grasp pose is being published!")
        # Extract position
        lines = output.split('\n')
        for line in lines:
            if 'x:' in line or 'y:' in line or 'z:' in line:
                print(f"    {line.strip()}")
        return True
    else:
        print("  ❌ No grasp pose detected!")
        print("     Possible causes:")
        print("     1. YOLO detection not finding objects")
        print("     2. TF transform failing")
        print("     3. Target out of workspace")
        return False


def check_action_server():
    """Check MoveIt action server."""
    print("\n" + "=" * 60)
    print("5. MOVEIT ACTION SERVER")
    print("=" * 60)
    
    output, _ = run_cmd("ros2 action list 2>/dev/null")
    
    if '/move_action' in output:
        print("  ✅ /move_action server available")
        return True
    else:
        print("  ❌ /move_action server NOT available!")
        print("     MoveIt may not be running properly")
        return False


def main():
    print("\n" + "#" * 60)
    print("# RX200 GRASP SYSTEM DIAGNOSTIC")
    print("#" * 60)
    
    results = {
        'nodes': check_nodes(),
        'tf': check_tf_tree(),
        'topics': check_topics(),
        'grasp_pose': check_grasp_pose(),
        'action': check_action_server(),
    }
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    all_ok = all(results.values())
    for name, ok in results.items():
        status = "✅ OK" if ok else "❌ FAIL"
        print(f"  {name}: {status}")
    
    if all_ok:
        print("\n✅ System ready! Try: ros2 service call /grasp/execute std_srvs/srv/Trigger")
    else:
        print("\n❌ System has issues. Fix the problems above first.")
        
        if not results['nodes']:
            print("\n⚠️  Make sure to run: ros2 launch rx200_handbag_grasp all_in_one.launch.py")
        if not results['tf']:
            print("\n⚠️  TF tree is broken. Check camera_tf_publisher and robot_state_publisher")
    
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
