# RX200 椅子指向系统

基于 YOLO 视觉检测的 RX200 机械臂椅子指向流程。

## 系统组成

| 组件 | 说明 |
|------|------|
| 机械臂 | Interbotix RX200 (5-DOF) |
| 升降柱 | 定制升降机构 (0-275mm) |
| 相机 | Intel RealSense D435 |
| 视觉模型 | YOLO 椅子检测模型 |

## 目录结构

```
rx200_handbag_grasp/
├── config/                 # 配置文件
│   ├── camera_tf.yaml      # 相机标定结果
│   ├── combined.srdf.xacro # MoveIt SRDF
│   ├── controllers.yaml    # 控制器配置
│   ├── grasp_params.yaml   # 抓取参数
│   ├── joint_limits.yaml   # 关节限位
│   ├── kinematics.yaml     # 运动学配置
│   ├── ompl_planning.yaml  # OMPL 规划配置
├── launch/                 # 启动文件
│   ├── all_in_one.launch.py         # 一键启动（Step1+2+3）
│   ├── step1_startup.launch.py      # 机械臂 + 升降柱 + MoveIt
│   ├── step2_perception.launch.py   # 相机 + YOLO 椅子检测
│   ├── step3_point.launch.py        # Step3 指向执行
│   ├── step3_grasp.launch.py        # 旧兼容入口（转发到 Step3 指向）
│   ├── step4_detect_hand_bag.launch.py # 手指选包
│   └── display.launch.py            # URDF 显示测试
├── models/                 # YOLO 模型存放目录
│   └── best.pt             # 训练好的模型（需用户添加）
├── rviz/                   # RViz 配置
├── scripts/                # Python 脚本
│   ├── autonomous_reach_grab_coordinator.py # 自主协同流程
│   ├── camera_tf_publisher.py  # 运行时 TF 发布
│   ├── hand_bag_selector.py    # 手指指向选包
│   ├── grasp_node.py           # 核心抓取逻辑
│   └── yolo_handle_detector.py # YOLO11 检测节点
├── urdf/                   # 机器人描述
│   └── combined.urdf.xacro # RX200 + 升降柱 组合模型
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 快速开始

### 1. 安装依赖

```bash
# ROS2 依赖
sudo apt install ros-humble-realsense2-camera

# Python 依赖
pip install ultralytics scipy pyyaml
```

### 2. 构建

```bash
cd ~/arm_ws
colcon build --packages-select rx200_handbag_grasp
source install/setup.bash
```

### 3. 放置模型

将椅子检测模型放入 `models/chair.pt`

### 4. 启动系统（分步）

```bash
# 终端 1: 机械臂 + 升降柱 + MoveIt
ros2 launch rx200_handbag_grasp step1_startup.launch.py

# 终端 2: 相机 + YOLO 椅子检测
ros2 launch rx200_handbag_grasp step2_perception.launch.py

# 终端 3: Step3 指向节点
ros2 launch rx200_handbag_grasp step3_point.launch.py
```

Step3 流程：
- 先到 `Ready`
- 连续检测到 `chair` 2 秒后转到 `point`
- 保持 2 秒
- 最后回到 `Sleep`

### 5. 一键启动（可选）

```bash
ros2 launch rx200_handbag_grasp all_in_one.launch.py
```

### 7. Step4：手指指向最近包检测

```bash
# 先启动 Step2（相机 + YOLO）
ros2 launch rx200_handbag_grasp step2_perception.launch.py \
  yolo_model:=/home/teo/Downloads/bag_27_Feb.pt

# 再单独启动 Step4（手指选包）
ros2 launch rx200_handbag_grasp step4_detect_hand_bag.launch.py
```

Step4 输出：
- `/handbag/selected_detection`：当前选中的最近包（Detection2D）
- `/handbag/selected_detections`：数组版（0 或 1 个）
- `/handbag/selected`：是否选中目标（Bool）
- `/handbag/pointing_annotated/compressed`：可视化图像
- `/handbag/check_detection`：服务检查当前是否检测成功（Trigger）

```bash
ros2 service call /handbag/check_detection std_srvs/srv/Trigger
```

## 参数配置

编辑 `config/grasp_params.yaml` 调整：
- 工作空间边界
- 预抓取距离
- 夹爪 PWM 值
- 升降柱位置

## 详细文档

完整操作手册请参考：`~/arm_ws/操作手册.md`

## License

BSD-3-Clause
