# 🤖 Eternal Codebase

A powerful, multi-workspace robotics and computer vision platform for **warehouse automation**, built using **ROS 2**, **OpenCV**, **micro-ROS**, and embedded systems integration.

This repository combines:
- Real-time rack scanning
- QR code detection
- Shelf occupancy analysis
- Autonomous navigation
- Human-Machine Interface (HMI)
- Robot bringup and SLAM
- Embedded micro-ROS support

into a single modular engineering ecosystem for intelligent warehouse robotics.

---

# 🚀 Project Highlights

## 🔍 `cv_ws` — Computer Vision Workspace
ROS 2 Python workspace containing the **`rack_scanner`** package.

### Features
- Detects horizontal rack beams across reflective and dark warehouse surfaces
- Performs shelf occupancy analysis
- Saves enriched scan metadata for inventory systems
- Optimized edge filtering and object masking
- Real-time camera stream processing

### Core Technologies
- ROS 2
- OpenCV
- NumPy
- Python

---

## 🏭 `dev_ws` / `dev2_ws` — Warehouse Robot Workspaces

Complete ROS 2 robot stacks for warehouse automation.

### Included Packages

#### 🚗 `warehouse_navigation`
- SLAM and localization
- Autonomous navigation
- Path planning
- TF broadcasting
- Navigation stack integration

#### 📷 `warehouse_scanning`
- QR code detection
- Camera-based inventory scanning
- Rack recognition pipeline

#### 🤖 `warehouse_robot_bringup`
- Robot launch orchestration
- Sensor initialization
- Navigation + SLAM startup

#### 🖥️ `warehouse_hmi`
- Human-machine interface
- Diagnostics and robot monitoring
- Operator controls

#### 📡 `warehouse_msgs`
- Shared ROS 2 custom message definitions

#### 🛰️ `sllidar_ros2`
- LiDAR integration
- Obstacle detection
- Environment perception

---

## ⚡ `micro_ros_raspberrypi_pico_sdk`
Embedded micro-ROS support for Raspberry Pi Pico.

### Features
- Low-level ROS connectivity
- Embedded sensor/actuator integration
- micro-ROS transport layers
- Real-time microcontroller communication

Ideal for extending the warehouse stack to lightweight embedded nodes.

---

# ✨ Why This Project Stands Out

- ✅ Modular multi-workspace ROS 2 architecture
- ✅ Real-time robotics + computer vision integration
- ✅ Warehouse-focused automation stack
- ✅ Embedded micro-ROS support
- ✅ Scalable perception and navigation pipelines
- ✅ Designed for autonomous inventory systems

---

# 💻 Tech Stack

### Robotics & Middleware
![ROS2](https://img.shields.io/badge/ROS2-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![micro-ROS](https://img.shields.io/badge/micro--ROS-22314E?style=for-the-badge&logo=ros&logoColor=white)

### Programming Languages
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Computer Vision & AI
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white)

### Robotics Hardware & Tools
![Raspberry Pi](https://img.shields.io/badge/RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)
![LiDAR](https://img.shields.io/badge/LiDAR-Robotics-blue?style=for-the-badge)

### Development & Build Tools
![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)
![Colcon](https://img.shields.io/badge/colcon-ROS2-blue?style=for-the-badge)
![Git](https://img.shields.io/badge/git-%23F05033.svg?style=for-the-badge&logo=git&logoColor=white)

---

# 📦 Recommended Workflow

## 🔨 Build the CV Workspace

```bash
cd cv_ws
colcon build --packages-select rack_scanner --symlink-install
source install/setup.bash
```

---

## 🔨 Build the Warehouse Robot Workspace

```bash
cd dev_ws
colcon build --symlink-install
source install/setup.bash
```

> If you use `dev2_ws`, the workflow is identical.

```bash
cd dev2_ws
colcon build --symlink-install
source install/setup.bash
```

---

# ▶️ Run Key Components

## 🚗 Rack Scanner

```bash
ros2 run rack_scanner process_rack
```

This node:
- subscribes to `/raw_image`
- performs rack beam detection
- analyzes shelf occupancy
- outputs enriched scan metadata

---

## 🤖 Warehouse Robot Bringup

Use launch files inside:

```bash
dev_ws/src/warehouse_robot_bringup/launch
```

to start:
- SLAM
- navigation stack
- LiDAR systems
- robot drivers
- perception pipelines

---

# 📂 Workspace Layout

```bash
Eternal-Codebase/
│
├── cv_ws/
│   ├── src/rack_scanner/
│   └── readme.md
│
├── dev_ws/
│   └── src/
│       ├── warehouse_hmi/
│       ├── warehouse_msgs/
│       ├── warehouse_navigation/
│       ├── warehouse_robot_bringup/
│       ├── warehouse_scanning/
│       └── sllidar_ros2/
│
├── dev2_ws/
│   └── src/
│
└── micro_ros_raspberrypi_pico_sdk/
    └── embedded micro-ROS support
```

---

# 📊 System Capabilities

- Real-time rack scanning and occupancy analysis
- Autonomous warehouse robot navigation
- QR-based inventory identification
- LiDAR mapping and localization
- Embedded sensor integration with micro-ROS
- ROS 2 distributed communication architecture

---

# 🚀 Future Improvements

- AI-powered object detection using YOLO
- Multi-camera warehouse perception
- Cloud-based inventory analytics
- Fleet management system
- Autonomous pallet detection
- Web dashboard for robot monitoring

---

# 🤝 Contributing

Contributions are welcome from:
- robotics engineers
- ROS 2 developers
- computer vision specialists
- embedded systems developers
- warehouse automation researchers

Feel free to fork the repository and improve the platform.

---

# 📜 License

Most packages in this workspace use the **Apache-2.0 License** as indicated in their package metadata.