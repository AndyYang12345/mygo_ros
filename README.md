# MyGo —— 基于 ROS 2 Jazzy 的全自主移动操作机器人

> **MyGo Robot** — A ROS 2-based semi-autonomous mobile manipulation platform designed for robotics competitions (RoboMaster / similar events), featuring differential-drive chassis, 5-DOF robotic arm, dual-roller collection mechanism, and vision-guided autonomous tasks.

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

---

## 📖 项目简介 / Project Overview

MyGo 是一个面向机器人竞赛场景的半自主移动操作平台，能够完成底盘全向运动、五轴机械臂抓取、能量机关收集与提交等复杂任务。系统基于 **ROS 2 Jazzy** 分布式架构，采用**分层解耦**设计理念，通过 Xbox 手柄 + 类《终末地》派状 UI 实现直观操控。

### 🎯 核心能力

| 能力 | 描述 |
|------|------|
| **底盘运动控制** | 履带式差分驱动，支持速度/角速度实时控制 |
| **机械臂操作** | 5-DOF 总线舵机机械臂 + 二指夹爪，支持摇杆操控与预设轨迹 |
| **自主任务执行** | 有限状态机驱动的 8 大模式 + 14+ 子模式，覆盖收集、提交、越障等场景 |
| **视觉识别追踪** | 多色彩空间融合色块识别 + Kalman 滤波 ROI 追踪 + 双轴 PID 云台控制 |
| **仿真驱动开发** | PC 端 OpenCV 全链路仿真验证 → MaixCDK 交叉编译 → MaixCam2 嵌入式部署 |

### 🏗️ 系统架构

```
┌──────────────────────────────────────────────────────┐
│                   操作手柄 (Xbox)                      │
└─────────────────────┬────────────────────────────────┘
                      │ /joy
                      ▼
┌──────────────────────────────────────────────────────┐
│              joystick_parser (输入抽象层)              │
│   原始摇杆 → Button/Joystick/Combo/Trigger Intent     │
└─────────────────────┬────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────┐
│              state_machine (核心决策层)                │
│  ┌──────┐ ┌───────┐ ┌─────┐ ┌──────┐ ┌────────────┐ │
│  │ IDLE │ │CHASSIS│ │ ARM │ │ MENU │ │VISION_TASK │ │
│  ├──────┤ ├───────┤ ├─────┤ ├──────┤ ├────────────┤ │
│  │ POLE │ │BALL   │ │EMERG│ │  ... │ │    ...     │ │
│  └──────┘ └───────┘ └─────┘ └──────┘ └────────────┘ │
└─────┬───────────────┬───────────────┬───────┬────────┘
      │               │               │       │
      ▼               ▼               ▼       ▼
┌──────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐
│ /cmd/    │  │ /cmd/arm/    │  │ /cmd/gripper │  │  视觉模块 (TCP)   │
│ chassis  │  │ pose_target  │  │ /command     │  │  MaixCam2        │
└────┬─────┘  │ joint_target │  └──────┬───────┘  └────────┬─────────┘
     │        │ named_target │         │                   │
     ▼        └──────┬───────┘         ▼                   │
┌──────────────────────────────────────────────────┐       │
│        robot_hardware_interfaces (硬件驱动层)      │◄──────┘
│  chassis_serial │ arm_serial │ pole_serial │ ... │
└─────────────────────┬────────────────────────────┘
                      │ USB 模拟串口
                      ▼
┌──────────────────────────────────────────────────┐
│   STM32 嵌入式下位机 (USB 模拟串口)                │
│  F407(底盘) │ F103(机械臂/车载) │ 舵机 │ 电机      │
└──────────────────────────────────────────────────┘
```

### 🤖 硬件配置

| 组件 | 型号/规格 |
|------|-----------|
| **上位机** | Raspberry Pi 4B |
| **底盘驱动** | STM32F407ZET6 |
| **机械臂/车载驱动** | STM32F103C8T6 |
| **机械臂** | 5-DOF 总线舵机 + 二指夹爪 |
| **视觉模块** | Sipeed MaixCam2 |
| **底盘** | 履带式差分驱动 |
| **连接方式** | USB 模拟串口（替代传统杜邦线） |

---

## 📦 工作空间包结构 / Package Overview

| 包名 | 类型 | 描述 |
|------|------|------|
| [`custom_interfaces`](src/custom_interfaces/) | 接口定义 | 11 种自定义消息 + 1 种服务（JoystickIntent, ArmPoseTarget, GripperCommand 等） |
| [`joystick_parser`](src/joystick_parser/) | 输入抽象 | 将原始 `/joy` 数据解析为语义化 Intent（摇杆/按键/扳机/组合键） |
| [`state_machine`](src/state_machine/) | 核心决策 | 有限状态机引擎（8 大主状态 + 子模式）+ Qt5 派状 UI |
| [`mygo_robot_description`](src/mygo_robot_description/) | 机器人模型 | Xacro/URDF 模型定义、STL 网格碰撞模型 |
| [`mygo_moveit_config`](src/mygo_moveit_config/) | 运动规划 | MoveIt2 配置（IK 求解器、碰撞矩阵、规划组） |
| [`robot_hardware_interfaces`](src/robot_hardware_interfaces/) | 硬件驱动 | 底盘/机械臂/回收杆/收集器/相机串口通信节点 |
| [`mygo_bringup`](src/mygo_bringup/) | 启动编排 | Launch 文件 + 控制器配置 + RViz 配置 |
| [`mygo_car_description`](src/mygo_car_description/) | 整车模型 | 整车 URDF 及材质纹理 |

---

## 🚀 快速开始 / Quickstart

### 环境要求

- **OS**: Ubuntu 24.04（或兼容的 Linux 发行版）
- **ROS 2**: Jazzy Jalisco
- **编译器**: GCC 11+（C++17）
- **构建工具**: colcon
- **附加依赖**: Qt5 (Widgets), MoveIt2, xacro, ros2_control
- **硬件**: Xbox 兼容手柄（可选，仿真调试可用键盘替代）

### 1. 安装 ROS 2 Jazzy

参考官方文档安装 ROS 2 Jazzy：  
https://docs.ros.org/en/jazzy/Installation.html

### 2. 安装系统依赖

```bash
# 安装 MoveIt2 及相关包
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joy \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-rviz2

# 安装 Qt5 开发库（UI 界面需要）
sudo apt install -y \
  qtbase5-dev \
  libqt5widgets5

# 安装串口通信依赖
sudo apt install -y \
  libserial-dev
```

### 3. 克隆并构建工作空间

```bash
# 克隆仓库
git clone https://github.com/AndyYang12345/mygo_ros.git ~/mygo_ws
cd ~/mygo_ws

# 安装 ROS 依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译（使用优化参数）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境
source install/setup.bash
```

### 4. 启动系统

#### PC 端调试（不含机械臂 MoveIt）

```bash
ros2 launch mygo_bringup bringup.launch.xml
```

启动节点：
- `joy_node` — 手柄驱动
- `joystick_parser` — 输入解析
- `state_machine` — 状态机核心
- `chassis_serial_node` / `pole_serial_node` / `collector_serial_node` — 硬件串口

#### 树莓派全系统启动（含机械臂 MoveIt2）

```bash
ros2 launch mygo_bringup bringup_on_raspi.launch.xml
```

额外启动：
- `robot_state_publisher` — 发布机器人 TF 树
- `controller_manager` + `arm_controller` + `gripper_controller` — ros2_control 控制器
- `move_group` — MoveIt2 运动规划
- `arm_serial_node` / `camera_serial_node` / `commander` — 机械臂驱动与指令下发

#### 开机自启配置（树莓派）

```bash
# 复制 systemd 服务文件
sudo cp scripts/raspi/mygo_ros2_launch.service /etc/systemd/system/

# 启用服务
sudo systemctl enable mygo_ros2_launch.service
sudo systemctl start mygo_ros2_launch.service

# 查看状态
sudo systemctl status mygo_ros2_launch.service
```

### 5. 连接手柄

将 Xbox 兼容手柄通过 USB 或蓝牙连接到上位机，手柄输入将自动被 `joy_node` 捕获并经由 `joystick_parser` 解析为语义化指令。

### 6. 运行 UI 界面

在 PC 端（与树莓派同一局域网）：

```bash
source install/setup.bash
ros2 run state_machine menu_ui_node
```

UI 通过 ROS 2 话题与主控系统通信，可以实时显示机器人状态、当前模式、视觉连接状态等信息。

### 7. 手柄操作说明

| 操作 | 功能 |
|------|------|
| **LT（左扳机）** | 进入 MENU 模式 |
| **RT（右扳机）** | 确认 / 执行 |
| **LT + RT** | 双扳机折叠手臂 |
| **LB** | 切换子模式（CHASSIS 内） |
| **RB** | 切换机械臂预设位姿 |
| **左摇杆** | 底盘移动 / 机械臂 XY 控制 |
| **右摇杆** | 菜单导航 / 机械臂 Z + Yaw |
| **D-Pad** | 机械臂 Pitch 控制 |
| **A / B / X / Y** | 功能按键（依当前模式而定） |
| **START** | 急停（EMERGENCY） |

---

## 🧠 状态机设计

| 主状态 | 编号 | 功能简述 |
|--------|------|----------|
| **IDLE** | 1 | 启动默认状态，仅可进入 MENU，防误触 |
| **CHASSIS** | 2 | 底盘模式（含 HOME / 检测 / 收集 / 提交 / 越障等 8 个子模式） |
| **ARM** | 3 | 机械臂模式：摇杆控制关节 + 扳机控制夹爪 |
| **MENU** | 4 | 菜单选择：右摇杆导航 + 松扳机确认 |
| **POLE** | 5 | 回收杆模式：扳机模拟量控制旋转 |
| **BALL** | 6 | 球类收集模式 |
| **VISION_TASK** | 7 | 视觉任务模式：类比 ARM + 视觉锁定辅助 |
| **EMERGENCY** | 8 | 急停模式：立即刹停所有执行器 |

---

## 🔗 关联项目

| 项目 | 描述 | 仓库 |
|------|------|------|
| **MyGo ROS 上位机** | ROS 2 主控系统（本仓库） | [GitHub](https://github.com/AndyYang12345/mygo_ros.git) |
| **视觉模块** | 基于 MaixCDK 的嵌入式视觉 | [GitHub](https://github.com/AndyYang12345/mygo_via_maixcdk.git) |
| **PC 仿真环境** | OpenCV 视觉算法仿真 | [GitHub](https://github.com/AndyYang12345/MygoVison.git) |
| **个人博客** | 开发笔记与技术总结 | [Blog](https://andyyang12345.github.io/) |

---

## 🐛 已知问题 / Known Issues

- 整车 URDF 模型面数过高，树莓派加载需 1-2 分钟，目前仅保留机械臂部分 + 底座
- 视觉模块在复杂光照环境下识别鲁棒性有待提升
- 机械臂因未使用 ros2_control 的 JointTrajectoryController，无法实现系统级闭环控制（总线舵机内部自带角度闭环）
- 遗传算法 PID 整定仅适用于仿真环境，实机调参仍需人工迭代

---

## 📝 许可

本项目基于 Apache 2.0 许可证开源。

---

*Made with ❤️ by MyGo Team*
