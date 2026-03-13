# aicar_simulation

基于 ROS2 Humble 的 AI 小车仿真项目。

## 环境依赖

### ROS2 版本

- **ROS2 Humble**（Ubuntu 22.04）

### 必要依赖包

在运行仿真之前，请确保已安装以下 ROS2 包：

#### 1. xacro

用于解析机器人描述文件（URDF/XACRO）。

```bash
sudo apt install ros-humble-xacro
```

#### 2. gazebo_ros_pkgs

提供 Gazebo 与 ROS2 的集成支持，包含 `gazebo_ros`、`gazebo_plugins` 等组件。

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

#### 3. joint_state_publisher_gui

提供关节状态发布器的图形界面，用于 URDF 模型可视化调试。

```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

#### 一键安装所有依赖

```bash
sudo apt update && sudo apt install \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui
```

---

## 构建与运行

```bash
# 1. Source ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 构建项目
cd ~/ai_car
colcon build

# 3. Source 工作空间
source install/setup.bash

# 4. 启动仿真
ros2 launch aicar_simulation gazebo.launch.py
```

> 建议将 `source /opt/ros/humble/setup.bash` 添加到 `~/.bashrc`，避免每次手动执行。
