# aicar_simulation

基于 ROS2 Humble 的 AI 小车 Gazebo 仿真项目，搭载 Ackermann 阿克曼驱动、3D 激光雷达、IMU 和 RGB-D 相机，用于 SLAM 建图、导航、视觉算法的仿真验证。

---

## 项目结构

```
aicar_simulation/
├── urdf/
│   ├── akermann_car_16.urdf.xacro   # 基础阿克曼小车模型（16线雷达）
│   └── jkgn_car_96.urdf.xacro       # 对齐真实车辆模型（96线雷达仿真）
├── launch/
│   ├── gazebo.launch.py             # Gazebo 仿真启动文件
│   └── display.launch.py            # RViz 模型预览启动文件
├── worlds/                          # 自定义仿真场景
└── config/                          # 配置文件
```

---

## 机器人模型

### 车体参数

| 参数 | 数值 |
|------|------|
| 车身长度 | 1.2m |
| 车身宽度 | 0.8m |
| 车身高度 | 0.4m |
| 轮子半径 | 0.18m |
| 轴距 | 1.0m |
| 驱动方式 | Ackermann 阿克曼四轮转向 |

### 传感器配置

| 传感器 | 参考型号 | 发布话题 |
|--------|---------|---------|
| 3D 激光雷达 | RoboSense Airy 96线 | `/velodyne_points` |
| IMU | - | `/imu` |
| RGB-D 相机 | Realsense D435 | `/camera/color/image_raw` |
| 深度相机 | Realsense D435 | `/camera/depth/image_raw` `/camera/depth/points` |

### 激光雷达仿真参数

真实雷达为 RoboSense Airy，以下为仿真参数与真实参数对比：

| 参数 | 真实雷达 | 仿真设置 | 说明 |
|------|---------|---------|------|
| 线数 | 96线 | 24线 | 降低以保证帧率 |
| 水平采样 | 900点 | 240点 | 降低以减少 CPU 压力 |
| 水平 FOV | 360° | 360° | 完全一致 |
| 垂直 FOV | 0°~90° | 0°~90° | 半球形，完全一致 |
| 安装角度 | 前倾约 30° | 前倾 30° | 完全一致 |
| 最远测距 | 60m | 60m | 完全一致 |
| 盲区 | <0.1m | 0.1m | 完全一致 |
| 帧率 | 10Hz | ~8.4Hz | 受 CPU 算力限制 |

> 仿真点云密度约为真实雷达的 6.7%，用于算法验证完全够用。

---

## 仿真场景

### AWS RoboMaker Warehouse（推荐）

仓库走廊场景，特征点丰富，适合 SLAM 建图验证。

```bash
# 下载场景（放在工作空间外，避免 colcon 编译报错）
git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git ~/aws-robomaker-small-warehouse-world

# 设置模型路径（永久生效）
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/aws-robomaker-small-warehouse-world/models' >> ~/.bashrc
source ~/.bashrc
```

在 `gazebo.launch.py` 顶部修改 `WORLD_FILE_PATH`：

```python
# 有屋顶版本
WORLD_FILE_PATH = '/home/robot/aws-robomaker-small-warehouse-world/worlds/small_warehouse.world'
# 无屋顶版本（更适合建图）
WORLD_FILE_PATH = '/home/robot/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse.world'
```

---

## 环境依赖

- Ubuntu 22.04
- ROS2 Humble

### 一键安装依赖

```bash
sudo apt update && sudo apt install \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui
```

| 包名 | 用途 |
|------|------|
| `ros-humble-xacro` | 解析 URDF/XACRO 机器人描述文件 |
| `ros-humble-gazebo-ros-pkgs` | Gazebo 与 ROS2 集成 |
| `ros-humble-gazebo-plugins` | Gazebo 传感器和驱动插件 |
| `ros-humble-robot-state-publisher` | 发布机器人 TF 坐标变换 |
| `ros-humble-joint-state-publisher` | 发布关节状态 |
| `ros-humble-joint-state-publisher-gui` | 关节状态图形调试界面 |

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

# 4. 启动 Gazebo 仿真
ros2 launch aicar_simulation gazebo.launch.py

# 5. 启动 RViz 模型预览（可选）
ros2 launch aicar_simulation display.launch.py
```

> 建议将以下内容添加到 `~/.bashrc`，避免每次手动执行：
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ai_car/install/setup.bash
> ```

---

## Launch 文件配置

`gazebo.launch.py` 顶部集中了所有常用配置项，修改这里即可：

```python
PACKAGE_NAME    = 'aicar_simulation'           # 功能包名称
XACRO_FILE_NAME = 'jkgn_car_96.urdf.xacro'    # URDF 文件名
WORLD_FILE_PATH = '/path/to/your.world'         # Gazebo 世界文件路径
USE_SIM_TIME    = True                          # 是否使用仿真时间
VERBOSE         = False                         # Gazebo 是否输出详细日志
SPAWN_POSITION  = (0.0, 0.0, 0.1)              # 机器人初始生成位置（z 轴稍微抬高避免穿模）
```

---

## 话题列表

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 3D 激光雷达点云 |
| `/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 深度点云 |

---

## 常见问题

**1. Gazebo 启动后机器人没有生成**

大场景加载时间较长，spawn_entity 可能超时，手动生成：
```bash
ros2 run gazebo_ros spawn_entity.py \
    -topic robot_description \
    -entity aicar \
    -x 0.0 -y 0.0 -z 0.1
```

**2. 点云频率低于 8Hz**

降低雷达采样数，在 URDF 中修改：
```xml
<horizontal>
    <samples>240</samples>
</horizontal>
<vertical>
    <samples>24</samples>
</vertical>
```

**3. 编译时报 catkin 找不到**

aws-robomaker-small-warehouse-world 是 ROS1 包，必须放在工作空间外：
```bash
mv ~/ai_car/src/aws-robomaker-small-warehouse-world ~/
rm -rf ~/ai_car/build ~/ai_car/install ~/ai_car/log
colcon build
```