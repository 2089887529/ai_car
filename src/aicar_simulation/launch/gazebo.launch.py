from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 基础配置
    package_name = 'aicar_simulation'
    xacro_file = 'aicar.urdf.xacro'

    # 获取路径并解析xacro
    pkg_path = get_package_share_directory(package_name)
    xacro_path = os.path.join(pkg_path, 'urdf', xacro_file)
    # 使用 xacro 库解析文件, 将其转化为 urdf 字符串
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # 机器人状态发布节点(发布TF)
    # 这个节点是 ROS 2 的标配，负责根据 URDF 和关节状态计算并发布完整的 TF 坐标变换
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True 
        }]
    )

    # 启动 Gazebo 官方基础 Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # 生成机器人的节点
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description', '-entity', 'aicar', '-z', '0.1'],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    return ld