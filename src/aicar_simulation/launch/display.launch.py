from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1.定位包的路径
    pkg_share = get_package_share_directory("aicar_simulation")

    # 2.定位到xacro文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'aicar.urdf.xacro')
    
    # RViz 配置文件路径
    rviz_config  = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # 3.核心逻辑: 使用 xacro 命令动态解析文件
    robot_description_str = Command(['xacro ', default_model_path])

    # 4.定义 robot_state_publisher 节点
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description':ParameterValue(robot_description_str, value_type=str)}]
    )

    # 5.定义 robot_state_publisher_gui 节点
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # 6.定义rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config],
        output="screen"
    )

    # 返回启动描述
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz_node)
    return ld