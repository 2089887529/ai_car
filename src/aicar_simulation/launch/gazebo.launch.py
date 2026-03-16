from launch import LaunchDescription
from launch_ros.actions import Node
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
import xacro
from launch.actions import RegisterEventHandler,ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction

def generate_launch_description():
    # 基础配置
    PACKAGE_NAME = 'aicar_simulation'           #功能包名称
    XACRO_FILE_NAME = 'akermann_car_16.urdf.xacro'   #urdf文件名称，位于功能包的urdf目录下
    WORLD_FILE_PATH = '/home/robot/aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse.world'  #gazebo 世界文件路径
    USE_SIM_TIME = True  #是否使用仿真时间
    VERBOSE = False       #Gazebo是否输出详细日志
    SPAWN_POSITION = (0.0, 0.0, 0.1)  #机器人初始位置，z轴稍微抬高以避免穿模

    # 获取路径并解析xacro
    pkg_path = get_package_share_directory(PACKAGE_NAME)
    xacro_path = os.path.join(pkg_path, 'urdf', XACRO_FILE_NAME)
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
            'use_sim_time': USE_SIM_TIME 
        }]
    )

    # 启动 Gazebo 官方基础 Launch
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    #     ]),
    #     launch_arguments={
    #         'world': WORLD_FILE_PATH,
    #         'verbose': str(VERBOSE).lower()
    #     }.items()
    # )
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', 
            *(['--verbose'] if VERBOSE else []),
            '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so', 
            WORLD_FILE_PATH
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )


    # 生成机器人的节点
    x, y, z = SPAWN_POSITION
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic','robot_description', 
            '-entity', 'aicar', 
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ],
        output='screen'
    )

    # Gazebo 启动后在等待5秒生成机器人
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver,
            on_start=[
                TimerAction(
                    period=1.0,         # Gazebo 启动后在等待1秒生成机器人
                    actions=[spawn_entity]
                )
            ]
        )
    )
    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_after_gazebo)
    return ld