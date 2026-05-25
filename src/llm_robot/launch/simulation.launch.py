from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

# Home pozisyona getir (8 saniye bekle, simülasyon yüklensin)
set_home_position = TimerAction(
    period=8.0,
    actions=[
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                '/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                '{"joint_names": ["arm_0_shoulder_pan_joint", "arm_0_shoulder_lift_joint", "arm_0_elbow_joint", "arm_0_wrist_1_joint", "arm_0_wrist_2_joint", "arm_0_wrist_3_joint"], "points": [{"positions": [0.0, -1.57, 0.0, -1.57, -1.57, 0.0], "time_from_start": {"sec": 3}}]}'],
            output='screen'
        )
    ]
)

# Nodelar 10 saniye sonra başlasın (simülasyon + home pozisyon tamamlansın)
camera_node = TimerAction(
    period=10.0,
    actions=[Node(
        package="llm_robot",
        executable="camera",
        name="camera_viewer",
        output="screen"
    )]
)

detector_node = TimerAction(
    period=10.0,
    actions=[Node(
        package="llm_robot",
        executable="detector",
        name="detector",
        output="screen"
    )]
)

scanner_node = TimerAction(
    period=10.0,
    actions=[Node(
        package="llm_robot",
        executable="scanner",
        name="scanner",
        output="screen"
    )]
)

navigator_node = TimerAction(
    period=10.0,
    actions=[Node(
        package="llm_robot",
        executable="navigator",
        name="navigator",
        output="screen"
    )]
)

arm_controll_node = TimerAction(
    period=11.0,
    actions=[Node(
        package="llm_robot",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        remappings=[
            ('/tf', '/a200_0000/tf'),
            ('/tf_static', '/a200_0000/tf_static'),
        ]
    )]
)

def generate_launch_description():
    world = "my_world"

    clearpath_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('clearpath_gz'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        launch_arguments={'world': world}.items()
    )

    return LaunchDescription([
        clearpath_sim,
        set_home_position,
        camera_node,
        detector_node,
        scanner_node,
        navigator_node,
        arm_controll_node
    ])