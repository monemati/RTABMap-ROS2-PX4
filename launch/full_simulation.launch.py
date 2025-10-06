import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    px4_dir = '/root/PX4-Autopilot'
    mavsdk_script = '/root/PX4-ROS2-Gazebo-YOLOv8/keyboard-mavsdk-test.py'

    # Environment variables for PX4
    px4_env = {
        'PX4_GZ_WORLD': 'palace',
        'PX4_SYS_AUTOSTART': '4002',
        'PX4_GZ_MODEL_POSE': '20.0,0.0,3.86,0.00,0,3.14',
        'PX4_GZ_MODEL': 'x500_depth'
    }

    # Launch PX4 SITL
    px4_process = ExecuteProcess(
        cmd=[os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4')],
        cwd=px4_dir,  # Set working directory to PX4-Autopilot root
        env=px4_env,
        output='screen'
    )

    # Launch MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Launch the Python script for drone control
    drone_control_script = ExecuteProcess(
        cmd=['python3', mavsdk_script],
        output='screen'
    )

    # ROS-GZ bridges
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        output='screen'
    )

    image_bridge_camera = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera'],
        output='screen'
    )

    image_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/depth_camera'],
        output='screen'
    )

    # Static transform publishers
    static_tf_imx214 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'x500_depth_0/OakD-Lite/base_link',
                   '--child-frame-id', 'x500_depth_0/OakD-Lite/base_link/IMX214'],
        output='screen'
    )

    static_tf_stereo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'x500_depth_0/OakD-Lite/base_link',
                   '--child-frame-id', 'x500_depth_0/OakD-Lite/base_link/StereoOV7251'],
        output='screen'
    )

    # Include RTAB-MAP launch
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch_dir, '/launch/rtabmap.launch.py']),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgbd': 'true',
            'rgb_topic': '/camera',
            'depth_topic': '/depth_camera',
            'camera_info_topic': '/camera_info',
            'frame_id': 'x500_depth_0/OakD-Lite/base_link',
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        #px4_process,
        micro_xrce_agent,
        parameter_bridge,
        image_bridge_camera,
        image_bridge_depth,
        drone_control_script,
        static_tf_imx214,
        static_tf_stereo,
        rtabmap_include
    ])
