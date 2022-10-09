"""Launch Gazebo world with an ellipsoid buoy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    # pkg_gz_waves_bridge = get_package_share_directory("gz_waves_bridge")
    # model_dir = "ellipsoid_buoy"
    model_name = "ellipsoid_buoy"

    # sdf_file = os.path.join(pkg_gz_waves_bridge, "models", model_dir, "model.sdf")
    # with open(sdf_file, "r") as infp:
    #     robot_desc = infp.read()

    # assumes we have generated a urdf file
    # urdf_file = os.path.join(pkg_gz_waves_bridge, "models", model_dir, "model.sdf.urdf")
    # with open(urdf_file, "r") as infp:
    #     robot_desc = infp.read()

    # gazebo_world_file_launch_arg = DeclareLaunchArgument(
    #     "world_file",
    #     default_value=["waves.sdf"],
    #     description="Gazebo world filename.sdf",
    # )

    # gazebo_world_name_launch_arg = DeclareLaunchArgument(
    #     "world_name", default_value=["world_demo"], description="Gazebo <world name>"
    # )

    # rviz_launch_arg = DeclareLaunchArgument(
    #     "rviz", default_value="false", description="Open RViz."
    # )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
    #     ),
    #     launch_arguments={"gz_args": "-v4 -s waves.sdf"}.items(),
    # )

    # Bridge to forward tf and joint states to ros2
    # LaunchConfiguration("world_name"),
    joint_state_gz_topic = (
        "/world/" + "world_demo" + "/model/" + model_name + "/joint_state"
    )
    odom_gz_topic = "/model/" + model_name + "/odometry"
    link_pose_gz_topic = "/model/" + model_name + "/pose"
    link_pose_static_gz_topic = "/model/" + model_name + "/pose_static"
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            joint_state_gz_topic + "@sensor_msgs/msg/JointState[gz.msgs.Model",
            odom_gz_topic + "@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            link_pose_gz_topic + "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            link_pose_static_gz_topic + "_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/force/gravity@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/buoyancy@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/restoring@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/radiation_damping@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/radiation_added_mass@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/excitation@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/excitation_froude_krylov@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            "/force/excitation_scattering@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
        ],
        remappings=[
            (joint_state_gz_topic, "joint_states"),
            (link_pose_gz_topic, "/tf"),
            (link_pose_static_gz_topic, "/tf_static"),
        ],
        parameters=[
            {"qos_overrides./tf_static.publisher.durability": "transient_local"}
        ],
        output="screen",
    )

    # body_response_publisher republishes the buoy odometry to maritime dof
    # topics: surge, sway, heave, roll, pitch, yaw
    # mainly for plotting - where plotting tools do not have built-in
    # converters from quaternions to euler angles.
    # 
    body_response_publisher = Node(
        package="gz_waves_bridge",
        executable="body_response_publisher",
        arguments=[],
        remappings=[
            ("/odom", odom_gz_topic),
            ("/surge", "/model/" + model_name + "/surge"),
            ("/sway", "/model/" + model_name + "/sway"),
            ("/heave", "/model/" + model_name + "/heave"),
            ("/roll", "/model/" + model_name + "/roll"),
            ("/pitch", "/model/" + model_name + "/pitch"),
            ("/yaw", "/model/" + model_name + "/yaw"),
        ],
        parameters=[],
        output="screen",
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="both",
    #     parameters=[
    #         {"use_sim_time": True},
    #         {"robot_description": robot_desc},
    #     ],
    # )

    # Launch rviz
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=[
    #         "-d",
    #         os.path.join(pkg_buoy_gazebo, "rviz", "ellipsoid_buoy.rviz"),
    #     ],
    #     condition=IfCondition(LaunchConfiguration("rviz")),
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    # )

    return LaunchDescription(
        [
            # gazebo_world_file_launch_arg,
            # gazebo_world_name_launch_arg,
            # rviz_launch_arg,
            # gazebo,
            bridge,
            body_response_publisher,
            # robot_state_publisher,
            # rviz,
        ]
    )
