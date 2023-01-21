"""Launch Gazebo world with a RM3 WEC."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    model_name = "rm3_wec"

    # Bridge to forward tf and joint states to ros2
    # LaunchConfiguration("world_name"),
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/waves/model/rm3_wec/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/rm3_wec/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/rm3_wec/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/rm3_wec/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # "/force/gravity@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/buoyancy@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/restoring@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/radiation_damping@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/radiation_added_mass@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/excitation@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/excitation_froude_krylov@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/excitation_scattering@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/damping@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/viscous_drag@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/pressure_drag@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
            # "/force/froude_krylov@geometry_msgs/msg/Wrench[gz.msgs.Wrench",
        ],
        remappings=[
            (
                "/world/waves/model/rm3_wec/joint_state",
                "joint_states",
            ),
            ("/model/rm3_wec/pose", "/tf"),
            ("/model/rm3_wec/pose_static", "/tf_static"),
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
    # body_response_publisher = Node(
    #     package="gz_waves_bridge",
    #     executable="body_response_publisher",
    #     arguments=[],
    #     remappings=[
    #         ("/odom", "/model/rm3_wec/odometry"),
    #         ("/surge", "/model/" + model_name + "/surge"),
    #         ("/sway", "/model/" + model_name + "/sway"),
    #         ("/heave", "/model/" + model_name + "/heave"),
    #         ("/roll", "/model/" + model_name + "/roll"),
    #         ("/pitch", "/model/" + model_name + "/pitch"),
    #         ("/yaw", "/model/" + model_name + "/yaw"),
    #     ],
    #     parameters=[],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            bridge,
            # body_response_publisher,
        ]
    )
