"""Publish odometry position and orientation to topics labelled by dofs."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

import transforms3d


class BodyResponsePublisher(Node):
    def __init__(self):
        super().__init__("body_response_publisher")

        self.get_logger().info("Starting node 'body_response_publisher'")

        # subscribe to odom and republish as maritime dofs:
        # surge, sway, heave, roll, pitch and yaw

        # Subscribe to topic 'odom'
        self.subscription = self.create_subscription(
            Odometry, f"/odom", self.handle_odom, 1
        )
        self.subscription

        # publishers for maritime dofs
        self.dof_topics = ["/surge", "/sway", "/heave", "/roll", "/pitch", "/yaw"]
        self.pubs = [
            self.create_publisher(Float64, topic, 10) for topic in self.dof_topics
        ]

    def handle_odom(self, msg):

        # self.get_logger().info("received odom")

        pos = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        q = [rot.w, rot.x, rot.y, rot.z]
        ai, aj, ak = transforms3d.euler.quat2euler(q)
        x = [pos.x, pos.y, pos.z, ai, aj, ak]

        for i, topic in enumerate(self.dof_topics):
            dof_msg = Float64()
            dof_msg.data = x[i]
            self.pubs[i].publish(dof_msg)


def main():
    rclpy.init()
    node = BodyResponsePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
