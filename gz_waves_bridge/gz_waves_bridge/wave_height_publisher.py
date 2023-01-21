#!/usr/bin/env python

# Copyright (C) 2023 Rhys Mainwaring
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish wave height measured by a WEC device."""

import time

# ros2
import builtin_interfaces
import rclpy
from rclpy.node import Node

# ros2 msgs
from geometry_msgs.msg import PoseStamped

# gz msgs
from gz.msgs.clock_pb2 import Clock
from gz.msgs.pose_v_pb2 import Pose_V

# gz transport
from gz.transport import SubscribeOptions
from gz.transport import Node as GzNode

# constants
SUB_LINK = "rm3_wec::float_link"
SUB_TOPIC = "/model/rm3_wec/pose"
PUB_TOPIC = "/wave_height"
OUT_FILE = "wave_height.csv"

class WaveHeightPublisher(Node):
    def __init__(self):
        super().__init__("wave_height_publisher")

        self.get_logger().info("Starting node 'wave_height_publisher'")

        # create a gz.transport node
        self.gz_node = GzNode()

        # throttle
        self.last_sim_time = 0
        self.pub_period = 1.0/10.0

        # publishers
        self.publisher = self.create_publisher(PoseStamped, PUB_TOPIC, 10)

        # subscribers
        def sub_to_pose():
            topic = SUB_TOPIC
            msg_type_name = Pose_V.DESCRIPTOR.full_name
            sub_options = SubscribeOptions()

            if self.gz_node.subscribe(topic, self.pose_cb, msg_type_name, sub_options):
                print(
                    "Subscribing to type {} on topic [{}]".format(msg_type_name, topic)
                )
            else:
                print("Error subscribing to topic [{}]".format(topic))

        sub_to_pose()

        # file
        self.num_samples = 512
        self.times = []
        self.positions = []
        self.did_write = False



    def pose_cb(self, msg: Pose_V) -> None:
        # throttle
        sec = msg.pose[0].header.stamp.sec
        nsec = msg.pose[0].header.stamp.nsec
        sim_time = sec + nsec * 1.0e-9
        if sim_time - self.last_sim_time < self.pub_period:
            return

        self.last_sim_time = sim_time

        # filter on link
        name = SUB_LINK
        for pose in msg.pose:
            if pose.name == name:
                sec = pose.header.stamp.sec
                nsec = pose.header.stamp.nsec
                sim_time = sec + nsec * 1.0e-9
                pos = pose.position
                frame_id = pose.header.data[0].value[0]
                print("time:     {}".format(sim_time))
                print("name:     {}".format(pose.name))
                print("pos.z:    {}".format(pos.z))
                print("frame_id: {}".format(frame_id))

                # create timestamp
                stamp = builtin_interfaces.msg.Time(sec=sec, nanosec=nsec)

                # publish to ros2
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = frame_id
                pose_msg.pose.position.z = pos.z
                self.publisher.publish(pose_msg)

                # collect samples and write when complete
                if len(self.times) < self.num_samples:
                    self.times.append(sim_time)
                    self.positions.append(pos.z)
                elif not self.did_write:
                    # write to file
                    with open(OUT_FILE, "w", encoding="utf-8") as f:
                        for t, x in zip(self.times, self.positions):
                            f.write("{} {}\n".format(t, x))
                    self.did_write = True


def main():
    rclpy.init()
    node = WaveHeightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
