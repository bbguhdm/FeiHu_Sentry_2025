#!/usr/bin/env python3
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        for pose_stamped in msg.poses:
            position_x = pose_stamped.pose.position.x
            position_y = pose_stamped.pose.position.y
            self.get_logger().info(f'position x: {position_x} , position y: {position_y}')

def main(args=None):
    rclpy.init(args=args)

    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()