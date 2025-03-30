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
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RobotPosition(Node):
    def __init__(self):
        super().__init__('robot_position')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'lidar_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except LookupException as e:
            self.get_logger().error('Failed to get robot position: {}'.format(e))
            return None

def main(args=None):
    rclpy.init(args=args)
    robot_position = RobotPosition()
    while rclpy.ok():
        position = robot_position.get_position()
    if position is not None:
        print('Robot position: x={}, y={}'.format(position[0], position[1]))
        rclpy.spin(robot_position, timeout_sec=1.0)
    robot_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()