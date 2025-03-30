import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import serial

class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.listener_callback,
            10)
        self.subscription
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)

    def listener_callback(self, msg):
        for pose_stamped in msg.poses:
            position_x = pose_stamped.pose.position.x
            position_y = pose_stamped.pose.position.y
            self.get_logger().info(f'position x: {position_x} , position y: {position_y}')
            self.ser.write(f'x:{position_x},y:{position_y}\n'.encode())

def main(args=None):
    rclpy.init(args=args)

    path_subscriber = PathSubscriber()

    rclpy.spin(path_subscriber)

    path_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()