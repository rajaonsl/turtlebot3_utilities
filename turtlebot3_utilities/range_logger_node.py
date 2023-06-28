"""
ROS2 Node used for logging turtlebot range measurements in a file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


PATH = '/home/ubuntu/share/ranges_long.txt'


# pylint: disable=C0116
class RangeLogger(Node):
    """ Range logger node """
    def __init__(self):
        super().__init__('range_logger')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.file = None

    def scan_callback(self, msg):
        if self.file is not None:
            for dist in msg.ranges:
                self.file.write(str(dist) + ' ')
            self.file.write('\n\n')
            # input("Press Enter to log the next set of ranges or Ctrl-C to stop....")

    def run(self):
        # input("Press Enter to start logging ranges...")
        self.file = open(PATH, 'w', encoding='utf-8')
        while True:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                break
        print("Stopping logger...")
        self.file.close()

    def __del__(self):
        if self.file is not None:
            self.file.close()


def main(args=None):
    rclpy.init(args=args)
    range_logger = RangeLogger()
    range_logger.run()
    range_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
