"""
Node used for plotting turtlebot range measurements on a canvas
"""

# ROS libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from turtlebot3_utilites.sensor_plotter import SensorPlotter

# Forcing the max range value to 3.5. In simulation, this is not an issue,
# as the max_range returned by the sensor is already correct. However, on
# the physical Turtlebot3 with LD02, the max_range value is incorrect,
# which results in  bad display scaling, among other problems.
# According to specs, max range is:
# 3.5 for LDS01 (simulation, and older turtlebot3s)
# 8 for LDS02 (newer turtlebot3s)
MAX_RANGE = 3.5

# pylint: disable=C0116
class PlotNode(Node):
    """ Sensor plotter node """
    def __init__(self):
        super().__init__("slam_node")

        self.get_logger().info("Sensor plotting node has been started")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.scan_subscriber_ = self.create_subscription(
            msg_type=LaserScan, topic="/scan", callback=self.scan_callback, qos_profile=qos_profile)

        self.plotter = SensorPlotter()

    def scan_callback(self, msg: LaserScan):
        self.plotter.draw_rays(msg.ranges,
                               maxrange=MAX_RANGE if MAX_RANGE is not None else msg.range_max)

def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    rclpy.spin(node)
    rclpy.shutdown()
