#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from turtlesim_plus_controller_interface.srv import SetTarget
import math


class TurtleScheduler(Node):
    def __init__(self):
        super().__init__('turtle_scheduler')
        


def main(args=None):
    rclpy.init(args=args)
    node = TurtleScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

# running this node with namespace
# ros2 run turtlesim_plus_controller turtle_controller.py --ros-args -r __ns:=/turtle1