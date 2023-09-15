#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from turtlesim_plus_controller_interface.srv import SetTarget
from ament_index_python.packages import get_package_share_directory
import math
import time
import yaml, argparse, os

my_pkg = get_package_share_directory("turtlesim_plus_controller")

class TurtleScheduler(Node):
    def __init__(self, file_path):
        time.sleep(1)
        super().__init__('turtle_scheduler')
        self.set_target_client = self.create_client(SetTarget, "go_and_place")

        with open(os.path.join(my_pkg, "via_point", file_path), 'r') as file:
            self.data = yaml.load(file, Loader=yaml.SafeLoader)["via_point"]

        def go_and_place_svc(pos):
            try:
                request = SetTarget.Request()
                request.target.x = pos[0]
                request.target.y = pos[1]
                svc_call = self.set_target_client.call_async(request)
                rclpy.spin_until_future_complete(self, svc_call, timeout_sec=1.0)
                res = svc_call.result().result
                return res
            except Exception as e:
                if e is KeyboardInterrupt:
                    exit()
                return False

        for i in self.data:
            while(not go_and_place_svc(i)):
                pass

        exit()


def main(args=None):
    parser = argparse.ArgumentParser(description='schedule via points')
    parser.add_argument('-f', '--file', help='path to the YAML file of via points')
    parsed_args, remaining_args = parser.parse_known_args(args=args)

    rclpy.init(args=remaining_args)
    node = TurtleScheduler(parsed_args.file)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

# running this node with namespace
# ros2 run turtlesim_plus_controller turtle_scheduler.py --ros-args -r __ns:=/Foxy -f "via_point_F.yaml"