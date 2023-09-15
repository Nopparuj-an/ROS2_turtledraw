#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import time
from turtlesim_plus_controller_interface.srv import SetTarget


class StatusChecker(Node):
    def __init__(self):
        super().__init__('status_checker')
        self.timer = self.create_timer(0.5, self.check_node_status)
        self.active = False
        self.start_time = 0
        self.phase = 0  # 0 idle, 1 arm, 2 run toward top right


    def all_go_top_right(self):
        turtles = ["Foxy", "Neotic", "Humble", "Iron"]
        for turtle in turtles:
            self.set_target_client = self.create_client(SetTarget, f"/{turtle}/go")
            request = SetTarget.Request()
            request.target.x = 15.0
            request.target.y = 15.0
            self.set_target_client.call_async(request)


    def check_node_status(self):
        node_names = self.get_node_names()
        scheduler_running = "turtle_scheduler" in node_names
        if scheduler_running and self.active == False:
            self.active = True

        if self.active and not scheduler_running:
            if self.phase == 0:
                self.start_time = time.time()
                self.phase = 1
            if self.phase == 1:
                if time.time() - self.start_time > 5.0:
                    self.phase = 2
                    self.all_go_top_right()
            if self.phase == 2:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = StatusChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()