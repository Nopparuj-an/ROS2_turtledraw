#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from turtlesim_plus_controller_interface.srv import SetTarget
import math
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.pub_cmdvel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Pose, "pose", self.pose_callback , 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "eat")
        self.create_timer(0.01, self.timer_callback)
        self.target = [0.0, 0.0, 0.0]
        self.current_pose = [0.0, 0.0, 0.0]
        self.enableController = False
        self.doSpawnPizza = False
        self.continuousEating = False
        self.create_service(SetTarget, "go_and_place", self.go_and_place_callback)
        self.create_service(SetTarget, "go", self.go_callback)
        self.declare_parameter(name='linear_gain',value=1.0)
        self.declare_parameter(name='angular_gain',value=5.0)
        self.declare_parameter(name='tolerance',value=0.1)

        if self.get_namespace() == "/Melodic":
            self.continuousEating = True
            self.set_parameters([Parameter('linear_gain', value=10.0),
                                  Parameter('angular_gain', value=30.0),
                                  Parameter('tolerance', value=0.1)])

    def go_and_place_callback(self, request, response):
        if self.enableController:
            response.result = False
        else:
            self.target = [request.target.x, request.target.y, 0]
            self.doSpawnPizza = True
            self.enableController = True
            response.result = True
        return response
    
    def go_callback(self, request, response):
        self.target = [request.target.x, request.target.y, 0]
        self.doSpawnPizza = False
        self.enableController = True
        response.result = True
        return response
    
    def spawn_pizza(self, position):
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def pose_callback(self, msg):
        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.theta

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmdvel.publish(cmd_vel)

    def wrap_angle(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def timer_callback(self):
        if self.continuousEating:
            self.eat_pizza_client.call_async(Empty.Request())

        if(not self.enableController):
            return
        
        # Calculate distance between current pose and target
        distance = math.sqrt((self.current_pose[0] - self.target[0])**2 + (self.current_pose[1] - self.target[1])**2)
        if(abs(distance) < self.get_parameter('tolerance').value):
            self.enableController = False
            self.cmd_vel(0.0, 0.0)
            if self.doSpawnPizza and not self.continuousEating:
                self.spawn_pizza(self.current_pose)
                self.doSpawnPizza = False
            return
        P_distance = distance * self.get_parameter('linear_gain').value  # Kp value (2)
        
        # Calculate angle between current pose and target
        angle = math.atan2(-1*self.current_pose[1] + self.target[1], -1*self.current_pose[0] + self.target[0])
        error_angle = self.wrap_angle(angle - self.current_pose[2])
        P_angle = error_angle * self.get_parameter('angular_gain').value  # Kp value (10)
        if(abs(error_angle) > math.pi):
            P_angle *= -1

        print(angle, self.current_pose[2], error_angle)
        self.cmd_vel(P_distance, P_angle)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

# running this node with namespace
# ros2 run turtlesim_plus_controller turtle_controller.py --ros-args -r __ns:=/turtle1

# ros2 service call /turtle1/go_and_place turtlesim_plus_controller_interface/srv/SetTarget "target:
#  x: 5.0
#  y: 5.0
#  z: 0.0" 
