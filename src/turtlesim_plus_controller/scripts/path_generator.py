#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np

class DummyNode(Node):
    def __init__(self):
        super().__init__('path_node')
        pkg_path = get_package_share_directory('turtlesim_plus_controller')

        # Define the paths for the four YAML files
        yaml_files = [
            ('via_point_F.yaml'),
            ('via_point_I.yaml'),
            ('via_point_B.yaml'),
            ('via_point_O.yaml')
        ]

        # Iterate through the YAML files and generate them
        for yaml_file in yaml_files:
            shape = yaml_file[10]
            via_point_path = os.path.join(pkg_path, 'via_point', yaml_file)
            self.generate_shape_yaml(via_point_path, shape)
        
        exit()

    def generate_shape_yaml(self, path: str, shape: str):
        # Define data for each shape
        data = {'via_point': []}
        if shape == 'F':
            data['via_point'] = [[0.0, 0.0], [0.0, 15.0]]
        elif shape == 'I':
            data['via_point'] = [[0.0, 0.0], [0.0, 10.0]]
        elif shape == 'B':
            data['via_point'] = [[0.0, 0.0], [0.0, 8.0]]
        elif shape == 'O':
            data['via_point'] = [[-6.0, 0.0], [-6.0, 0.0]]
        else:
            self.get_logger().error(f"Unsupported shape: {shape}")

        # Write the data to the YAML file
        with open(path, 'w') as file:
            yaml.dump(data, file)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()