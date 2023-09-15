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

        self.declare_parameter(name='pizza_amt',value=20)

        yaml_files = [
            ('via_point_F.yaml'),
            ('via_point_I.yaml'),
            ('via_point_B.yaml'),
            ('via_point_O.yaml')
        ]
        self.trajec = []
        pizza_amt = 1 * self.get_parameter('pizza_amt').value
        print(pizza_amt)
        print("-"*100)
        for yaml_file in yaml_files:
            shape = yaml_file[10]
            via_point_path = os.path.join(pkg_path, 'via_point', yaml_file)
            self.generate_shape_yaml(via_point_path, shape, int(pizza_amt))
        exit()

    def generate_shape_yaml(self, path: str, shape: str, pizza_amt: int):
        data = {'via_point': []}
        if shape == 'F':
            self.generate_letter_F_path(pizza_amt,2,7,1)
            data['via_point'] = self.trajec.tolist()

        elif shape == 'I':
            hint_target = [[0.0, 0.0], [0.0, 3.0]]
            segments = 1
            self.generate_letter_path(hint_target,segments, pizza_amt, 7, 7, 1)
            data['via_point'] = self.trajec.tolist()

        elif shape == 'B':
            hint_target = [[0.0, 0.0], [0.0, 3.0],[1.5, 2.25],[0, 1.5], [1.5, 0.75], [0.0,0.0]]
            segments = 5
            self.generate_letter_path(hint_target,segments, pizza_amt, 2, 2, 1)
            data['via_point'] = self.trajec.tolist()
            
        elif shape == 'O':
            hint_target = [[0.0, 0.0], [0.0, 3.0],[1.5, 3.0],[1.5, 0.0], [0.0, 0.0]]
            segments = 4
            self.generate_letter_path(hint_target,segments, pizza_amt, 7, 2, 1)
            data['via_point'] = self.trajec.tolist()

        else:
            self.get_logger().error(f"Unsupported shape: {shape}")

        # Write the data to the YAML file
        with open(path, 'w') as file:
            yaml.dump(data, file)

    def generate_letter_F_path(self, n, xoffset, yoffset, scaled):
        coordinates = [[0.0, 0.0], [0.0, 3.0], [2.0, 3.0], [2.0, 1.5], [0.0, 1.5]]
        point = n
        line = 3

        line1_x = (coordinates[0][0] - coordinates[1][0])
        line1_y = (coordinates[0][1] - coordinates[1][1])

        line2_x = (coordinates[1][0] - coordinates[2][0])
        line2_y = (coordinates[1][1] - coordinates[2][1])

        line3_x = (coordinates[3][0] - coordinates[4][0])
        line3_y = (coordinates[3][1] - coordinates[4][1])

        point_each_line = int(point / line)

        x = []
        y = []
        # Generate points for the first line segment with offsets and scale
        for i in range(point_each_line):
            x.append((coordinates[0][0] - i * (line1_x / point_each_line) + xoffset) * scaled)
            y.append((coordinates[0][1] - i * (line1_y / point_each_line) + yoffset) * scaled)

        # Generate points for the second line segment with offsets and scale
        for i in range(point_each_line):
            x.append((coordinates[1][0] - i * (line2_x / point_each_line) + xoffset) * scaled)
            y.append((coordinates[1][1] - i * (line2_y / point_each_line) + yoffset) * scaled)

        # Generate points for the third line segment with offsets and scale
        for i in range(point_each_line):
            x.append((coordinates[3][0] - i * (line3_x / point_each_line) + xoffset) * scaled)
            y.append((coordinates[3][1] - i * (line3_y / point_each_line) + yoffset) * scaled)

        last_x = x[-1]
        last_y = y[-1]

        loss = point%line
        print(point%line)
        print("-"*100)
        for i in range(loss):
            x.append(last_x)
            y.append(last_y) 

        self.trajec = np.column_stack((x, y))


    def generate_letter_path(self,coordinates,num_segments,n, xoffset, yoffset, scaled):
        total_points = n

        x = []
        y = []

        for i in range(num_segments):
            # Calculate the number of points for this line segment
            points_in_segment = total_points // (num_segments - i)
            
            # coordinates for the current segment
            x1, y1 = coordinates[i]
            x2, y2 = coordinates[i + 1]

            # Generate points for the current segment
            for j in range(points_in_segment):
                t = j / (points_in_segment - 1)  # Interpolation parameter [0, 1]
                x_point = x1 + (x2 - x1) * t
                y_point = y1 + (y2 - y1) * t
                x.append((x_point + xoffset) * scaled)
                y.append((y_point + yoffset) * scaled)

            # Update the remaining number of points
            total_points -= points_in_segment

        self.trajec = np.column_stack((x, y))


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()