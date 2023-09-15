from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


turtles = ["Foxy", "Neotic", "Humble", "Iron"]
my_pkg = get_package_share_directory("turtlesim_plus_controller")


def modify_config_namespace(path: str, new_path: str, namespace: str):
    with open(path, 'r') as file:
        data = yaml.load(file, Loader=yaml.SafeLoader)

    new_data = {namespace: data}

    with open(new_path, 'w') as file:
        yaml.dump(new_data, file)


def render_namespace(context: LaunchContext, launch_description:LaunchDescription):
    pass


def spawn_turtle(context: LaunchContext, launch_description:LaunchDescription):
    for turtle in turtles:
        spawn_turtle = ExecuteProcess(
            cmd=["ros2 service call /spawn_turtle turtlesim/srv/Spawn \"{x: 0.1, y: 0.1, theta: 0.0, name: '" + turtle + "'}\""],
            shell=True,
        )
        launch_description.add_action(spawn_turtle)


def create_controller(context: LaunchContext, launch_description:LaunchDescription):
    for turtle in turtles:
        controller = Node(
            package="turtlesim_plus_controller",
            executable="turtle_controller.py",
            namespace=turtle,
            parameters=[],  # TODO: implement config file
        )
        launch_description.add_action(controller)


def generate_launch_description():
    launch_description = LaunchDescription()

    turtlesim_plus = Node(
        package="turtlesim_plus",
        executable="turtlesim_plus_node.py",
    )

    remove_turtle1 = ExecuteProcess(
        cmd=["ros2 service call /remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
        shell=True,
    )


    launch_description.add_action(turtlesim_plus)
    launch_description.add_action(remove_turtle1)
    launch_description.add_action(OpaqueFunction(function=spawn_turtle, args=[launch_description]))
    launch_description.add_action(OpaqueFunction(function=create_controller, args=[launch_description]))
    return launch_description

# ros2 launch turtlesim_plus_controller turtlesimplus.launch.py
