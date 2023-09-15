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


my_pkg = get_package_share_directory("turtlesim_control")


def modify_config_namespace(path: str, new_path: str, namespace: str):
    with open(path, 'r') as file:
        data = yaml.load(file, Loader=yaml.SafeLoader)

    new_data = {namespace: data}

    with open(new_path, 'w') as file:
        yaml.dump(new_data, file)


def render_namespace(context: LaunchContext, launch_description:LaunchDescription):
    pass


def generate_launch_description():
    launch_description = LaunchDescription()

    return launch_description