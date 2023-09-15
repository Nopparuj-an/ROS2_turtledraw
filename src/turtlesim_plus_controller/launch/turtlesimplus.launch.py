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
# from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.events import Shutdown
import os
import yaml


turtles = ["Foxy", "Neotic", "Humble", "Iron"]
character = ["via_point_F.yaml", "via_point_I.yaml", "via_point_B.yaml", "via_point_O.yaml"]
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


def create_scheduler(context: LaunchContext, launch_description:LaunchDescription):
    for i in range(len(turtles)):
        viapoint_path = os.path.join(my_pkg,'via_point',character[i])
        scheduler = Node(
            package="turtlesim_plus_controller",
            executable="turtle_scheduler.py",
            namespace=turtles[i],
            arguments=['-f',viapoint_path]
        )
        launch_description.add_action(scheduler)
        
            

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

    path_generator = Node(
        package="turtlesim_plus_controller",
        executable="path_generator.py",
    )

    status_checker = Node(
        package="turtlesim_plus_controller",
        executable="status_checker.py",
    )

    # exit_event_handler = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=turtlesim_plus,
    #         on_exit=[
    #             LogInfo(msg='closed the turtlesim window'),
    #             EmitEvent(event=Shutdown(reason='Window closed'))
    #         ]
    #     )
    # )

    launch_description.add_action(path_generator)
    launch_description.add_action(turtlesim_plus)
    # launch_description.add_action(exit_event_handler)
    launch_description.add_action(remove_turtle1)
    launch_description.add_action(OpaqueFunction(function=spawn_turtle, args=[launch_description]))

    launch_description.add_action(OpaqueFunction(function=create_controller, args=[launch_description]))
    launch_description.add_action(OpaqueFunction(function=create_scheduler, args=[launch_description]))
    launch_description.add_action(status_checker)
    return launch_description

# ros2 launch turtlesim_plus_controller turtlesimplus.launch.py
