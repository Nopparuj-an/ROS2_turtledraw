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
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import os
import yaml


turtles = ["Foxy", "Neotic", "Humble", "Iron"]
character = ["via_point_F.yaml", "via_point_I.yaml", "via_point_B.yaml", "via_point_O.yaml"]
my_pkg = get_package_share_directory("turtlesim_plus_controller")


def combine_path(context, character):
    path = []
    for i in character:
        with open(os.path.join(my_pkg,'via_point',i), 'r') as file:
            data = yaml.load(file, Loader=yaml.SafeLoader)["via_point"]
            path = path + data
    
    path += [[10.0, 10.0]]

    with open(os.path.join(my_pkg,'via_point','combined_path.yaml'), 'w') as file:
        yaml.dump({"via_point": path}, file)


def modify_config_namespace(path: str, new_path: str, namespace: str):
    with open(path, 'r') as file:
        data = yaml.load(file, Loader=yaml.SafeLoader)

    new_data = {namespace: data}

    with open(new_path, 'w') as file:
        yaml.dump(new_data, file)


def render_namespace(context: LaunchContext,launch_description: LaunchDescription):
    pass

    
def spawn_turtle(context: LaunchContext, launch_description:LaunchDescription):
    for turtle in turtles:
        spawn_turtle = ExecuteProcess(
            cmd=["ros2 service call /spawn_turtle turtlesim/srv/Spawn \"{x: 0.1, y: 0.1, theta: 0.0, name: '" + turtle + "'}\""],
            shell=True,
        )
        launch_description.add_action(spawn_turtle)


def create_controller(context: LaunchContext, launch_description:LaunchDescription,namespace: LaunchConfiguration):
    config_file_string = context.perform_substitution(namespace)
    config_path = os.path.join(my_pkg,'config',config_file_string)
    for turtle in turtles:
        new_config_path = os.path.join(my_pkg,'config','controller_config_'+turtle+'.yaml')
        modify_config_namespace(config_path,new_config_path,turtle)

        controller = Node(
            package="turtlesim_plus_controller",
            executable="turtle_controller.py",
            namespace=turtle,
            parameters=[new_config_path],
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
    
    config_launch_arg = DeclareLaunchArgument('config_path',default_value='controller_config_1.yaml')
    config_path = LaunchConfiguration('config_path')

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

    exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_plus,
            on_exit=[
                LogInfo(msg='closed the turtlesim window'),
                EmitEvent(event=Shutdown(reason='Window closed'))
            ]
        )
    )

    melodic_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=status_checker,
            on_exit=[
                LogInfo(msg='Melodic is unleashed!'),
                ExecuteProcess(cmd=["ros2 service call /spawn_turtle turtlesim/srv/Spawn \"{x: 0.1, y: 0.1, theta: 0.0, name: 'Melodic'}\""], shell=True),
                ExecuteProcess(cmd=["ros2 run turtlesim_plus_controller turtle_controller.py --ros-args -r __ns:=/Melodic"], shell=True),
                ExecuteProcess(cmd=["ros2 run turtlesim_plus_controller turtle_scheduler.py --ros-args -r __ns:=/Melodic -f \"combined_path.yaml\""], shell=True),
            ]
        )
    )

    launch_description.add_action(config_launch_arg)
    launch_description.add_action(path_generator)
    launch_description.add_action(OpaqueFunction(function=combine_path, args=[character]))
    launch_description.add_action(turtlesim_plus)
    launch_description.add_action(exit_event_handler)
    launch_description.add_action(remove_turtle1)

    launch_description.add_action(OpaqueFunction(function=spawn_turtle, args=[launch_description]))
    launch_description.add_action(OpaqueFunction(function=create_controller, args=[launch_description,config_path]))
    launch_description.add_action(OpaqueFunction(function=create_scheduler, args=[launch_description]))
    launch_description.add_action(status_checker)
    launch_description.add_action(melodic_event_handler)
    return launch_description

# ros2 launch turtlesim_plus_controller turtlesimplus.launch.py
