# FRA501_exam1_6434_6438
"turtlesim_plus_controller" is a ROS2 custom package that launches a turtlesim_plus node with 4 turtles, 4 controller and scheduler nodes for each turtle. Each turtle has a role to draw one of the letters of the word "FIBO" using 20 pizzas each.

<br>

## Installation
1. Clone the repository and move `turtlesim_plus_controller` and `turtlesim_plus_controller_interface` to the `src` folder of your ROS2 workspace.
2. Build `turtlesim_plus_controller` and `turtlesim_plus_controller_interface`
```bash
colcon build --packages-select turtlesim_plus_controller turtlesim_plus_controller_interface
```
3. Source the workspace
```bash
source install/setup.bash
```

<br>

## Usage
Launch the `turtlesim_plus_controller` package
```bash
ros2 launch turtlesim_plus_controller turtlesimplus.launch.py
```
You can select between `controller_config_1.yaml` and `controller_config_2.yaml` to change the speed of the turtles. You can also specify the amount of pizza per character by adjusting `pizza_amt`
```bash
ros2 launch turtlesim_plus_controller turtlesimplus.launch.py config_path:="controller_config_2.yaml" pizza_amt:=20
```
**If the launch complains about missing folder on the first run, please try running again.**

<br>

## System Architecture
<p align="center">
  <img src="https://github.com/Nopparuj-an/FRA501_exam1_6434_6438/blob/main/system_architecture.jpg?raw=true">
</p>

<br>

## Packages and nodes

### turtlesim_plus
- **turtlesim_plus_node.py**: A modified version of the turtlesim node by AJ.Pi.

### turtlesim_plus_interface
- A package that provides a custom service for turtlesim_plus_node.

### turtlesim_plus_controller
- **path_generator.py**: A node that generates YAML files for each turtle to follow.
- **turtle_controller.py**: A node that controls the turtlesim_plus node to reach target and drop a pizza.
- **turtle_scheduler.py**: The node that controls the turtlesim_plus node by sending targets to draw the word "FIBO".
- **status_checker.py**: A python script that checks the status of the turtlesim_plus node and move all turtle to top right on completion. It also trigger the launch file to spawn Melodic, the destructor turtle.

### turtlesim_plus_controller_interface
- The node that provides a custom service so the controller and scheduler can communicate.

<br>

## Nodes interaction

- **turtlesim_plus_node.py**
    - Subscribes to `/<namespace>/cmd_vel` to move the turtle.
    - Publishes to `/<namespace>/pose` to get the turtle's position.
    - Svc server to `/spawn_pizza` to drop a pizza.
    - Svc server to `/<namespace>/eat` to let Melodic eat pizza.
- **path_generator.py** do not interact with any other node. Is called by the launch file.
    - Receive parameters `pizza_amt` from the launch file for pizza per character (Defaults to 20).
- **turtle_controller.py**
    - Publishes to `/<namespace>/cmd_vel` to move the turtle.
    - Subscribes to `/<namespace>/pose` to get the turtle's position.
    - Svc client to `/<namespace>/spawn_pizza` to drop a pizza.
    - Svc client to `/<namespace>/eat` to let Melodic eat pizza.
    - Svc server to `/<namespace>/go` to get target location.
    - Svc server to `/<namespace>/go_and_place` to get target location to drop a pizza.
- **turtle_scheduler.py**
    - Receive parameters `config_path` from the launch file to get the YAML file path.
    - Svc client to `/<namespace>/go_and_place` to set target location to drop a pizza.
    - Parameter `-f` to get viapoints from the YAML file path.
- **status_checker.py**
    - Svc client to `/<namespace>/go` to set target location.
    - `get_node_names()` to scan for running scheduler nodes.
