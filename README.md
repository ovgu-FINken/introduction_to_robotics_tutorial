# Tutorial Workspace for Introduction to Robotics

In this workpsace we will create packages for the programming assignments.
You can start working on the first assignment immediatly.

## Assignment 1: Turtlesim

We will start by using the turtlesim simulation. You can find a lot of documentation for the turtlesim online, make sure that you read documentation for ros2 (version humble).
Your task in this assignment is to enhance a 'watchdog' node, which intecepts and changes the inputs given by the user.

The goal of this assignment is that you familiarize with the ros2 ecosystem:
- launch a collection of nodes
- start individual nodes
- visualize topics and nodes using `rqt`
- see what messages are sent using `rqt` and `ros2 topic`
- modify existing code and see the effect of your changes
- learn to understand errors in your own code

To start you can install the workspace using `git clone`.
Build the workspace with the command `colcon build` (from the new directory you just downloaded). Now you have to 'source' the workspace `source install/setup.bash` to use it. You can also add this to your .bashrc file, so you do not have to do this every time you open a new terminal.

Start the package for assignment one in the following way:
- `ros2 launch watchdog watchdog.launch.py` will start the turtlesim and the watchdog node
- `ros2 run turtlesim turtle_teleop_key` starts the teleoperation node - this lets you steer the turtle
    - Use the arrow keys to turn the turtle (the keys GVBR etc. use another mechanism that controls the turtle which bypasses the watchdog node)
- Now you can inspect the current behaviour of the system with `rqt`
    - We suggest two plugins that you can enable: *introspection - node graph* and *topics - topic monitor*
- You can see that the teleop node directly steers the turtle - this is not what we want
    - You can redirect the output of the teleop node to the input topic for the watchdog node topic: `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle1/input_cmd`
    - Check the changes in the node graph (the teleop node should now send its messages to the topic 'turtle1/input_cmd' which is subscribed by the watchdog)
    - Now the turtle should move backwards
- When the behaviour of the turtle is right, pay attention to the controller node
    - This node publishes a start and a stop message
    - Modify the behaviour of the watchdog, so the turtle is only able to turn before the start command is sent and stopped completly after the stop command is sent
    - In order to create the desired behaviour you have to set certain components of the 'cmd_vel' topic to zero

## Assignment 2: Reactive Behaviour

In this assignment we want to program a reactive behaviour, that controls the robot by directly computing actuator commands from the sensor input. The sensor input we use is the laser scanner of the robot, which publishes to the `scan` topic. The velocity of the robot is controlled via the `cmd_vel` topic.

- Make sure you use the most recent version of the assignment by using git (you can either use a new branch for assigment 2, or merge the upstream changes to your local copy of the workspace).
- The new code is in a ros-package called `reactive_behaviour`, use `colcon build` and start the behaviour with `ros2 launch reactive_behaviour robot.launch.py n_robots:=1`
- You can modify the code within the file `controller.py` to change the behaviour of the robot
- Explore as much of the reachable area as possible (think of a cleaning robot)
- Don't crash ;)

### Benchmark
Run your behaviour and note the score after 240s (sim-time)
- `ros2 launch reactive_behaviour robot.launch.py n_robots:=1`
- `ros2 launch reactive_behaviour swarmlab.launch.py n_robots:=1`

## Assignment 3: State Estimation

In this assignment your task is to estimate the state of the robot. To achieve this goal, you will follow a two step process:
- Compute the robot postition (x, y, z) in the node `locator.py`, based on range measurements (similar to GPS)
- Go to the position published in the topic `goal` by publishing the correct `cmd_vel`

You can start your code with the 'robot.launch.py' launch-file, just as you did in assignment 2.

- We will measure the time to complete the third goal
- As always: Don't crash

## Assignment 4: Planning

In this assignment, you will be resposible for planning the future path of the robots. For this planning task, you will get goal and position information for the robot, as well as a version of the map containing obstacle information.
As the state-space, we will use the pose information (x, y, angle) for the robot.

You can find out how the transition between two states is working, by creating a path between any pair of poses with a vehicle model. The robot is able to follow the path created by the vehicle model (with an unknown tracking error).

- Install the dependencies with `pip install git+https://github.com/ovgu-FINken/polygonal_roadmaps tqdm`, update the driving_swarm_infrastructure
- Start the behaviour using the `ros2 launch planning robot.launch.py`
- We also provide a jupyter-notebook to check your code outside of ros2 (a visualization can help to debug your code)


We suggest that you use the Probabilistic Roadmap as a planning approach, but you can also use different approaches if you like.

