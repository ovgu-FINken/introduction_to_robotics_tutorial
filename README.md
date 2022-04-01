# Tutorial Workspace for Introduction to Robotics

In this workpsace we will create packages for the programming assignments.
You can start working on the first assignment immediatly.

## Assignment 1: Turtlesim

We will start by using the turtlesim simulation. You can find a lot of documentation for the turtlesim online, make sure that you read documentation for ros2 (version foxy-fitzroy).
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
- `ros2 run turtlesim turtlesim_teleop_key` starts the teleoperation node - this lets you steer the turtle
- Now you can inspect the current behaviour of the system with `rqt`
    - We suggest two plugins that you can enable: *introspection - node graph* and *topics - topic monitor*
- You can see that the teleop node directly steers the turtle - this is not what we want
    - You can redirect the output of the teleop node to the input topic for the watchdog node topic: `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle1/input_cmd`
    - Check the changes in the node graph (the teleop node should now send its messages to the topic 'turtle1/input_cmd' which is subscribed by the watchdog)
    - Now the turtle should move backwards
- When the behaviour of the turtle is right, pay attention to the controller node
    - This node publishes a start and a stop message
    - Modify the behaviour of the watchdog, so the turtle is only able to turn before the start command is sent and stopped completly after the stop command is sent


