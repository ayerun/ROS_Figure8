# Turtle Trajectories Package
Author: Arun Kumar
This ROS package inputs parameters describing a figure eight trajectory. Using the inputs, the homework2 python package calculates control commands for a TurtleBot to execute the figure eight trajectory. Nodes within the package publish these commands to the TurtleBot. Trajectory and simulations of the Turtlebot are available to Gazebo and rviz.

### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile with `<catkin_make>`
 1. download any ROS packages that are necessary dependencies
1. Add the TurtleBot Burger model to your ROS environment variables
```
export TURTLEBOT3_MODEL=burger
```
1. Configure figure eight trajectory parameters in config/trajectory.yaml
1. Start simulation
```
roslaunch turtle_trajectories figure_eight.launch
```
    1. The figure_eight launch file contains three arguments, gaz, rviz, and plot, which correspond to Gazebo, rviz, and rqt_plot
    1. When these arguments are true the corresponding packages are launched
    1. By default all three arguments are true
    1. Set arguements false either via the command line or via the launch file.
```
roslaunch turtle_trajectories figure_eight.launch plot:=false
```
1. Turtle starts in a paused state
1. Call the /resume service to make turtle execute trajectory
```
rosservice call /resume
```
 1. Turtle can be paused at any time by calling the pause service

### Testing:
Run `<catkin_make run_tests>` in root directory of workspace to test python package, homework2

### Videos: