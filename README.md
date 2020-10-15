# Turtle Trajectories Package
### Author: Arun Kumar

### Turtlebot Figure Eight

#### This portion of the package inputs parameters describing a figure eight trajectory. Using the inputs, the homework2 python package calculates control commands for a TurtleBot to execute the figure eight trajectory. Nodes within the package publish these commands to the TurtleBot. Trajectory and simulations of the Turtlebot are available to Gazebo and rviz.

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile: `catkin_make`
1. Add the TurtleBot Burger model to your ROS environment variables: `export TURTLEBOT3_MODEL=burger`
1. Start simulation: `roslaunch turtle_trajectories figure_eight.launch`
1. Turtle starts in a paused state
1. Call the resume service to make turtle execute trajectory: `rosservice call /resume`
1. Turtle can be paused at any time by calling the pause service: `rosservice call /pause`

#### Configuration Instructions:
1. Configure figure eight trajectory parameters in config/trajectory.yaml
1. Launch configuration
    * The figure_eight launch file contains three arguments, gaz, rviz, and plot, which correspond to Gazebo, rviz, and rqt_plot
    * When these arguments are true the corresponding packages are launched
    * By default all three arguments are true
    * Set arguements false either via the command line or via the launch file.
    * The following command launches without rqt_plot:
```
roslaunch turtle_trajectories figure_eight.launch plot:=false
```

#### Testing:
Run `catkin_make run_tests` in root directory of workspace to test python package, homework2

#### Videos:
[![TurtleBot](http://img.youtube.com/vi/SWCIdvja4TE/0.jpg)](http://www.youtube.com/watch?v=SWCIdvja4TE "TurtleBot")


### Xacro Arm

#### This portion of the package models a 2R arm in rviz. The user is able to input the lengths of the arm links. Motion planning is done in the arm_traj node which makes the arm follow a continuous path. The arm_marker node tracks the end effector. This node also publishes markers in rviz showing the path of the end effector's recent motion.

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile: `catkin_make`
1. Start simulation: `roslaunch turtle_trajectories armstrong_attack.launch`
1. Or: `roslaunch turtle_trajectories markers.launch`

#### Configuration Instructions:
1. Configure link lengths and trajectory period of arm in config/arm.yaml
    * L<sub>1</sub> &ne; L<sub>2</sub>
1. Launch configuration (armstrong_attack.launch)
    * The armstrong_attack launch file contains two arguments, rviz and gui, which correspond to rviz and joint_state_publisher_gui
    * By default rviz is true and gui is false
    * When gui is false the arm follows the trajectory published by arm_traj
    * Set arguements true or false either via the command line or via the launch file.
    * The following command launches with the joint_state_publisher_gui instead of arm_traj:
```
roslaunch turtle_trajectories armstrong_attack.launch gui:=true
```
3. Launch configuration (markers.launch)
    * The markers launch file contains one arguement, markers
    * By default markers is true
    * When markers is false the arm follows the trajectory without markers being published to rviz
    * Set arguements true or false either via the command line or via the launch file.
    * The following command launches without the arm_marker node
```
roslaunch turtle_trajectories markers.launch markers:=false
```