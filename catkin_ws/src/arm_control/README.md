1. `source ./devel/setup.bash`
2.  `roslaunch part1_gazebo arm_world.launch`
3.  `roslaunch arm_control arm_control.launch`

Follow the [tutorial](http://gazebosim.org/tutorials/?tut=ros_control) to adjust PD controller


If you meet `controller type 'effort_controllers/JointPositionController' does not exist` this error
try `sudo apt-get install ros-noetic-effort-controllers`
