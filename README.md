# RBE500 ROS Project (built in a team of 3)
- Put part1, part2 and part3 packages in the catkin_ws/src folder
- Only push the package and not your devel, log, and build folders from catkin_ws (since these are different for each computer). Ideally the .gitignore should prevent this from happening by default when you do a git add.

### Build Instructions:
Run `catkin build` or `catkin_make` in the catkin_ws folder.
`source devel/setup.bash`

### Part1 Instructions:
- Run: `roslaunch part1_gazebo arm_world.launch`
- Gazebo joint positions are published to `/gazebo/link_states`
- Arm can be moved using gazebo servicie of `/gazebo/apply_joint_effort`

### Part2 Instructions:
- Run: 
1. `roslaunch part1_gazebo arm_world.launch` 
2. `rosrun part2_gazebo part2_gazebo.py`
3. `rostopic echo /chatter`

### Part3 Instructions:
- Run: `rosservice call /invkin x y z`
