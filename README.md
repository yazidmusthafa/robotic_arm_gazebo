Robotic_Arm Cartesian Path Planning

1. robotic_arm_v3 working having previous issue solved and successful trajectory planning
   with ERROR: [moveit_visual_tools]: Unable to get end effector tips from jmg

2. Following files are only for Reference and learning:
   1. c2.cpp             (Official cartesian Path code from moveit2 documentation)
   2. linear_movement.cpp     (Unfinished work of attempting linear movement using Task_constructor)
   3. robotic_arm_moveit.cpp   (Allows robotic_arm to move to a pose)
   4. pick_n_place_reference.cpp   (pick and place code from moveit2 Documentation)

3. Steps To run the program:
   1. Open terminal 1 -> clone this repo into a <workspace>/src
   2. run -> colcon build
   3. run -> . install/setup.bash
   4. run -> ros2 launch robotic_arm_v3 Robot.launch.py
   5. Open terminal 2
   6. run -> . install/setup.bash
   7. wait for a few seconds to load the robot model in RViz.
   8. Wait a little before going to next step, to give time to load everything internally.
   9. run -> ros2 launch robotic_arm_v3 cartesian_planning.launch.py    (This launch file will run cartesian_planning.cpp with necessary parameters)
