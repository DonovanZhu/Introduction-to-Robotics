#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment2 grade_asgn2.launch robot_urdf_location:='$(find ur_description)/urdf/ur5_robot.urdf.xacro' &

sleep 5 
xterm -hold -e "rosrun assignment2 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


