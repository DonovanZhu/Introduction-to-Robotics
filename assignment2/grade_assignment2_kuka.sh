#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment2 grade_asgn2.launch &

sleep 5 
xterm -hold -e "rosrun assignment2 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


