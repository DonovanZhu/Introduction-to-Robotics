#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment3 grade_asgn3.launch &

sleep 5 
xterm -hold -e "rosrun assignment3 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


