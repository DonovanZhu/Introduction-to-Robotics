#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment1 grade_asgn1.launch &

sleep 2 
xterm -hold -e "rosrun assignment1 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


