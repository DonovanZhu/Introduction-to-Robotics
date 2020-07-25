#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch assignment4 grade_asgn4.launch &

sleep 5 
xterm -hold -e "rosrun assignment4 AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster


