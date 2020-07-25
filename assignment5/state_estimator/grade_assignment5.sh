#!/bin/bash  

echo "Executing assignment in Xterm windows"

roslaunch state_estimator estimator.launch &

sleep 5 
rosrun state_estimator robot.py &
xterm -hold -e "rosrun state_estimator AutoGrade.py" 

sleep 2 
#Kill all ros nodes
killall -9 rosmaster
killall python


