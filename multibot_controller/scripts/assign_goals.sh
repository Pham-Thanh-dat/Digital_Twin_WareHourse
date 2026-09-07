#!/bin/bash
declare -A GOALS
GOALS[robot1]="2.0 13.1"
GOALS[robot2]="18.0 27.7"
GOALS[robot3]="10.0 40.0"
GOALS[robot4]="6.0 27.7"
GOALS[robot5]="14.0 13.1"
GOALS[robot6]="18.0 13.1"
GOALS[robot7]="6.0 13.1"
GOALS[robot8]="10.0 27.7"
GOALS[robot9]="14.0 27.7"
GOALS[robot10]="2.0 40.0"

for robot in "${!GOALS[@]}"; do
  read x y <<< "${GOALS[$robot]}"
  echo "$robot: di chuyen den ($x, $y)"
  ros2 service call /multimode_controller/set_goal multibot_interfaces/srv/SetGoal \
    "{robot_namespace: '$robot', x: $x, y: $y}"
  sleep 1.5
done
echo "Da gan xong goal cho tat ca robot."