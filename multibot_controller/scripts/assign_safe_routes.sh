#!/bin/bash
# Route an toan: di doc vung ngoai (x_spawn) toi hanh lang ngang y=13.1,
# re ngang qua hanh lang do toi dung lan doc x_dich, roi di doc len/xuong
# toi dung y_dich. Khong bao gio cat qua ket cau ke o gia tri x sai cho.

declare -A SPAWN
SPAWN[robot1]="24.6 2.0"
SPAWN[robot2]="25.6 2.0"
SPAWN[robot3]="26.6 2.0"
SPAWN[robot4]="27.6 2.0"
SPAWN[robot5]="28.6 2.0"
SPAWN[robot6]="24.6 4.0"
SPAWN[robot7]="25.6 4.0"
SPAWN[robot8]="26.6 4.0"
SPAWN[robot9]="27.6 4.0"
SPAWN[robot10]="28.6 4.0"

declare -A GOAL
GOAL[robot1]="2.0 13.1"
GOAL[robot2]="18.0 27.7"
GOAL[robot3]="10.0 40.0"
GOAL[robot4]="6.0 27.7"
GOAL[robot5]="14.0 13.1"
GOAL[robot6]="18.0 13.1"
GOAL[robot7]="6.0 13.1"
GOAL[robot8]="10.0 27.7"
GOAL[robot9]="14.0 27.7"
GOAL[robot10]="2.0 40.0"

ENTRY_Y=13.1   # hanh lang ngang dung de cat qua toan bo khu ke an toan

for robot in "${!SPAWN[@]}"; do
  read sx sy <<< "${SPAWN[$robot]}"
  read gx gy <<< "${GOAL[$robot]}"
  echo "$robot: ($sx,$sy) -> qua ($sx,$ENTRY_Y) -> ($gx,$ENTRY_Y) -> ($gx,$gy)"
  ros2 service call /multimode_controller/set_route multibot_interfaces/srv/SetRoute \
    "{robot_namespace: '$robot', xs: [$sx, $sx, $gx, $gx], ys: [$sy, $ENTRY_Y, $ENTRY_Y, $gy]}" &
done

wait
echo "Da gan xong route an toan cho tat ca robot."

# ==== Ham quay ve spawn (goi bang: bash assign_safe_routes.sh --return) ====
return_to_spawn() {
  for robot in "${!SPAWN[@]}"; do
    read sx sy <<< "${SPAWN[$robot]}"
    read gx gy <<< "${GOAL[$robot]}"
    echo "$robot: quay ve tu ($gx,$gy) -> ($sx,$sy)"
    ros2 service call /multimode_controller/set_route multibot_interfaces/srv/SetRoute \
      "{robot_namespace: '$robot', xs: [$gx, $gx, $sx, $sx], ys: [$gy, 13.1, 13.1, $sy]}" &
  done
  wait
  echo "Da quay ve spawn xong."
}

if [ "$1" == "--return" ]; then
  return_to_spawn
fi
