#!/bin/bash
echo "=================================================="
echo "So sanh ODOM vs VI TRI THAT (Gazebo ground-truth)"
echo "=================================================="

for i in 1 2 3 4 5; do
  echo ""
  echo "--- robot$i ---"
  
  odom_result=$(timeout 3 ros2 topic echo /robot$i/odom --field pose.pose.position --once 2>/dev/null)
  odom_x=$(echo "$odom_result" | grep "x:" | awk '{print $2}')
  odom_y=$(echo "$odom_result" | grep "y:" | awk '{print $2}')
  
  real_result=$(timeout 3 ign topic -e -t /world/warehouse_columns/pose/info -n 1 2>/dev/null | grep -A6 "\"turtlebot4_$i\"")
  real_x=$(echo "$real_result" | grep "x:" | head -1 | awk '{print $2}')
  real_y=$(echo "$real_result" | grep "y:" | head -1 | awk '{print $2}')
  
  echo "  ODOM: x=$odom_x  y=$odom_y"
  echo "  THAT: x=$real_x  y=$real_y"
  
  if [[ -n "$odom_x" && -n "$real_x" ]]; then
    diff=$(python3 -c "import math; print(round(math.hypot($real_x-$odom_x, $real_y-$odom_y), 3))" 2>/dev/null)
    echo "  SAI LECH: ${diff}m"
  fi
done
echo "=================================================="
