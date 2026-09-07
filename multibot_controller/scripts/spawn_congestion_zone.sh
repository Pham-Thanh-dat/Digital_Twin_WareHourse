#!/bin/bash
# Cach dung: ./spawn_congestion_zone.sh <zone_id> <x> <y> <radius>
# Vi du:     ./spawn_congestion_zone.sh zone1 10.0 27.7 2.0

set -e

if [ "$#" -ne 4 ]; then
  echo "Cach dung: $0 <zone_id> <x> <y> <radius>"
  exit 1
fi

ZONE_ID=$1
X=$2
Y=$3
RADIUS=$4

TEMPLATE=~/ros2_ws/src/multibot_controller/worlds/congestion_zone_template.sdf
TMP_SDF="/tmp/congestion_${ZONE_ID}.sdf"

# Thay placeholder trong template bang gia tri thuc te
sed "s/ZONE_NAME_PLACEHOLDER/${ZONE_ID}/; s/ZONE_RADIUS_PLACEHOLDER/${RADIUS}/" \
  "$TEMPLATE" > "$TMP_SDF"

echo "Spawning zone '$ZONE_ID' tai ($X, $Y) ban kinh $RADIUS ..."

ros2 run ros_gz_sim create \
  -world warehouse_columns \
  -file "$TMP_SDF" \
  -name "$ZONE_ID" \
  -x "$X" -y "$Y" -z 0.1

echo "Da spawn xong. Kiem tra trong Gazebo GUI."
