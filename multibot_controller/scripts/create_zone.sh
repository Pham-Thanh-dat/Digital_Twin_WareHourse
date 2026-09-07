#!/bin/bash
# Cach dung: ./create_zone.sh <zone_id> <x> <y> <radius>
# Vi du:     ./create_zone.sh zone2 8.0 27.7 1.0
#
# Script nay lam CA 2 VIEC CUNG LUC:
#   1. Spawn hinh tru do trong Gazebo (de nhin thay)
#   2. Goi service declare_zone (de robot THUC SU nhan duoc data va ne)
# Truoc day hay quen buoc 2 khien robot xuyen qua du van thay hinh do.

set -e

if [ "$#" -ne 4 ]; then
  echo "Cach dung: $0 <zone_id> <x> <y> <radius>"
  exit 1
fi

ZONE_ID=$1
X=$2
Y=$3
RADIUS=$4

echo "=== Buoc 1/2: Spawn hinh tru do trong Gazebo ==="
~/ros2_ws/src/multibot_controller/scripts/spawn_congestion_zone.sh "$ZONE_ID" "$X" "$Y" "$RADIUS"

echo ""
echo "=== Buoc 2/2: Khai bao data de robot THUC SU ne ==="
ros2 service call /congestion_zone_manager/declare_zone \
  multibot_interfaces/srv/DeclareCongestionZone \
  "{zone_id: '${ZONE_ID}', x: ${X}, y: ${Y}, radius: ${RADIUS}, clear: false}"

echo ""
echo "Da tao xong zone '$ZONE_ID' (ca hinh anh lan data ne)."
