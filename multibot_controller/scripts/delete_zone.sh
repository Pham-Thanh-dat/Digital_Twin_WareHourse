#!/bin/bash
# Cach dung: ./delete_zone.sh <zone_id>
# Vi du:     ./delete_zone.sh zone2
#
# Xoa CA hinh tru trong Gazebo LAN data trong /congestion_zones cung luc.

set -e

if [ "$#" -ne 1 ]; then
  echo "Cach dung: $0 <zone_id>"
  exit 1
fi

ZONE_ID=$1

echo "=== Buoc 1/2: Xoa hinh tru khoi Gazebo ==="
~/ros2_ws/src/multibot_controller/scripts/remove_congestion_zone.sh "$ZONE_ID"

echo ""
echo "=== Buoc 2/2: Xoa data khoi danh sach ne ==="
ros2 service call /congestion_zone_manager/declare_zone \
  multibot_interfaces/srv/DeclareCongestionZone \
  "{zone_id: '${ZONE_ID}', x: 0.0, y: 0.0, radius: 0.0, clear: true}"

echo ""
echo "Da xoa xong zone '$ZONE_ID' (ca hinh anh lan data ne)."
