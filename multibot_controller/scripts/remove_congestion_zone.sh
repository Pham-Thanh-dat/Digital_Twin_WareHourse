#!/bin/bash
# Cach dung: ./remove_congestion_zone.sh <zone_id>
# Vi du:     ./remove_congestion_zone.sh zone1

set -e

if [ "$#" -ne 1 ]; then
  echo "Cach dung: $0 <zone_id>"
  exit 1
fi

ZONE_ID=$1

echo "Xoa zone '$ZONE_ID' khoi Gazebo ..."

ign service --service /world/warehouse_columns/remove \
  --reqtype ign_msgs.Entity \
  --reptype ign_msgs.Boolean \
  --timeout 2000 \
  --req "name: \"${ZONE_ID}\" type: MODEL"

echo "Da xoa xong (neu ton tai)."
