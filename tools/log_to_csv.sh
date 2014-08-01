#!/bin/sh
# Examples:
#  log_to_csv.sh state.log model/link.pose
#  log_to_csv.sh state.log model/link.velocity

echo "simTime,x,y,z,roll,pitch,yaw"
gz log -e --raw -f $1 --filter $2 --stamp sim  | tr ' ' ','
