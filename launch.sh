#!/bin/bash

if [[ $# -lt 2 ]]; then
    echo "Error: Invalid parameter"
    echo "Usgae: $0 <package_name> <launch_file> [args]"
    exit 1
fi

cur_dir="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
source $cur_dir/install/setup.sh
export ROS_LOG_DIR=/tmp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" /></Interfaces></General></Domain></CycloneDDS>'

package_name=$1
launch_file=$2
shift 2

if [[ $# -eq 0 ]]; then
    ros2 launch $package_name $launch_file
else
    ros2 launch $package_name $launch_file $@
fi
