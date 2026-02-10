#! /bin/bash

source /opt/ros/humble/setup.bash

if [ "$1" = "debug" ]; then
    build_type="debug"
else
    build_type="release"
fi

echo "Build in $build_type mode"
colcon build --merge-install --mixin $build_type --cmake-clean-cache --cmake-clean-first
