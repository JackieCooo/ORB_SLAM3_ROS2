# ORB_SLAM3_ROS2

ORB_SLAM3 with ROS2 support

## Environment

OS: Ubuntu 22.04 LTS
ROS Distro: Humble

## Dependencies

### Pangolin
[Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. **Tested with Pangolin 0.9.1**.

```shell
sudo apt install ros-humble-pangolin
```

### OpenCV
[OpenCV](http://opencv.org) for manipulating images and features. **Tested with OpenCV 4.5.x**.

```shell
sudo apt install libopencv-dev
```

### Eigen3
[Eigen3](https://libeigen.gitlab.io/) for linear algebra computation like matrices, vectors, numerical solvers, and related algorithms. **Tested with Eigen3 3.4.0**.

```shell
sudo apt install libeigen3-dev
```

### Sophus
[Sophus](https://github.com/strasdat/Sophus) for Lie groups computation. **Tested with Sophus 1.22.x**.

```shell
sudo apt install ros-humble-sophus
```

## How to Use

We use binary vocabulary file for faster loading and lower file size. The converted vocabulary file is placed in `ORB_SLAM3/vocabulary/ORBvoc.bin`. We also provide a python script for convertion in `asset/python/voc2bin.py`, in case you want to use your own vocabulary.

### Compile

Simply run `./build.sh` if your're using the same ROS distro with me. If not, modify the second line `source /opt/ros/humble/setup.bash` to suit your ROS environment.

### Run

Running ORB_SLAM3 with Realsense D435i module for example.
> Currently only support RGB-D mode.

```shell
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orb_slam3 realsense_435i.launch.py
```

## To-do List

- [ ] Make visualization optional
- [ ] Support [Foxglove](https://foxglove.dev/homepage) for visualization
- [ ] Implement RGB-D with inertial mode
- [x] Make vocabulary file binary
