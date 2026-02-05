FROM ros:humble-ros-base

# Replace APT source if your're in Mainland China
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    sed -i 's|http://ports.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

RUN apt update && \
    apt -y upgrade && \
    apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-cv-bridge ros-humble-pangolin ros-humble-sophus libeigen3-dev libopencv-dev libboost-all-dev && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /var/cache/* && \
    mkdir -p /var/cache/apt/archives/partial && \
    rm -rf /var/log/*

CMD [ "/bin/bash", "-c", "uname -m; exec /bin/bash" ]
