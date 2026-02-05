cmake_minimum_required(VERSION 3.22)

project(orb_slam3)

# set C and C++ standard
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
    set(CMAKE_C_STANDARD_REQUIRED ON)
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# ORB_SLAM3
add_subdirectory(ORB_SLAM3)

# ROS2 package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# source file directory
set(SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

# RGB-D solution
set(RGBD_TARGET rgbd_node)
add_executable(${RGBD_TARGET} ${SOURCE_DIR}/${RGBD_TARGET}.cpp)

ament_target_dependencies(${RGBD_TARGET} PUBLIC
    rclcpp
    sensor_msgs
    cv_bridge
    message_filters
)
target_link_libraries(${RGBD_TARGET} PRIVATE ORB_SLAM3)

install(TARGETS ${RGBD_TARGET}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch ORB_SLAM3/config ORB_SLAM3/vocabulary
        DESTINATION share/${PROJECT_NAME}
)

# export dependencies
ament_export_dependencies(sensor_msgs cv_bridge message_filters)

# make ament package
ament_package()
