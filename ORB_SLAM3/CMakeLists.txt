cmake_minimum_required(VERSION 3.22)

project(ORB_SLAM3)

# Third party library
add_subdirectory(3rd-party/DBoW2)
add_subdirectory(3rd-party/g2o)

# Add dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

# Source files
file(GLOB ORB_SLAM3_SRC
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cc
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/CameraModels/*.cpp
)

# Header files
set(ORB_SLAM3_INC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/include/CameraModels
)

# Build library
add_library(${PROJECT_NAME} SHARED ${ORB_SLAM3_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${ORB_SLAM3_INC})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC DBoW2 g2o Sophus::Sophus)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} ${Pangolin_LIBRARY} boost_serialization)

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
