cmake_minimum_required(VERSION 3.22)

PROJECT(g2o)

# default built type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# specifying compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Generate config.h
configure_file(config.h.in ${CMAKE_CURRENT_LIST_DIR}/config.h)

# Header files
set(G2O_INC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/g2o/core
    ${CMAKE_CURRENT_LIST_DIR}/g2o/solvers
    ${CMAKE_CURRENT_LIST_DIR}/g2o/stuff
    ${CMAKE_CURRENT_LIST_DIR}/g2o/types
)

# Add source files
file(GLOB_RECURSE G2O_SRC ${CMAKE_CURRENT_LIST_DIR}/g2o/*.cpp)

# Build library
add_library(${PROJECT_NAME} SHARED ${G2O_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${G2O_INC})
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
