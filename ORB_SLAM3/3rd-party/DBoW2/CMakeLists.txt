cmake_minimum_required(VERSION 3.22)

project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(OpenCV REQUIRED)

set(DBOW2_INC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/include/DBoW2
)
file(GLOB DBOW2_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${DBOW2_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${DBOW2_INC})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
