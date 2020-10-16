project(event_camera_depth_map)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)

catkin_simple()

set(HEADERS
    include/position.hpp
    include/event.hpp
    include/voxel_grid.hpp
)

set(SOURCES
    src/position.cpp
    src/event.cpp
    src/voxel_grid.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_add_executable(depth_map src/main.cpp)
target_link_libraries(depth_map ${PROJECT_NAME})

cs_install()

cs_export()
