project(event_camera_depth_map)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)

catkin_simple()

set(HEADERS
    include/event.hpp
    include/event_obj.hpp
    include/voxel_grid.hpp
    include/median_filter.hpp
    include/trajectory.hpp
    include/geometry_utils.hpp
)

set(SOURCES
    src/event.cpp
    src/event_obj.cpp
    src/voxel_grid.cpp
    src/median_filter.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

cs_add_executable(depth_map src/main.cpp)
target_link_libraries(depth_map ${PROJECT_NAME})

cs_install()

cs_export()
