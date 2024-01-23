
cmake_minimum_required(VERSION 2.6)
project(lineLib)

set(CMAKE_CXX_STANDARD 11)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(Boost REQUIRED COMPONENTS system thread)

if (NOT Boost_FOUND)
    message("NOT found Boost")
endif ()
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        sensor_msgs
        laser_geometry
        pcl_conversions
        pcl_ros
)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# if(YAML_CPP_LIBRARIES)
#     message(STATUS "Found yaml-cpp: ${YAML_CPP_LIBRARIES}")
#     target_link_libraries(xian_line_1 PRIVATE ${YAML_CPP_LIBRARIES})
# else()
#     message(FATAL_ERROR "yaml-cpp not found!")
# endif()
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(lineLib
        src/line_lasder.cpp
)

# target_link_libraries (xian_line   PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES}   boost_thread boost_system -pthread)
target_link_libraries(lineLib ${PCL_LIBRARIES} yaml-cpp boost_thread boost_system -pthread)

add_executable(main main.cpp
        src/rosbridge.cpp
        include/rosbridge.h)
target_link_libraries(main
        lineLib
        ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp boost_thread boost_system -pthread
)

