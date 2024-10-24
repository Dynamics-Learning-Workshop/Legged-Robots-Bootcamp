cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    joint_state_controller
    gazebo_ros
    genmsg
    robot_state_publisher
    roscpp
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
    ros_utilities
    # gazebo
)

# catkin_package(
#     CATKIN_DEPENDS 
#     unitree_legged_msgs 
# )

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

add_executable(
    main 
    src/main.cpp
)
target_link_libraries(main
    ${catkin_LIBRARIES}
)
add_dependencies(main 
    unitree_legged_msgs_generate_messages_cpp
)
