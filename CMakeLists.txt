cmake_minimum_required(VERSION 3.0.2)
project(data_replay)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# find_package(Boost REQUIRED COMPONENTS system
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 message_generation
 sensor_msgs
 nav_msgs
)

add_message_files(
  FILES
  Bestpos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)


#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)


catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mower_driver_ros1.cpp
# )
# link_directories(lib)



add_executable(${PROJECT_NAME}_node src/data_replay_node.cpp
                                    src/data_replay.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Eigen3::Eigen)

