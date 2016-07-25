cmake_minimum_required(VERSION 2.8.3)
project(sensor_save)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  sensor_msgs
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(sensor_save_node src/sensor_save_node.cpp)


 add_dependencies(sensor_save_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(sensor_save_node
   ${catkin_LIBRARIES}
 )




