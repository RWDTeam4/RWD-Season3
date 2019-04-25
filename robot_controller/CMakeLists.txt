cmake_minimum_required(VERSION 2.8)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(robot_controller robot_controller.cpp)
target_link_libraries(robot_controller ${catkin_LIBRARIES})
install(TARGETS robot_controller
  RUNTIME DESTINATION .)
