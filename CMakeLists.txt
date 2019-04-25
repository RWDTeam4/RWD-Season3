cmake_minimum_required(VERSION 2.8.3)
project(rwd)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_filters message_generation rostime roscpp rosconsole roscpp_serialization)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(/usr/include/libevdev-1.0/libevdev/)
link_directories(${catkin_LIBRARY_DIRS})
link_directories(/usr/lib/x86_64-linux-gnu/)

add_message_files(
   FILES
   Speed.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_filters message_runtime std_msgs)

macro(rostutorial T)
  add_executable(${T} ${T}/${T}.cpp)
  target_link_libraries(${T} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(${T} rwd_gencpp)
  install(TARGETS ${T}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

foreach(dir
    robot_controller
    )
  rostutorial(${dir})
endforeach()

#add_executable(time_api_sleep time_api/sleep/sleep.cpp)
#target_link_libraries(time_api_sleep ${catkin_LIBRARIES})
target_link_libraries(robot_controller evdev)
#install(TARGETS time_api_sleep
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES
#  launch/robot_controller_listener.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)
