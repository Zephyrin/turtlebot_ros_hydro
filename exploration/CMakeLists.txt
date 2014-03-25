cmake_minimum_required(VERSION 2.8.3)
project(exploration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  costmap_2d
  navfn
  tf
  geometry_msgs
  dynamic_reconfigure
  dwa_local_planner
  move_base_msgs
  actionlib
)

generate_dynamic_reconfigure_options(config/exploration.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(frontiers src/frontieres.cpp)
target_link_libraries(frontiers ${catkin_LIBRARIES}) 

add_executable(decision src/decision_making.cpp)
target_link_libraries(decision ${catkin_LIBRARIES}) 

add_executable(build_map src/build_map.cpp)
target_link_libraries(build_map ${catkin_LIBRARIES})

#add_executable(exploration src/exploration.cpp)
#add_dependencies(exploration ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(exploration ${catkin_LIBRARIES})

