cmake_minimum_required(VERSION 3.0.2)
project(wifly2)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  grid_map_core
  grid_map_ros
  grid_map_visualization
  roscpp
  rospy
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  intensity.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES wifly2
 CATKIN_DEPENDS geometry_msgs grid_map_core grid_map_ros grid_map_visualization roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  position_query
  src/position_query.cpp
)

add_dependencies(position_query ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

target_link_libraries(
  position_query
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  position_query

	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/intensity_client_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)
