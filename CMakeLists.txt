cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_status)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  mavros_msgs
  message_generation
  mrs_msgs
  mrs_lib
  topic_tools
  )

find_package(Curses REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lncurses")

# please, NEVER commit those alternative flags with specific overrides of optimization
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -fopenmp -O0 -lncurses -g")

set(EXECUTABLES
  mrs_uav_status data_acquisition
  )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs mavros_msgs mrs_msgs mrs_lib
  )

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  )

# MRS UAV Status

add_executable(mrs_uav_status
  src/status.cpp
  src/menu.cpp
  src/topic_info.cpp
  src/input_box.cpp
  )

add_dependencies(mrs_uav_status
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mrs_uav_status
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
  )

# Data acquisition

add_executable(data_acquisition
  src/data_acquisition.cpp
  src/topic_info.cpp
  )

add_dependencies(data_acquisition
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(data_acquisition
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
