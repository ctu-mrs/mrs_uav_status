cmake_minimum_required(VERSION 2.8.3)
project(mrs_status)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -fopenmp -O0 -lncurses -g")

catkin_package(
  CATKIN_DEPENDS message_runtime # image_transport roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)

add_executable(mrs_status src/mrs_status.cpp src/status_window.cpp src/menu.cpp)

add_dependencies(mrs_status ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mrs_status
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

install(TARGETS
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		PATTERN ".svn" EXCLUDE
)

install(DIRECTORY doc
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

