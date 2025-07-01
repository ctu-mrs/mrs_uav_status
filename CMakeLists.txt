cmake_minimum_required(VERSION 3.5)
project(mrs_uav_status)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-format-security)
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  mrs_msgs
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  mrs_lib
  backward_ros
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Curses REQUIRED)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lncurses")

# please, NEVER commit those alternative flags with specific overrides of optimization
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -fopenmp -O0 -lncurses -g")

set(LIBRARIES
  MrsUavStatus_Acquisition
  )

set(EXECUTABLES
  MrsUavStatus_Status
  MrsUavStatus_NcursesTest
  )

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  )

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# NCurses test

add_executable(MrsUavStatus_NcursesTest
    src/ncurses_test.cpp
  )

ament_target_dependencies(MrsUavStatus_NcursesTest
  ${DEPENDENCIES}
  )

target_link_libraries(MrsUavStatus_NcursesTest
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  )

target_compile_definitions(MrsUavStatus_NcursesTest PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# MRS UAV Status

add_executable(MrsUavStatus_Status
    src/status.cpp
    src/menu.cpp
    src/topic_info.cpp
    src/input_box.cpp
  )

ament_target_dependencies(MrsUavStatus_Status
  ${DEPENDENCIES}
  )

target_link_libraries(MrsUavStatus_Status
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  )

target_compile_definitions(MrsUavStatus_Status PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# rclcpp_components_register_nodes(MrsUavStatus_Status PLUGIN "mrs_uav_status::Status" EXECUTABLE MrsUavStatus_Status)

# Data acquisition

add_library(MrsUavStatus_Acquisition SHARED
    src/data_acquisition.cpp
    src/topic_info.cpp
  )

ament_target_dependencies(MrsUavStatus_Acquisition
  ${DEPENDENCIES}
  )

target_link_libraries(MrsUavStatus_Acquisition
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  )

target_compile_definitions(MrsUavStatus_Acquisition PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(MrsUavStatus_Acquisition PLUGIN "mrs_uav_status::Acquisition" EXECUTABLE MrsUavStatus_Acquisition)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
  )

ament_package()
