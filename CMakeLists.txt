cmake_minimum_required(VERSION 3.28)
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
  add_compile_options("-Wall" "-Wextra" "-Wno-format-security")
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Curses REQUIRED)

find_package(std_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# From CMake docs: https://cmake.org/cmake/help/v4.2/module/FindCurses.html
if(Curses_FOUND AND NOT TARGET Curses::Curses)
  add_library(Curses::Curses INTERFACE IMPORTED)
  set_target_properties(
    Curses::Curses
    PROPERTIES
      INTERFACE_LINK_LIBRARIES "${CURSES_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${CURSES_INCLUDE_DIRS}"
  )
endif()

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# NCurses test

add_executable(MrsUavStatus_NcursesTest)

target_sources(MrsUavStatus_NcursesTest
  PRIVATE
    src/ncurses_test.cpp
)

target_link_libraries(MrsUavStatus_NcursesTest
  PRIVATE
    mrs_lib::mrs_lib
    Boost::filesystem
    Curses::Curses
)

target_compile_definitions(MrsUavStatus_NcursesTest PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# MRS UAV Status

add_executable(MrsUavStatus_Status)

target_sources(MrsUavStatus_Status
  PRIVATE
    "src/input_box.cpp"
    "src/menu.cpp"
    "src/status.cpp"
    "src/topic_info.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/commons.h"
    "include/input_box.h"
    "include/menu.h"
)

target_link_libraries(MrsUavStatus_Status
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    Boost::filesystem
    Curses::Curses
    ${std_msgs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(MrsUavStatus_Status PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Data acquisition

add_library(MrsUavStatus_Acquisition SHARED)

target_sources(MrsUavStatus_Acquisition
  PRIVATE
    "src/data_acquisition.cpp"
    "src/topic_info.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/commons.h"
    "include/input_box.h"
    "include/menu.h"
)

target_link_libraries(MrsUavStatus_Acquisition
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    Boost::filesystem
    Curses::Curses
    ${std_msgs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(MrsUavStatus_Acquisition PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(MrsUavStatus_Acquisition "mrs_uav_status::Acquisition")

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS MrsUavStatus_Acquisition
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    MrsUavStatus_NcursesTest
    MrsUavStatus_Status
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

ament_package()
