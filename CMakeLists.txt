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

# MRS UAV Status

add_executable(MrsUavStatus_Status)

target_sources(MrsUavStatus_Status
  PRIVATE
    "src/uav_status.cpp"
    "src/status/uav_status_core.cpp"
    "src/tui/ncurses_tui.cpp"
    "src/tui/pane_box.cpp"
    "src/tui/control_bar.cpp"
    "src/tui/status_window.cpp"
  PRIVATE FILE_SET private_headers TYPE HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_uav_status/uav_status.hpp"
    "include/mrs_uav_status/status/uav_status_core.hpp"
    "include/mrs_uav_status/status/data_types.hpp"
    "include/mrs_uav_status/tui/tui_actions.hpp"
    "include/mrs_uav_status/status/command_sink.hpp"
    "include/mrs_uav_status/tui/pane_box.hpp"
    "include/mrs_uav_status/tui/control_bar.hpp"
    "include/mrs_uav_status/tui/colors.hpp"
    "include/mrs_uav_status/tui/status_window.hpp"
    "include/mrs_uav_status/tui/system_info.hpp"
    "include/mrs_uav_status/tui/print_helpers.hpp"
    "include/mrs_uav_status/tui/constants.hpp"
    "include/mrs_uav_status/tui/ncurses_tui.hpp"
    "include/mrs_uav_status/utils/helpers.hpp"
    "include/mrs_uav_status/status/string_info.hpp"
)

target_link_libraries(MrsUavStatus_Status
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    Curses::Curses
    ${std_msgs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

target_compile_definitions(MrsUavStatus_Status PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)
  add_subdirectory(tui_examples)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
  find_package(ament_cmake_test REQUIRED)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS
    MrsUavStatus_Status
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
