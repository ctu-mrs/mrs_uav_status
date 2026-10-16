find_package(ament_cmake_gtest REQUIRED)

add_subdirectory(uav_status_core)

add_test(
  NAME check_tui_ros_free
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/check_tui_ros_free.sh" "${PROJECT_SOURCE_DIR}"
)
