get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

# UavStatusCore is ROS-free by construction -- this target deliberately links no rclcpp/mrs_lib.
ament_add_gtest(test_${TEST_NAME}
  test.cpp
  "${PROJECT_SOURCE_DIR}/src/status/uav_status_core.cpp"
)

target_include_directories(test_${TEST_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/include")
