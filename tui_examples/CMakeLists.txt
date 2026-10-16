# TUI ncurses example (manual, interactive — not an automated test)

add_executable(MrsUavStatus_NcursesExample)

target_sources(MrsUavStatus_NcursesExample
  PRIVATE
    ncurses_example.cpp
)

target_link_libraries(MrsUavStatus_NcursesExample
  PRIVATE
    mrs_lib::mrs_lib
    Curses::Curses
)

target_compile_definitions(MrsUavStatus_NcursesExample PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

install(
  TARGETS MrsUavStatus_NcursesExample
  DESTINATION lib/${PROJECT_NAME}
)
