#pragma once

namespace mrs_uav_status::tui
{

// Specific key codes used for terminal navigation. The arrow/delete values mirror ncurses' KEY_*
// macros so that ROS- and ncurses-free code (UavStatusCore) can compare against them without
// including <curses.h>; src/tui/ncurses_tui.cpp static_asserts that they still match.
enum class Key : int
{
  Enter  = 10,
  Escape = 27,
  Delete = 330, // KEY_DC
  Down   = 258, // KEY_DOWN
  Up     = 259, // KEY_UP
  Left   = 260, // KEY_LEFT
  Right  = 261  // KEY_RIGHT
};

// The logical pairs initialized via init_pair() in ncurses.
enum class ColorPair : int
{
  Normal = 100,
  Field  = 101,
  Green  = 102,
  Red    = 103,
  Yellow = 104
};

// Useful for specifying transparency.
enum class BackgroundColor : int
{
  Default = -1
};

// Standard xterm-256 color codes.
enum class Color : int
{
  NiceRed    = 196,
  NiceGreen  = 82,
  DarkGreen  = 2,
  NiceBlue   = 33,
  DarkBlue   = 19,
  NiceYellow = 220,
  DarkYellow = 172
};

} // namespace mrs_uav_status::tui
