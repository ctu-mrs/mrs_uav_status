#pragma once

/* includes //{ */

#include <ncurses.h>

#include <vector>

//}

namespace mrs_uav_status::tui
{

// A single-line numeric text-entry field drawn in an ncurses window (used by the goto menu).
class ControlBar {
public:
  // Seeds the buffer with initial_value formatted as "%6.2f".
  ControlBar(unsigned long size, WINDOW *win, double initial_value);

  // Applies one keypress (move cursor, backspace/delete, or insert a digit/'.'/'-'). Returns the cursor position.
  unsigned long process(int key);
  // Draws the buffer on the given window line; underlines the cursor if active.
  void print(int line, bool active);

  // Parses the buffer as a double; 0.0 if it doesn't parse.
  double getDouble() const;

  inline static unsigned long cursor_ = 0;

private:
  WINDOW           *win_;
  unsigned long     size_;
  std::vector<char> buffer_;
};

} // namespace mrs_uav_status::tui
