#pragma once

/* includes //{ */

#include <ncurses.h>

#include <vector>
#include <string>

//}

namespace mrs_uav_status::tui
{

// A boxed, scrollable list window (used for menus): draws its lines and highlights the selected one.
class StatusWindow {
public:
  // Creates the ncurses window sized to fit text, at (begin_y, begin_x).
  StatusWindow(int begin_y, int begin_x, const std::vector<std::string> &text);

  WINDOW *getWin() const;
  int     getLine() const;

  struct Result
  {
    enum class Action
    {
      None,
      Select,
      Exit
    } action = Action::None;

    int selected_line = -1;
    int pressed_key   = -1;
  };

  // Draws text and applies one keypress: up/down ('k'/'j') moves the highlighted line, 'q'/Escape or
  // empty text exits, any other key returns it as pressed_key with the current selection.
  Result iterate(const std::vector<std::string> &text, int key, bool refresh);
  // Same as above, using the text passed to the constructor instead of a fresh list.
  Result iterate(int key, bool refresh);

private:
  WINDOW                  *win_  = nullptr;
  int                      line_ = 0;
  std::vector<std::string> text_;
};

} // namespace mrs_uav_status::tui
