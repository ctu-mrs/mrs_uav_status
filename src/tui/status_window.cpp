/* includes //{ */

#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>

//}

namespace mrs_uav_status::tui
{

/* StatusWindow() //{ */

StatusWindow::StatusWindow(int begin_y, int begin_x, const std::vector<std::string> &text) {

  unsigned long longest_string = 0;

  for (unsigned long line = 0; line < text.size(); line++) {
    if (text[line].length() > longest_string) {
      longest_string = text[line].length();
    }
  }
  text_ = text;
  win_  = newwin(text_.size() + 2, longest_string + 2, begin_y, begin_x);
}

//}

/* getWin() //{ */

WINDOW *StatusWindow::getWin() const {
  return win_;
}

//}

/* getLine() //{ */

int StatusWindow::getLine() const {
  return line_;
}

//}

/* iterate(int key, bool refresh) //{ */

StatusWindow::Result StatusWindow::iterate(const std::vector<std::string> &text, int key, bool refresh) {

  Result result;

  wattron(win_, A_BOLD);

  if (key == 'q' || key == static_cast<int>(Key::Escape) || text.empty()) {
    result.action = Result::Action::Exit;
    wattroff(win_, A_BOLD);
    return result;
  }

  wattron(win_, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  box(win_, 0, 0);
  wattroff(win_, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

  for (unsigned long j = 0; j < text.size(); j++) {
    mvwaddstr(win_, j + 1, 1, text[j].c_str());
  }

  if (key == KEY_UP || key == 'k') {
    line_--;
    line_ = (line_ < 0) ? text.size() - 1 : line_;
  } else if (key == KEY_DOWN || key == 'j') {
    line_++;
    line_ = (line_ > int(text.size() - 1)) ? 0 : line_;
  } else {
    result.pressed_key   = key;
    result.selected_line = line_;
  }

  wattron(win_, A_STANDOUT);
  mvwaddstr(win_, line_ + 1, 1, text[line_].c_str());
  wattroff(win_, A_STANDOUT);

  if (refresh) {
    wrefresh(win_);
  }

  wattroff(win_, A_BOLD);
  return result;
}

//}

/* iterate() //{ */

StatusWindow::Result StatusWindow::iterate(int key, bool refresh) {
  return iterate(text_, key, refresh);
}

//}

} // namespace mrs_uav_status::tui
