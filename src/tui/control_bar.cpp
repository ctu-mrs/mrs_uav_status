/* includes //{ */

#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/constants.hpp>

#include <cctype>
#include <cstdio>
#include <cstdlib>

//}

namespace mrs_uav_status::tui
{

/* ControlBar() //{ */

ControlBar::ControlBar(unsigned long size, WINDOW *win, double initial_value) {

  size_ = size;
  win_  = win;
  buffer_.resize(size_, ' ');

  ControlBar::cursor_ = (size_ / 2) - 2;

  const int           required_length = std::snprintf(nullptr, 0, "%6.2f", initial_value);
  const unsigned long tmpbuffer_size =
      size_ + 1 > static_cast<unsigned long>(required_length + 1) ? size_ + 1 : static_cast<unsigned long>(required_length + 1);
  std::vector<char> tmpbuffer(tmpbuffer_size, '\0');

  if (std::snprintf(tmpbuffer.data(), tmpbuffer.size(), "%6.2f", initial_value) < 0) {
    tmpbuffer[0] = '\0';
  }

  for (unsigned long i = 0; i < size_; i++) {
    if (std::isdigit(tmpbuffer[i]) || tmpbuffer[i] == '.' || tmpbuffer[i] == '-') {
      buffer_[i] = tmpbuffer[i];
    } else {
      buffer_[i] = ' ';
    }
  }
}

//}

/* process() //{ */

unsigned long ControlBar::process(int key) {

  switch (key) {
  case -1:
    return ControlBar::cursor_;

  case KEY_LEFT:
  case 'h':
    if (ControlBar::cursor_ > 0) {
      ControlBar::cursor_--;
    }
    break;

  case KEY_RIGHT:
  case 'l':
    if (ControlBar::cursor_ + 1 < size_) {
      ControlBar::cursor_++;
    }
    break;

  case KEY_BACKSPACE:
    if (ControlBar::cursor_ > 0) {
      buffer_.erase(buffer_.begin() + ControlBar::cursor_ - 1);
      ControlBar::cursor_--;
    }
    break;

  case static_cast<int>(Key::Delete):
    buffer_.erase(buffer_.begin() + ControlBar::cursor_);
    break;

  default:
    if (ControlBar::cursor_ < size_) {

      if (std::isdigit(key) || key == '.' || key == '-') {
        if (buffer_[size_ - 1] != ' ') {
          if (buffer_[0] == ' ') {
            buffer_.insert(buffer_.begin() + ControlBar::cursor_, key);
            buffer_.erase(buffer_.begin());
            return ControlBar::cursor_;

          } else {

            return ControlBar::cursor_;
          }
        }

        buffer_.insert(buffer_.begin() + ControlBar::cursor_, key);

        if (ControlBar::cursor_ + 1 < size_) {
          ControlBar::cursor_++;
        }
      }
    }
    break;
  }
  buffer_.resize(size_, ' ');
  return ControlBar::cursor_;
}

//}

/* print() //{ */

void ControlBar::print(int line, bool active) {

  wattron(win_, COLOR_PAIR(static_cast<int>(ColorPair::Field)));
  wattron(win_, A_BOLD);

  for (unsigned long i = 0; i < buffer_.size(); i++) {
    if (i == ControlBar::cursor_ && active) {
      wattron(win_, A_UNDERLINE);
    }
    mvwprintw(win_, line, 10 + i, "%c", buffer_[i]);
    wattroff(win_, A_UNDERLINE);
  }

  wattroff(win_, A_BOLD);
  wattroff(win_, COLOR_PAIR(static_cast<int>(ColorPair::Field)));
}

//}

/* getDouble() //{ */

double ControlBar::getDouble() const {

  double ret_val;

  std::vector<char> tmparr(buffer_.begin(), buffer_.end());
  tmparr.push_back('\0');

  char *ptr;

  ret_val = strtod(tmparr.data(), &ptr);

  return ret_val;
}

//}

} // namespace mrs_uav_status::tui
