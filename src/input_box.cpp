#include <input_box.h>

/* Menu() //{ */

InputBox::InputBox(int size, WINDOW* win) {
  size_ = size;
  win_  = win;
  buffer_.resize(size_, ' ');

  cursor_ = size_ / 2;

  buffer_[cursor_]     = '.';
  buffer_[cursor_ + 1] = '0';
  buffer_[cursor_ - 1] = '0';
}

//}

/* Process() //{ */

void InputBox::Process(int key) {

  switch (key) {
    case -1:
      return;

    case KEY_LEFT:
      if (cursor_ > 0) {
        cursor_--;
      }
      break;

    case KEY_RIGHT:
      if (cursor_ + 1 < size_) {
        cursor_++;
      }
      break;

    case KEY_BACKSPACE:
      if (cursor_ > 0) {
        buffer_.erase(buffer_.begin() + cursor_ - 1);
        cursor_--;
      }
      break;

    case KEY_DELETE:
      buffer_.erase(buffer_.begin() + cursor_);
      break;

    default:
      if (cursor_ < size_) {

        if (buffer_[size_ - 1] != ' ') {
          if (buffer_[0] == ' ') {
            buffer_.insert(buffer_.begin() + cursor_, key);
            buffer_.erase(buffer_.begin());
            return;

          } else {

            return;

          }
        }

        buffer_.insert(buffer_.begin() + cursor_, key);

        if (cursor_ + 1 < size_) {
          cursor_++;
        }
      }
      break;
  }
  buffer_.resize(size_, ' ');
}

//}

/* Print() //{ */

void InputBox::Print() {

  mvwprintw(win_, 2, 0, "curs: %i", cursor_);

  mvwprintw(win_, 0, 0, "Contents:  ");

  wattron(win_, COLOR_PAIR(FIELD));
  wattron(win_, A_BOLD);

  for (unsigned long i = 0; i < buffer_.size(); i++) {
    if (i == cursor_) {
      wattron(win_, A_UNDERLINE);
    }
    mvwprintw(win_, 0, 10 + i, "%c", buffer_[i]);
    wattroff(win_, A_UNDERLINE);
  }

  wattroff(win_, A_BOLD);
  wattroff(win_, COLOR_PAIR(FIELD));
}

//}

/* getDouble() //{ */

double InputBox::getDouble() {
}

//}

