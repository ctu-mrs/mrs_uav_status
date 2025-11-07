#include <input_box.h>

/* InputBox() //{ */

InputBox::InputBox(int size, WINDOW *win, double initial_value) {

  size_ = size;
  win_  = win;
  buffer_.resize(size_, ' ');

  InputBox::cursor_ = (size_ / 2) - 2;

  char tmpbuffer[size_];

  sprintf(tmpbuffer, "%6.2f", initial_value);

  for (unsigned long i = 0; i < size_; i++) {
    if (std::isdigit(tmpbuffer[i]) || tmpbuffer[i] == '.' || tmpbuffer[i] == '-') {
      buffer_[i] = tmpbuffer[i];
    } else {
      buffer_[i] = ' ';
    }
  }
}

//}

/* Process() //{ */

unsigned long InputBox::Process(int key) {

  switch (key) {
  case -1:
    return InputBox::cursor_;

  case KEY_LEFT:
  case 'h':
    if (InputBox::cursor_ > 0) {
      InputBox::cursor_--;
    }
    break;

  case KEY_RIGHT:
  case 'l':
    if (InputBox::cursor_ + 1 < size_) {
      InputBox::cursor_++;
    }
    break;

  case KEY_BACKSPACE:
    if (InputBox::cursor_ > 0) {
      buffer_.erase(buffer_.begin() + InputBox::cursor_ - 1);
      InputBox::cursor_--;
    }
    break;

  case KEY_DELETE:
    buffer_.erase(buffer_.begin() + InputBox::cursor_);
    break;

  default:
    if (InputBox::cursor_ < size_) {

      if (std::isdigit(key) || key == '.' || key == '-') {
        if (buffer_[size_ - 1] != ' ') {
          if (buffer_[0] == ' ') {
            buffer_.insert(buffer_.begin() + InputBox::cursor_, key);
            buffer_.erase(buffer_.begin());
            return InputBox::cursor_;

          } else {

            return InputBox::cursor_;
          }
        }

        buffer_.insert(buffer_.begin() + InputBox::cursor_, key);

        if (InputBox::cursor_ + 1 < size_) {
          InputBox::cursor_++;
        }
      }
    }
    break;
  }
  buffer_.resize(size_, ' ');
  return InputBox::cursor_;
}

//}

/* Print() //{ */

void InputBox::Print(int line, bool active) {

  wattron(win_, COLOR_PAIR(FIELD));
  wattron(win_, A_BOLD);

  for (unsigned long i = 0; i < buffer_.size(); i++) {
    if (i == InputBox::cursor_ && active) {
      wattron(win_, A_UNDERLINE);
    }
    mvwprintw(win_, line, 10 + i, "%c", buffer_[i]);
    wattroff(win_, A_UNDERLINE);
  }

  wattroff(win_, A_BOLD);
  wattroff(win_, COLOR_PAIR(FIELD));
}

//}

/* getDouble() //{ */

double InputBox::getDouble() {

  double ret_val;

  char tmparr[buffer_.size()];

  for (unsigned long i = 0; i < buffer_.size(); i++) {
    tmparr[i] = buffer_[i];
  }

  char *ptr;

  ret_val = strtod(tmparr, &ptr);

  return ret_val;
}

//}
