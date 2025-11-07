#include <menu.h>

/* Menu() //{ */

Menu::Menu(int begin_y, int begin_x, std::vector<std::string> &text) {

  unsigned long longest_string = 0;

  for (unsigned long line = 0; line < text.size(); line++) {
    if (text[line].length() > longest_string) {
      longest_string = text[line].length();
    }
  }
  id_   = 0;
  text_ = text;
  win_  = newwin(text_.size() + 2, longest_string + 2, begin_y, begin_x);
}

Menu::Menu(int begin_y, int begin_x, std::vector<std::string> &text, int id) {

  unsigned long longest_string = 0;

  for (unsigned long line = 0; line < text.size(); line++) {
    if (text[line].length() > longest_string) {
      longest_string = text[line].length();
    }
  }
  id_   = id;
  text_ = text;
  win_  = newwin(text_.size() + 2, longest_string + 2, begin_y, begin_x);
}

//}

/* getWin() //{ */

WINDOW *Menu::getWin() {
  return win_;
}

//}

/* getId() //{ */

int Menu::getId() {
  return id_;
}

//}

/* getLine() //{ */

int Menu::getLine() {
  return line;
}

//}

/* iterate() //{ */

// return tuple - int selected_menu_line, int pressed key
//
std::optional<std::tuple<int, int>> Menu::iterate(int key, bool refresh) {

  std::optional<std::tuple<int, int>> ret_val = std::nullopt;

  wattron(win_, A_BOLD);

  if (key == 'q' || key == KEY_ESC) {
    ret_val = std::make_tuple(666, 666);
    wattroff(win_, A_BOLD);
    return ret_val;
  }

  wattron(win_, COLOR_PAIR(GREEN));
  box(win_, 0, 0);
  wattroff(win_, COLOR_PAIR(GREEN));

  for (unsigned long j = 0; j < text_.size(); j++) {

    mvwaddstr(win_, j + 1, 1, text_[j].c_str());
  }

  // use a variable to increment or decrement the value based on the input.
  if (key == KEY_UP || key == 'k') {

    line--;
    line = (line < 0) ? text_.size() - 1 : line;

  } else if (key == KEY_DOWN || key == 'j') {

    line++;
    line = (line > int(text_.size() - 1)) ? 0 : line;

  } else {
    ret_val = std::make_tuple(line, key);
  }

  // now highlight the next item in the list.
  wattron(win_, A_STANDOUT);
  mvwaddstr(win_, line + 1, 1, text_[line].c_str());
  wattroff(win_, A_STANDOUT);

  if (refresh) {
    wrefresh(win_); // update the terminal screen
  }

  wattroff(win_, A_BOLD);
  return ret_val;
}

//}

/* iterate() //{ */

// return tuple - int selected_menu_line, int pressed key
//
std::optional<std::tuple<int, int>> Menu::iterate(std::vector<std::string> &text, int key, bool refresh) {

  std::optional<std::tuple<int, int>> ret_val = std::nullopt;

  wattron(win_, A_BOLD);

  if (key == 'q' || key == KEY_ESC) {
    ret_val = std::make_tuple(666, 666);
    wattroff(win_, A_BOLD);
    return ret_val;
  }

  wattron(win_, COLOR_PAIR(GREEN));
  box(win_, 0, 0);
  wattroff(win_, COLOR_PAIR(GREEN));

  for (unsigned long j = 0; j < text.size(); j++) {

    mvwaddstr(win_, j + 1, 1, text[j].c_str());
  }

  // use a variable to increment or decrement the value based on the input.
  if (key == KEY_UP || key == 'k') {

    line--;
    line = (line < 0) ? text.size() - 1 : line;

  } else if (key == KEY_DOWN || key == 'j') {

    line++;
    line = (line > int(text.size() - 1)) ? 0 : line;

  } else {
    ret_val = std::make_tuple(line, key);
  }

  // now highlight the next item in the list.
  wattron(win_, A_STANDOUT);
  mvwaddstr(win_, line + 1, 1, text[line].c_str());
  wattroff(win_, A_STANDOUT);

  if (refresh) {
    wrefresh(win_); // update the terminal screen
  }

  wattroff(win_, A_BOLD);
  return ret_val;
}

//}
