#include <menu.h>

/* Menu() //{ */

Menu::Menu(int begin_y, int begin_x, std::vector<std::string>& text) {

  unsigned long longest_string = 0;

  for (unsigned long i = 0; i < text.size(); i++) {
    if (text[i].length() > longest_string) {
      longest_string = text[i].length();
    }
  }

  win_ = newwin(text.size() + 2, longest_string + 2, begin_y, begin_x);
}

//}

/* Iterate() //{ */

// return tuple - int selected_menu_line, int pressed key
//
std::optional<std::tuple<int, int>> Menu::Iterate(std::vector<std::string>& text, int key) {

  std::optional<std::tuple<int, int>> ret_val = std::nullopt;

  wclear(win_);  // update the terminal screen

  if (key == 'q' || key == KEY_ESC) {
    ret_val = std::make_tuple(666, 666);
    return ret_val;
  }

  box(win_, '*', '*');

  for (unsigned long j = 0; j < text.size(); j++) {

    mvwaddstr(win_, j + 1, 1, text[j].c_str());
  }

  // use a variable to increment or decrement the value based on the input.
  if (key == KEY_UP || key == 'k') {

    i--;
    i = (i < 0) ? text.size() - 1 : i;

  } else if (key == KEY_DOWN || key == 'j') {

    i++;
    i = (i > int(text.size() - 1)) ? 0 : i;

  } else {
    ret_val = std::make_tuple(i, key);
  }

  // now highlight the next item in the list.
  wattron(win_, A_STANDOUT);
  mvwaddstr(win_, i + 1, 1, text[i].c_str());
  wattroff(win_, A_STANDOUT);

  wrefresh(win_);  // update the terminal screen

  return ret_val;
}

//}

