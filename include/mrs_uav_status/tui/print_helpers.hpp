#pragma once

/* includes //{ */

#include <ncurses.h>
#include <string>
#include <cmath>
#include <vector>

#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

//}

namespace mrs_uav_status::tui
{

/* printLimitedInt() //{ */

// Prints num with the given printf-style format string, switching to "%.0e"-style scientific
// notation if it exceeds limit so it doesn't overflow its field width.
inline void printLimitedInt(WINDOW *win, int y, int x, const std::string &str_in, int num, int limit) {
  std::string str_out = str_in;
  if (std::abs(num) > limit) {
    for (unsigned long i = 0; i < str_out.length() - 2; i++) {
      if (str_out[i] == '.' && str_out[i + 2] == 'i') {
        str_out[i + 1] = '0';
        str_out[i + 2] = 'e';
        break;
      }
    }
  }
  mvwprintw(win, y, x, str_out.c_str(), num);
}

//}

/* printLimitedDouble() //{ */

// Same as printLimitedInt(), for a double.
inline void printLimitedDouble(WINDOW *win, int y, int x, const std::string &str_in, double num, double limit) {
  std::string format_str = str_in;
  if (std::abs(num) > limit) {
    for (unsigned long i = 0; i < format_str.length() - 2; i++) {
      if (format_str[i] == '.' && format_str[i + 2] == 'f') {
        format_str[i + 1] = '0';
        format_str[i + 2] = 'e';
        break;
      }
    }
  }
  mvwprintw(win, y, x, format_str.c_str(), num);
}

//}

/* printLimitedString() //{ */

// Prints str_in, truncated to limit characters.
inline void printLimitedString(WINDOW *win, int y, int x, const std::string &str_in, unsigned long limit) {
  if (str_in.length() > limit) {
    std::string truncated_str = str_in.substr(0, limit);
    mvwprintw(win, y, x, "%s", truncated_str.c_str());
  } else {
    mvwprintw(win, y, x, "%s", str_in.c_str());
  }
}

//}

/* wrapText() //{ */

// Word-wraps str_in into lines of at most width chars. Pure, so callers can measure a row cost
// before printing anything.
inline std::vector<std::string> wrapText(const std::string &str_in, unsigned long width) {
  std::vector<std::string> lines;
  std::string              remaining = str_in;

  while (!remaining.empty()) {
    if (remaining.length() <= width) {
      lines.push_back(remaining);
      break;
    }

    std::size_t break_pos = remaining.rfind(' ', width);
    if (break_pos == std::string::npos || break_pos == 0) {
      break_pos = width; // no space to break at -- hard-break the word itself
    }

    lines.push_back(remaining.substr(0, break_pos));

    std::size_t next_start = remaining.find_first_not_of(' ', break_pos);
    remaining              = (next_start == std::string::npos) ? "" : remaining.substr(next_start);
  }

  return lines;
}

//}

/* capWrappedLines() //{ */

// Wraps str_in via wrapText(str_in, width) and keeps at most max_lines lines, marking truncation
// with "..." on the last kept line. Bounds a single list entry's row cost so one verbose string
// can't crowd out the rest of a row-budget-capped list (see renderCappedList()).
inline std::vector<std::string> capWrappedLines(const std::string &str_in, unsigned long width, int max_lines) {
  std::vector<std::string> lines = wrapText(str_in, width);
  if (lines.size() > static_cast<std::size_t>(max_lines)) {
    lines.resize(static_cast<std::size_t>(max_lines));
    std::string &last = lines.back();
    if (last.size() > 3) {
      last.resize(last.size() - 3);
    }
    last += "...";
  }
  return lines;
}

//}

/* printWrappedString() //{ */

// Prints wrapText(str_in, width) starting at (y, x), one line per row, stopping past max_row.
// Returns the row after the last line printed.
inline int printWrappedString(WINDOW *win, int y, int x, const std::string &str_in, unsigned long width, int max_row) {
  int row = y;
  for (const auto &line : wrapText(str_in, width)) {
    if (row > max_row) {
      break;
    }
    mvwprintw(win, row, x, "%s", line.c_str());
    ++row;
  }
  return row;
}

//}

/* fitCountWithOverflow() //{ */

// Given each item's row cost and how many rows are available, returns how many leading items to
// print. If they all fit, returns item_rows.size(). Otherwise reserves 1 row for a "+N more"
// indicator -- an item that would only partially fit isn't shown at all, it's folded into the count.
inline std::size_t fitCountWithOverflow(const std::vector<int> &item_rows, int budget) {
  int total = 0;
  for (int r : item_rows) {
    total += r;
  }
  if (total <= budget) {
    return item_rows.size();
  }

  std::size_t shown = 0;
  int         used  = 0;
  for (int r : item_rows) {
    if (used + r > budget - 1) {
      break;
    }
    used += r;
    ++shown;
  }
  return shown;
}

//}

/* renderCappedList() //{ */

// Shared skeleton behind every row-budget-capped pane list: cost each item via row_cost(i), print
// as many as fit via print_item(row, i, max_row), and add a "+N more" line if some didn't fit.
template <typename RowCost, typename PrintItem>
inline int renderCappedList(WINDOW *win, int row, int max_row, std::size_t count, RowCost row_cost, PrintItem print_item, unsigned long more_width) {
  std::vector<int> item_rows;
  item_rows.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    item_rows.push_back(row_cost(i));
  }
  const std::size_t shown = fitCountWithOverflow(item_rows, max_row - row + 1);

  for (std::size_t i = 0; i < shown; ++i) {
    row = print_item(row, i, max_row);
  }

  if (shown < count && row <= max_row) {
    printLimitedString(win, row++, 1, "+" + std::to_string(count - shown) + " more", more_width);
  }

  return row;
}

//}

/* printNoData(WINDOW *win, int y, int x, bool mini) //{ */

// Prints a blinking red "NO DATA" marker at (y, x).
inline void printNoData(WINDOW *win, int y, int x, [[maybe_unused]] bool mini) {
  // Save/restore instead of wattroff(): ncurses attributes aren't stacked, so a blind
  // wattroff() would clobber a color the caller turned on around this call.
  attr_t saved_attrs;
  short  saved_pair;
  wattr_get(win, &saved_attrs, &saved_pair, nullptr);

  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwprintw(win, y, x, "NO DATA");

  wattr_set(win, saved_attrs, saved_pair, nullptr);
}

//}

/* printNoData(WINDOW *win, int y, int x, const std::string &text, bool mini) //{ */

// Same as above, with a label printed before the "NO DATA" marker.
inline void printNoData(WINDOW *win, int y, int x, const std::string &text, bool mini) {
  attr_t saved_attrs;
  short  saved_pair;
  wattr_get(win, &saved_attrs, &saved_pair, nullptr);

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwprintw(win, y, x, text.c_str());

  wattr_set(win, saved_attrs, saved_pair, nullptr);
  printNoData(win, y, x + static_cast<int>(text.length()), mini);
}

//}

/* printBox() //{ */

// Draws the window's border, blinking red if avoiding a collision or the bumper is active,
// or yellow if the drone can't take off and the null tracker is engaged.
inline void printBox(WINDOW *win, bool avoiding_collision, bool bumper_active, bool can_takeoff, bool null_tracker) {
  if (avoiding_collision || bumper_active) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    wattron(win, A_STANDOUT);
  }
  if (!can_takeoff && null_tracker) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
    wattron(win, A_STANDOUT);
  }
  box(win, 0, 0);
  wattroff(win, A_BLINK);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wattroff(win, A_STANDOUT);
}

//}

/* printServiceResult() //{ */

// Prints a service call's result line, green on success and red on failure.
inline void printServiceResult(WINDOW *win, bool light, bool success, const std::string &msg) {
  if (light) {
    wattron(win, A_STANDOUT);
  }
  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  if (success) {
    printLimitedString(win, 0, 0, "Service call success: " + msg, 120);
  } else {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    printLimitedString(win, 0, 0, "Service call failed: " + msg, 120);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  wattroff(win, A_BOLD);
}

//}

/* printHelp() //{ */

// Prints the full help text if help_active, otherwise just the "press 'h' for help" hint.
inline void printHelp(WINDOW *win, bool help_active) {
  werase(win);
  if (help_active) {
    printLimitedString(win, 1, 0, "How to use mrs_status:", 120);
    printLimitedString(win, 2, 0, "Press the 'm' key to enter a services menu", 120);
    printLimitedString(win, 3, 0, "Press the 'g' key to set a goto reference", 120);
    printLimitedString(win, 4, 0, "Press the 'M' key to switch into minimalistic mode, which takes less screen space", 120);
    printLimitedString(win, 5, 0, "Press the 'R' key to enter 'remote' mode to take direct control of the uav with your keyboad", 120);
    printLimitedString(win, 6, 0, "   In remote mode, use these keys to control the drone:", 120);
    printLimitedString(win, 7, 0, "      'w','s','a','d' to control pitch and roll ('h','j','k','l' works too)", 120);
    printLimitedString(win, 8, 0, "      'q','e'         to control heading", 120);
    printLimitedString(win, 9, 0, "      'r','f'         to control altitude", 120);
    printLimitedString(win, 10, 0, "      'G'             to switch controlling in the FCU frame (local) or the world frame (global)", 120);
    printLimitedString(win, 12, 0, "You can also display any info from your node in the mrs_status:", 120);
    printLimitedString(win, 14, 0, "   topic: mrs_status/display_string (std_msgs::String)", 120);
    printLimitedString(win, 15, 0, "   - Publish any string to this topic and it will show up in mrs_status", 120);
    printLimitedString(win, 17, 0, "Press 'D' to display info from other panes of this tmux session, up to 2 panes can be viewed", 120);
    printLimitedString(win, 18, 0, "Press 'p' to cycle the top-right pane through various system info (sensors, node CPU usage, GNSS & strings, problems)",
                       120);
    printLimitedString(win, 19, 0, "Press '1'-'9' to directly select a pane ", 120);

    printLimitedString(win, 21, 0, "Press 'h' to hide help", 120);
  } else {
    printLimitedString(win, 1, 0, "Press 'h' key for help", 120);
  }
  wnoutrefresh(win);
}

//}

/* printRemoteHelp() //{ */

// Shown instead of printHelp() while in Remote mode, where 'h' is a flight command, not a help toggle.
inline void printRemoteHelp(WINDOW *win) {
  werase(win);
  printLimitedString(win, 1, 0, "REMOTE MODE ACTIVE -- press 'R' or Esc to exit", 120);
  printLimitedString(win, 2, 0, "   'w','s','a','d' to control pitch and roll ('h','j','k','l' works too)", 120);
  printLimitedString(win, 3, 0, "   'q','e'         to control heading", 120);
  printLimitedString(win, 4, 0, "   'r','f'         to control altitude", 120);
  printLimitedString(win, 5, 0, "   'G'             to switch controlling in the FCU frame (local) or the world frame (global)", 120);
  wnoutrefresh(win);
}

//}

/* printTmuxDump() //{ */

// Captures up to 2 selected tmux windows via the tmux CLI and prints them into sub1/sub2,
// with each pane's display_menu_text label along the border.
inline void printTmuxDump(WINDOW *debug_window, WINDOW *sub1, WINDOW *sub2, const std::vector<int> &selected, const std::string &session_name,
                          const std::vector<std::string> &display_menu_text, int max_windows, bool avoiding_collision, bool bumper_active, bool can_takeoff,
                          bool null_tracker) {
  werase(debug_window);
  printBox(debug_window, avoiding_collision, bumper_active, can_takeoff, null_tracker);

  if (static_cast<int>(selected.size()) > max_windows) {
    return;
  }

  int tmp_cols, tmp_rows;
  getmaxyx(sub1, tmp_rows, tmp_cols);

  for (size_t i = 0; i < selected.size(); i++) {
    std::string command_str = "tmux resize-window -t " + session_name + ":" + std::to_string(selected[i]) + " -A";
    mrs_uav_status::utils::callTerminal(command_str.c_str());
    command_str          = "tmux capture-pane -pt " + session_name + ":" + std::to_string(selected[i]) + " -S 0 | tail -n " + std::to_string(tmp_rows + 1);
    std::string response = mrs_uav_status::utils::callTerminal(command_str.c_str());
    switch (i) {
    case 0:
      mvwaddstr(sub1, 0, 0, response.c_str());
      break;
    case 1:
      mvwaddstr(sub2, 0, 0, response.c_str());
      break;
    }
  }

  mvwhline(debug_window, tmp_rows + 1, 1, 0, tmp_cols - 1);

  // A persisted index may no longer be valid if the tmux window count changed since it was saved.
  if (selected.size() > 1 && selected[1] >= 0 && static_cast<size_t>(selected[1]) < display_menu_text.size()) {
    printLimitedString(debug_window, tmp_rows + 1, 3, display_menu_text[selected[1]], 50);
  }
  if (!selected.empty() && selected[0] >= 0 && static_cast<size_t>(selected[0]) < display_menu_text.size()) {
    printLimitedString(debug_window, 0, 3, display_menu_text[selected[0]], 50);
  }

  wnoutrefresh(debug_window);
  touchwin(debug_window);
  wnoutrefresh(sub1);
  wnoutrefresh(sub2);
}

//}

/* rateColor() //{ */

// Map a measured Hz against an expected Hz to a ColorPair (Green ≥ 90% expected,
// Yellow ≥ 50% expected, Red otherwise).
inline int16_t rateColor(double rate, double expected) {
  if (expected <= 0.0) {
    return static_cast<int16_t>(ColorPair::Normal);
  }
  if (rate >= 0.9 * expected) {
    return static_cast<int16_t>(ColorPair::Green);
  }
  if (rate >= 0.5 * expected) {
    return static_cast<int16_t>(ColorPair::Yellow);
  }
  return static_cast<int16_t>(ColorPair::Red);
}

//}

/* printHotkey() //{ */

// btop-style hotkey hint: render word with its first character (the trigger
// key) in red+bold and the remainder in the terminal's default colour, never
// in standout, so it looks the same in both colorschemes. Returns the column
// just past the word (plus one space) so hints can be chained left-to-right.
inline int printHotkey(WINDOW *win, int y, int x, const std::string &word) {
  if (word.empty()) {
    return x;
  }
  attr_t saved_attrs;
  short  saved_pair;
  wattr_get(win, &saved_attrs, &saved_pair, nullptr);

  wattroff(win, A_STANDOUT);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  mvwaddch(win, y, x, static_cast<chtype>(word.front()));
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  mvwaddstr(win, y, x + 1, word.substr(1).c_str());

  wattr_set(win, saved_attrs, saved_pair, nullptr);

  return x + static_cast<int>(word.size()) + 1; // +1 for a single-space gap
}

//}

} // namespace mrs_uav_status::tui
