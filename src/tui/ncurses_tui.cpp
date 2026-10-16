/* includes //{ */

#include <mrs_uav_status/tui/ncurses_tui.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <mrs_lib/geometry/cyclic.h>

//}

namespace mrs_uav_status::tui
{

using radians = mrs_lib::geometry::radians;

// tui::Key hardcodes ncurses' KEY_* values so UavStatusCore can use them without <curses.h>.
static_assert(static_cast<int>(Key::Up) == KEY_UP, "tui::Key::Up drifted from ncurses KEY_UP");
static_assert(static_cast<int>(Key::Down) == KEY_DOWN, "tui::Key::Down drifted from ncurses KEY_DOWN");
static_assert(static_cast<int>(Key::Left) == KEY_LEFT, "tui::Key::Left drifted from ncurses KEY_LEFT");
static_assert(static_cast<int>(Key::Right) == KEY_RIGHT, "tui::Key::Right drifted from ncurses KEY_RIGHT");
static_assert(static_cast<int>(Key::Delete) == KEY_DC, "tui::Key::Delete drifted from ncurses KEY_DC");


/* NcursesTui() //{ */

NcursesTui::NcursesTui(const NcursesTui::Params &params) : params_(params) {

  light_scheme_ = (params_.colorscheme.find("COLORSCHEME_LIGHT") != std::string::npos);
}

//}

// | --------------------- Lifecycle & window setup --------------------- |

/* initTerminal() //{ */

void NcursesTui::initTerminal() {
  initscr();
  start_color();
  cbreak();
  noecho();
  clear();
  nodelay(stdscr, true);
  keypad(stdscr, true);
  timeout(0);
  curs_set(0);
  set_escdelay(0);
  use_default_colors();
  attron(A_BOLD);
}

//}

/* shutdownTerminal() //{ */

void NcursesTui::shutdownTerminal() {
  endwin();
}

//}

/* pollKey() //{ */

int NcursesTui::pollKey() {
  return getch();
}

//}

/* flushInput() //{ */

void NcursesTui::flushInput() {
  flushinp();
}

//}

/* commitFrame() //{ */

void NcursesTui::commitFrame() {
  doupdate();
}

//}

/* setSnapshot() //{ */

void NcursesTui::setSnapshot(const status::RenderSnapshot &snapshot) {
  snapshot_ = snapshot;
}

//}

/* tickSlowCounter() //{ */

void NcursesTui::tickSlowCounter() {
  increment_counter_ = !increment_counter_;
  estimator_display_counter_ += int(increment_counter_);
  if (estimator_display_counter_ >= 3) {
    estimator_display_counter_ = 0;
  }
}

//}

/* updateTermSize() //{ */

bool NcursesTui::updateTermSize() {

  bool changed = false;

  std::string command  = "tmux list-panes -F '#{pane_width}x#{pane_height}'";
  std::string response = utils::callTerminal(command.c_str());

  std::vector<std::string> results;

  results = mrs_uav_status::utils::splitByChar(response, 'x');

  int cols, lines;

  try {
    cols  = std::stoi(results[0]);
    lines = std::stoi(results[1]);
  }

  catch (const std::invalid_argument &e) {
    cols  = 0;
    lines = 0;
  }

  if (terminal_cols_ != cols || terminal_lines_ != lines) {
    terminal_lines_ = lines;
    terminal_cols_  = cols;
    changed         = true;
  }

  return (changed);
}

//}

/* setupWindows() //{ */

void NcursesTui::setupWindows() {

  std::string command = "tmux display-message -p '#S'";
  session_name_       = utils::callTerminal(command.c_str());
  session_name_.erase(std::remove(session_name_.begin(), session_name_.end(), '\n'), session_name_.end());

  // derwin() children must be delwin()'d before their parent debug_window_ is
  // replaced/destroyed, so reset them first in both branches.
  sub_tmux_window_1_.reset();
  sub_tmux_window_2_.reset();

  if (params_.start_minimized) {
    control_manager_window_.reset(newwin(4, 9, 1, 1));
    uav_state_window_.reset(newwin(6, 9, 5, 1));
    top_bar_window_.reset(newwin(1, 140, 0, 1));
    general_info_window_.reset(newwin(4, 9, 1, 10));
    hw_api_state_window_.reset(newwin(6, 9, 5, 10));
    debug_window_.reset(newwin(terminal_lines_ - 15, terminal_cols_ - 1, 13, 1));
    pane_window_.reset();
    bottom_window_.reset(newwin(1, 120, 11, 1));

  } else {

    uav_state_window_.reset(newwin(7, 26, 5, 1));
    control_manager_window_.reset(newwin(4, 26, 1, 1));
    hw_api_state_window_.reset(newwin(7, 25, 5, 27));
    general_info_window_.reset(newwin(4, 25, 1, 27));
    top_bar_window_.reset(newwin(1, 140, 0, 1));
    bottom_window_.reset(newwin(1, 120, 12, 1));
    pane_window_.reset(newwin(11, 82, 1, 52));
    const int debug_height = std::max(3, terminal_lines_ - 13);
    debug_window_.reset(newwin(debug_height, terminal_cols_ - 1, 13, 1));
    const int half_lines = std::max(1, (debug_height - 2) / 2);
    sub_tmux_window_1_.reset(derwin(debug_window_.get(), half_lines, terminal_cols_ - 3, 1, 1));
    sub_tmux_window_2_.reset(derwin(debug_window_.get(), half_lines, terminal_cols_ - 3, half_lines + 2, 1));
  }

  clear();
  refresh();
  light_scheme_ = tui::setupColors(params_.colorscheme, params_.colorblind_mode);
}

//}

/* resize() //{ */

bool NcursesTui::resize() {
  if (!updateTermSize()) {
    return false;
  }
  if (terminal_cols_ > 30) {
    resize_term(terminal_lines_, terminal_cols_);
    setupWindows();
    return true;
  }
  return false;
}

//}

/* toggleMini() //{ */

void NcursesTui::toggleMini() {
  params_.start_minimized = !params_.start_minimized;
}

//}

/* toggleHelp() //{ */

void NcursesTui::toggleHelp() {
  help_active_ = !help_active_;
}

//}

/* setRemoteMode() //{ */

void NcursesTui::setRemoteMode(bool in_remote_mode) {
  in_remote_mode_ = in_remote_mode;
}

//}

/* refreshTopBar() //{ */

void NcursesTui::refreshTopBar() {
  wnoutrefresh(top_bar_window_.get());
}

//}

/* refreshBottomWindow() //{ */

void NcursesTui::refreshBottomWindow() {
  wnoutrefresh(bottom_window_.get());
}

//}

/* refreshAfterMenu() //{ */

void NcursesTui::refreshAfterMenu() {
  wnoutrefresh(debug_window_.get());
  wnoutrefresh(bottom_window_.get());
}

//}

// | --------------------- Render handlers --------------------- |

/* renderFast() //{ */

void NcursesTui::renderFast() {
  topLineHandler();
  renderTmuxOrHelp();
  uavStateHandler();
}

//}

/* renderSlow() //{ */

void NcursesTui::renderSlow() {
  tickSlowCounter();
  hwApiStateHandler();
  controlManagerHandler();
  paneHandler();
  generalInfoHandler();
}

//}

/* generalInfoHandler() //{ */

void NcursesTui::generalInfoHandler() {
  WINDOW *win = general_info_window_.get();
  werase(win);
  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
  wattroff(win, A_STANDOUT);

  const status::BorderStatus bs = snapshot_.border_status;

  double cpu_load, cpu_ghz, free_ram, total_ram;
  int    free_hdd;
  bool   have_system_health_info;
  {
    const auto &oc          = snapshot_.system_health_info.onboard_computer_info;
    cpu_load                = oc.cpu_load;
    cpu_ghz                 = oc.cpu_ghz;
    free_ram                = oc.free_ram;
    total_ram               = oc.total_ram;
    free_hdd                = oc.free_hdd;
    have_system_health_info = snapshot_.freshness.system_health_info;
  }

  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  if (!have_system_health_info) {
    // Fields default to -1/-1.0 and would otherwise render as a healthy-looking 0% CPU/RAM.
    printNoData(win, 1, 1, params_.start_minimized);
  } else {
    printCpuLoad(win, cpu_load, params_.start_minimized);
    printMemLoad(win, free_ram, total_ram, params_.start_minimized);
    if (!params_.start_minimized) {
      printCpuFreq(win, cpu_ghz);
    }
    printDiskSpace(win, free_hdd, params_.start_minimized);
  }

  wnoutrefresh(win);
}

//}

/* cyclePanes() //{ */

void NcursesTui::cyclePanes() {
  pane_box_.cycle();
}

//}

/* selectPane() //{ */

void NcursesTui::selectPane(std::size_t idx) {
  pane_box_.select(idx);
}

//}

/* paneHandler() //{ */

void NcursesTui::paneHandler() {
  pane_box_.render(pane_window_.get(), snapshot_, light_scheme_, params_.start_minimized);
}

//}

/* uavStateHandler() //{ */

void NcursesTui::uavStateHandler() {
  WINDOW     *win = uav_state_window_.get();
  double      avg_rate, color, heading;
  double      state_x, state_y, state_z;
  double      cmd_x, cmd_y, cmd_z, cmd_hdg;
  std::string odom_frame, main_estimator, horizontal_estimator, vertical_estimator, heading_estimator, agl_estimator;
  double      max_flight_z;
  bool        null_tracker, have_control_info, have_state_estimation_info;

  const status::BorderStatus bs = snapshot_.border_status;

  {
    const auto &est            = snapshot_.state_estimation_info;
    have_state_estimation_info = snapshot_.freshness.state_estimation_info;
    avg_rate                   = snapshot_.system_health_info.state_estimation_rate;
    heading                    = est.heading;
    state_x                    = est.pos_x;
    state_y                    = est.pos_y;
    state_z                    = est.pos_z;
    odom_frame                 = est.frame_id;

    cmd_x   = snapshot_.control_info.cmd_pose_x;
    cmd_y   = snapshot_.control_info.cmd_pose_y;
    cmd_z   = snapshot_.control_info.cmd_pose_z;
    cmd_hdg = snapshot_.control_info.cmd_pose_heading;

    // DiagnosticsManager's default for current_estimator is the literal string "unknown", not empty.
    main_estimator       = (est.current_estimator.empty() || est.current_estimator == "unknown") ? std::string("NONE") : est.current_estimator;
    horizontal_estimator = est.horizontal_estimator;
    vertical_estimator   = est.vertical_estimator;
    heading_estimator    = est.heading_estimator;
    agl_estimator        = est.agl_estimator;

    max_flight_z = est.max_flight_z;
    null_tracker = bs.null_tracker;
    // "unknown" means no TrackerCommand yet, so cmd_pose is still (0,0,0).
    have_control_info = (snapshot_.control_info.active_tracker != "unknown");
  }
  // Nominal MRS estimation rate is 100 Hz; threshold the color band off that.
  color = rateColor(avg_rate, 100.0);

  double cerr_x   = std::fabs(state_x - cmd_x);
  double cerr_y   = std::fabs(state_y - cmd_y);
  double cerr_z   = std::fabs(state_z - cmd_z);
  double cerr_hdg = std::fabs(radians::diff(heading, cmd_hdg));

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Odm %3.0f", avg_rate, 1000);

    if (avg_rate <= 0.0 || !have_state_estimation_info) {

      printNoData(win, 0, 1, params_.start_minimized);

    } else {

      printLimitedDouble(win, 1, 1, "%4.0f", state_x, 1000);
      printLimitedDouble(win, 2, 1, "%4.0f", state_y, 1000);
      printLimitedDouble(win, 3, 1, "%4.0f", state_z, 1000);
      printLimitedDouble(win, 4, 1, "%4.1f", heading, 1000);

      printLimitedString(win, 1, 6, main_estimator, 2);
    }
  }

  else {

    if (avg_rate <= 0.0 || !have_state_estimation_info) {

      // Showing a healthy Hz next to NO DATA reads as contradictory -- suppress it too.
      printNoData(win, 0, 12, "Odom ", params_.start_minimized);
      printNoData(win, 1, 1, params_.start_minimized); // Inside the box; rows 1-4 are otherwise blank.

    } else {

      printLimitedDouble(win, 0, 12, "Odom %5.1f Hz", avg_rate, 1000);

      // Position/heading/estimator names reset together upstream, so NaN position is a reliable proxy.
      const bool estimation_data_valid = !std::isnan(state_x);

      if (estimation_data_valid) {
        printLimitedDouble(win, 1, 1, "X %7.2f", state_x, 1000);
        printLimitedDouble(win, 2, 1, "Y %7.2f", state_y, 1000);
        printLimitedDouble(win, 3, 1, "Z %7.2f", state_z, 1000);
        printLimitedDouble(win, 4, 1, "hdg %5.2f", heading, 1000);
      } else {
        printNoData(win, 1, 1, "X ", params_.start_minimized);
        printNoData(win, 2, 1, "Y ", params_.start_minimized);
        printNoData(win, 3, 1, "Z ", params_.start_minimized);
        // "hdg " leaves only a 5-char field before the estimator name column -- too narrow for NO DATA.
        printLimitedString(win, 4, 1, "hdg ERR", 7);
      }

      if (!null_tracker && have_control_info) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 5, 1, "C/E");

        if (cerr_x < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_x < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 5, "X%1.1f", cerr_x, 10);


        if (cerr_y < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_y < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 10, "Y%1.1f", cerr_y, 10);

        if (cerr_z < 0.5) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_z < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 15, "Z%1.1f", cerr_z, 10);

        if (cerr_hdg < 0.2) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        } else if (cerr_hdg < 0.4) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
        printLimitedDouble(win, 5, 20, "H%1.1f", cerr_hdg, 10);

        wattron(win, COLOR_PAIR(color));
      }

      if (!estimation_data_valid) {
        printLimitedString(win, 1, 11, "NO DATA", 14);
        printLimitedString(win, 2, 11, "NO DATA", 14);
        printLimitedString(win, 4, 11, "NO DATA", 14);
      } else {
        printLimitedString(win, 1, 11, main_estimator, 14);

        switch (estimator_display_counter_) {
        case 0:
          printLimitedString(win, 2, 11, "hor: " + horizontal_estimator, 14);
          break;
        case 1:
          printLimitedString(win, 2, 11, "ver: " + vertical_estimator, 14);
          break;
        case 2:
          printLimitedString(win, 2, 11, "hdg: " + heading_estimator, 14);
          break;
        }

        printLimitedString(win, 4, 11, "ag: " + agl_estimator, 14);
      }

      if (max_flight_z < 0.0) {
        // Negative means EstimationDiagnostics was never received, not that we're near the ceiling.
        printNoData(win, 3, 11, "Max: ", params_.start_minimized);

      } else {
        double dist_to_max_z = max_flight_z - state_z;
        if (dist_to_max_z < 0.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
          wattron(win, A_BLINK);
        } else if (dist_to_max_z < 0.3) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        } else if (dist_to_max_z < 1.0) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        }

        printLimitedDouble(win, 3, 11, "Max: %5.1f", max_flight_z, 1000);
        wattron(win, COLOR_PAIR(color));
        wattroff(win, A_BLINK);
      }
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

//}

/* controlManagerHandler() //{ */

void NcursesTui::controlManagerHandler() {
  WINDOW *win = control_manager_window_.get();

  int16_t     color;
  bool        null_tracker, have_system_health_info, have_control_info, have_uav_state;
  double      rate;
  std::string curr_controller, curr_tracker, curr_gains, curr_constraints;
  bool        callbacks_enabled, rc_mode, have_goal, tracking_trajectory;

  const status::BorderStatus bs = snapshot_.border_status;

  {
    const auto &ci = snapshot_.control_info;

    rate                    = snapshot_.system_health_info.control_manager_rate;
    have_system_health_info = snapshot_.freshness.system_health_info;
    have_control_info       = snapshot_.freshness.control_info;
    have_uav_state          = snapshot_.freshness.uav_state;

    // "unknown" self-corrects only while DiagnosticsManager stays alive; !have_system_health_info
    // catches it dying entirely, which would otherwise freeze these at their last real names.
    curr_controller =
        (!have_system_health_info || ci.active_controller.empty() || ci.active_controller == "unknown") ? std::string("NO DATA") : ci.active_controller;
    curr_tracker = (!have_system_health_info || ci.active_tracker.empty() || ci.active_tracker == "unknown") ? std::string("NO DATA") : ci.active_tracker;
    curr_gains   = (!have_system_health_info || ci.active_gains.empty() || ci.active_gains == "unknown") ? std::string("NO DATA") : ci.active_gains;
    curr_constraints =
        (!have_system_health_info || ci.active_constraints.empty() || ci.active_constraints == "unknown") ? std::string("NO DATA") : ci.active_constraints;

    callbacks_enabled   = ci.callbacks_enabled;
    rc_mode             = (snapshot_.uav_state.state == status::STATE_RC_MODE);
    have_goal           = ci.have_goal;
    tracking_trajectory = ci.tracking_trajectory;
    null_tracker        = bs.null_tracker;
  }
  // Nominal MRS control_manager diagnostics rate is 10 Hz.
  color = rateColor(rate, 10.0);

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Ctr %3.0f", rate, 1000);

    if (rate <= 0.0 || !have_system_health_info || !have_control_info) {

      printNoData(win, 0, 1, params_.start_minimized);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 1, "ERR");
      mvwprintw(win, 2, 1, "ERR");
      wattroff(win, COLOR_PAIR(color));

    } else {

      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        printLimitedString(win, 1, 1, curr_controller, 3);
      } else {
        printLimitedString(win, 1, 1, curr_controller, 3);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 1, 4, "%s", "/");
      }

      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NlT";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != static_cast<int>(ColorPair::Red)) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }

        printLimitedString(win, 2, 1, curr_tracker, 3);

      } else {
        printLimitedString(win, 2, 1, curr_tracker, 3);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        mvwprintw(win, 2, 4, "%s", "/");
        wattron(win, COLOR_PAIR(color));
      }

      printLimitedString(win, 1, 5, curr_gains, 3);
      printLimitedString(win, 2, 5, curr_constraints, 3);
    }
  }

  else {

    if (rate <= 0.0 || !have_system_health_info || !have_control_info) {

      // Showing a healthy Hz next to NO DATA reads as contradictory -- suppress it too.
      printNoData(win, 0, 1, "Control ", params_.start_minimized);

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwprintw(win, 1, 1, "NO_CONTROLLER");
      mvwprintw(win, 2, 1, "NO_TRACKER");
      wattroff(win, COLOR_PAIR(color));

    } else {
      printLimitedDouble(win, 0, 1, "Control Manager %5.1f Hz", rate, 1000);

      if (curr_controller != "Se3Controller" && curr_controller != "MpcController") {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }
      printLimitedString(win, 1, 1, curr_controller, 13);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
      printLimitedString(win, 1, 1 + std::min(int(curr_controller.length()), 13), "/" + curr_gains, 10);
      wattron(win, COLOR_PAIR(color));

      if (null_tracker) {
        curr_tracker = "NullTracker";
      }

      if (curr_tracker != "MpcTracker") {
        if (curr_tracker == "LandoffTracker" && color != static_cast<int>(ColorPair::Red)) {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        } else {
          wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        }
      }

      printLimitedString(win, 2, 1, curr_tracker, 13);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
      printLimitedString(win, 2, 1 + std::min(int(curr_tracker.length()), 13), "/" + curr_constraints, 8);
      wattron(win, COLOR_PAIR(color));

      // uav_state has its own freshness (unlike control_info's callbacks_enabled/have_goal/
      // tracking_trajectory below, which are already covered by the outer !have_control_info gate).
      if (have_uav_state && rc_mode) {
        wattron(win, A_BLINK);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        mvwprintw(win, 1, 18, "RC_MODE");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattroff(win, A_BLINK);

      } else if (!callbacks_enabled) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        mvwprintw(win, 1, 20, "NO_CB");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      if (tracking_trajectory) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        mvwprintw(win, 2, 21, "TRAJ");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      } else if (have_goal) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
        mvwprintw(win, 2, 21, "GOTO");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      } else {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
        mvwprintw(win, 2, 21, "IDLE");
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* hwApiStateHandler() //{ */

void NcursesTui::hwApiStateHandler() {
  WINDOW     *win = hw_api_state_window_.get();
  int16_t     color;
  double      hw_api_rate, cmd_rate;
  bool        gnss_ok, armed, have_uav_info, autopilot_ok, have_system_health_info, have_general_robot_info;
  std::string mode;
  double      battery_volt, battery_curr, battery_wh_drained;
  double      throttle, mass_estimate, mass_set, gnss_qual, mag_norm, mag_norm_rate;

  const status::BorderStatus bs = snapshot_.border_status;

  {
    const auto &bat = snapshot_.general_robot_info.battery_state;

    hw_api_rate             = snapshot_.system_health_info.hw_api_rate;
    cmd_rate                = snapshot_.system_health_info.control_manager_rate;
    have_uav_info           = snapshot_.freshness.uav_info;
    have_system_health_info = snapshot_.freshness.system_health_info;
    have_general_robot_info = snapshot_.freshness.general_robot_info;

    // have_system_health_info catches a frozen sensor list from before DiagnosticsManager died.
    gnss_ok = false;
    if (const auto *gnss = utils::findSensor(snapshot_.system_health_info.available_sensors, status::SENSOR_TYPE_GNSS); gnss) {
      gnss_ok   = have_system_health_info && (gnss->level == status::SENSOR_STATUS_OK);
      gnss_qual = utils::parseDoubleOr(utils::lookupDetail(gnss->details, "quality"), 0.0);
    }

    // Catches HwApiStatus (armed/offboard) going stale on its own; have_uav_info alone can't, since
    // DiagnosticsManager keeps republishing UavInfo. Defaults true if the AUTOPILOT handler isn't configured.
    autopilot_ok = true;
    if (const auto *autopilot = utils::findSensor(snapshot_.system_health_info.available_sensors, status::SENSOR_TYPE_AUTOPILOT); autopilot) {
      autopilot_ok = (autopilot->level == status::SENSOR_STATUS_OK);
    }

    mag_norm      = 0.0;
    mag_norm_rate = 0.0;
    if (const auto *mag = utils::findSensor(snapshot_.system_health_info.available_sensors, status::SENSOR_TYPE_MAGNETOMETER); mag) {
      mag_norm      = utils::parseDoubleOr(utils::lookupDetail(mag->details, "norm_gauss"), 0.0);
      mag_norm_rate = have_system_health_info ? mag->rate : 0.0;
    }

    armed = snapshot_.uav_info.armed;
    mode  = snapshot_.uav_info.flight_state;
    // Forced to the same -1.0 sentinel used below; a frozen GeneralRobotInfo wouldn't reset it on its own.
    battery_volt       = have_general_robot_info ? bat.voltage : -1.0;
    battery_curr       = bat.current;
    battery_wh_drained = bat.wh_drained;
    throttle           = snapshot_.control_info.throttle;
    mass_estimate      = snapshot_.uav_info.mass_estimate;
    mass_set           = snapshot_.uav_info.mass_nominal;
  }
  // Nominal MRS hw_api rate is 100 Hz.
  color = rateColor(hw_api_rate, 100.0);

  std::string tmp_string;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }


  wattron(win, COLOR_PAIR(color));

  if (params_.start_minimized) {
    printLimitedDouble(win, 0, 1, "Mav %3.0f", hw_api_rate, 1000);
    wattroff(win, COLOR_PAIR(color));

    if (hw_api_rate <= 0.0 || !have_system_health_info) {
      printNoData(win, 0, 1, params_.start_minimized);
    }

    if (!have_uav_info || !autopilot_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 1, "ERR", 3);
      printLimitedString(win, 2, 1, "ERR", 3);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      if (armed) {
        tmp_string = "ARM";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      } else {
        tmp_string = "DIS";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 1, 1, tmp_string, 15);
      wattron(win, COLOR_PAIR(color));

      if (utils::isFlightStateUnhealthy(mode)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 2, 1, mode, 3);
      wattron(win, COLOR_PAIR(color));
    }

    // battery_volt defaults to a negative sentinel whenever BatteryState is stale or was never
    // received (DiagnosticsManager nulls it out via not_reporting_timeout_), so this alone is sufficient.
    if (battery_volt < 0.0) {

      printLimitedString(win, 3, 1, "ERR", 3);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (battery_volt < 3.7 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedString(win, 3, 1, "Bat", 3);
    }


    if (cmd_rate <= 0.0 || throttle < 0.0 || !have_system_health_info) {

      printLimitedString(win, 3, 5, "ERR", 3);

    } else {

      if (throttle > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (throttle > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 5, ".%2.0f", throttle * 100, 100);
      wattron(win, COLOR_PAIR(color));
    }

    // mass_nominal/mass_estimate come from UavInfo; !have_uav_info catches a frozen topic that the
    // sentinel check below can't (values stop updating but stay non-negative).
    if (!have_uav_info || mass_set < 0.0) {

      printNoData(win, 4, 1, params_.start_minimized);

    } else if (mass_estimate < 0.0) {

      // No room here for a "?/" prefix like the full-mode display -- blink red instead.
      wattron(win, A_BLINK);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedDouble(win, 4, 1, "%4.1f kg", mass_set, 99.99);
      wattroff(win, A_BLINK);

    } else {

      color = static_cast<int>(ColorPair::Green);

      double mass_diff = std::fabs(mass_estimate - mass_set) / mass_set;

      if (mass_diff > 0.3) {

        color = static_cast<int>(ColorPair::Red);

      } else if (mass_diff > 0.2) {

        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));
      printLimitedDouble(win, 4, 1, "%4.1f kg", mass_estimate, 99.99);
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 5, "GNSS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 5, "GNSS", 6);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      color = static_cast<int>(ColorPair::Red);

      if (gnss_qual < 5.0) {
        color = static_cast<int>(ColorPair::Green);
      } else if (gnss_qual < 10.0) {
        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));

      if (gnss_qual < 10.0) {
        printLimitedDouble(win, 2, 5, "%3.1f", gnss_qual, 9.9);
      } else {
        printLimitedString(win, 2, 5, ">10", 3);
      }
      wattroff(win, COLOR_PAIR(color));
    }

  }

  else {

    if (hw_api_rate <= 0.0 || !have_system_health_info) {

      // Showing a healthy Hz next to NO DATA reads as contradictory -- suppress it too.
      // (State/Mode/Mag/Batt/Thrust below already show their own NO DATA, no extra marker needed.)
      printNoData(win, 0, 9, "HW Api ", params_.start_minimized);
      wattroff(win, COLOR_PAIR(color));

    } else {

      printLimitedDouble(win, 0, 9, "HW Api %5.1f Hz", hw_api_rate, 1000);
      wattroff(win, COLOR_PAIR(color));
    }

    if (!have_uav_info || !autopilot_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 1, "State: ", 15);
      printNoData(win, 1, 9, params_.start_minimized);
      printLimitedString(win, 2, 1, "Mode: ", 15);
      printNoData(win, 2, 9, params_.start_minimized);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      if (armed) {
        tmp_string = "ARMED";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      } else {
        tmp_string = "DISARMED";
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 1, 1, "State: " + tmp_string, 15);
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (utils::isFlightStateUnhealthy(mode)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      }

      printLimitedString(win, 2, 1, "Mode:  " + mode, 15);
      wattron(win, COLOR_PAIR(color));
    }

    if (battery_volt < 0.0) {

      printNoData(win, 4, 1, "Batt:  ", params_.start_minimized);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      (battery_volt > 17.0) ? (battery_volt = battery_volt / 6) : (battery_volt = battery_volt / 4);

      if (battery_volt < 3.6) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (battery_volt < 3.7 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 4, 1, "%4.2fV ", battery_volt, 10);
      printLimitedDouble(win, 4, 8, "%5.2fA", battery_curr, 100);
      printLimitedDouble(win, 4, 16, "%6.1fWh", battery_wh_drained, 9999.9);
    }

    if (mag_norm_rate <= 0.0) {

      printNoData(win, 3, 1, "Mag:  ", params_.start_minimized);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (mag_norm > 0.9 || mag_norm < 0.25) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (mag_norm > 0.65) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 3, 1, "Mag: %4.2f", mag_norm, 9.99);
    }

    if (cmd_rate <= 0.0 || throttle < 0.0 || !have_system_health_info) {

      printNoData(win, 5, 1, "Thrtl: ", params_.start_minimized);

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      if (throttle > 0.75) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      } else if (throttle > 0.65 && color != static_cast<int>(ColorPair::Red)) {
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
      }
      printLimitedDouble(win, 5, 1, "Thrtl: %4.2f", throttle, 1.01);
      wattron(win, COLOR_PAIR(color));
    }

    if (!have_uav_info || mass_set < 0.0) {

      // x=17 clears "Thrtl: NO DATA" (cols 1-14) when both blocks are missing at once.
      printNoData(win, 5, 17, params_.start_minimized);

    } else {

      // mass_set always starts flush at the fixed x=18 (can't push further right -- window's
      // only 25 cols wide). A wide estimate could in theory touch "Thrtl: NO DATA", but throttle
      // and mass_estimate share the same active control loop, so that combination can't occur.
      constexpr int SET_X = 18;

      if (mass_estimate < 0.0) {

        wattron(win, A_BLINK);
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        printLimitedString(win, 5, SET_X - 2, "?", 1);
        wattroff(win, A_BLINK);
        wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
        wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
        printLimitedString(win, 5, SET_X - 1, "/", 1);

      } else {

        color            = static_cast<int>(ColorPair::Green);
        double mass_diff = std::fabs(mass_estimate - mass_set) / mass_set;

        if (mass_diff > 0.3) {

          color = static_cast<int>(ColorPair::Red);

        } else if (mass_diff > 0.2) {

          color = static_cast<int>(ColorPair::Yellow);
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << mass_estimate << "/";
        std::string lead_str = oss.str();

        wattron(win, COLOR_PAIR(color));
        printLimitedString(win, 5, SET_X - static_cast<int>(lead_str.length()), lead_str, lead_str.length());
      }

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Normal)));
      printLimitedDouble(win, 5, SET_X, "%.1f", mass_set, 99.99);
      printLimitedString(win, 5, SET_X + 4, "kg", 2);
    }

    if (!gnss_ok) {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      printLimitedString(win, 1, 17, "NO_GNSS", 7);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

    } else {

      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      printLimitedString(win, 1, 17, "GNSS_OK", 7);
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

      color = static_cast<int>(ColorPair::Red);

      if (gnss_qual < 5.0) {
        color = static_cast<int>(ColorPair::Green);
      } else if (gnss_qual < 10.0) {
        color = static_cast<int>(ColorPair::Yellow);
      }

      wattron(win, COLOR_PAIR(color));
      printLimitedDouble(win, 2, 17, "Q: %4.1f", gnss_qual, 99.9);
      wattroff(win, COLOR_PAIR(color));
    }
  }

  wattroff(win, COLOR_PAIR(color));
  wattroff(win, A_BOLD);

  wnoutrefresh(win);
}

//}

/* topLineHandler() //{ */

void NcursesTui::topLineHandler() {
  WINDOW *win = top_bar_window_.get();
  werase(win);

  std::string uav_name, uav_type;
  bool        collision_avoidance_enabled, avoiding_collision, bumper_active;
  uint16_t    num_other_uavs;
  int         secs_flown;
  bool        have_general_robot_info, have_collision_avoidance_info, have_uav_info;

  {
    uav_name                      = snapshot_.general_robot_info.robot_name;
    uav_type                      = utils::robotTypeToString(snapshot_.general_robot_info.robot_type);
    collision_avoidance_enabled   = snapshot_.collision_avoidance_info.collision_avoidance_enabled;
    avoiding_collision            = snapshot_.collision_avoidance_info.avoiding_collision;
    bumper_active                 = snapshot_.collision_avoidance_info.bumper_active;
    num_other_uavs                = static_cast<uint16_t>(snapshot_.collision_avoidance_info.num_other_robots_visible);
    secs_flown                    = static_cast<int>(std::max(0.0f, snapshot_.uav_info.flight_duration));
    have_general_robot_info       = snapshot_.freshness.general_robot_info;
    have_collision_avoidance_info = snapshot_.freshness.collision_avoidance_info;
    have_uav_info                 = snapshot_.freshness.uav_info;
  }

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  wattron(win, A_BOLD);

  const int status_x = params_.start_minimized ? 27 : 26;
  const int alert_x  = params_.start_minimized ? 22 : 26;
  const int count_x  = params_.start_minimized ? 31 : 51;
  const int uavs_x   = params_.start_minimized ? -1 : 45;

  // x=10 fits the common case ("uav1 - DRONE") without truncation; still capped against status_x.
  if (have_general_robot_info) {
    const std::string name_type = " " + uav_name + " - " + uav_type + " ";
    printLimitedString(win, 0, 10, name_type, static_cast<unsigned long>(std::min(status_x, alert_x) - 11));
  } else {
    printNoData(win, 0, 10, params_.start_minimized);
  }

  const char *disabled_text = params_.start_minimized ? "C/A" : "COL AVOID DISABLED";
  const char *avoiding_text = params_.start_minimized ? "!AVOIDING!" : "!! AVOIDING COLLISION !!";
  const char *bumper_text   = params_.start_minimized ? "!BUMPER!" : "!! BUMPER ACTIVE !!";
  const char *enabled_text  = params_.start_minimized ? "C/A" : "COL AVOID ENABLED,";

  if (!have_collision_avoidance_info) {
    printNoData(win, 0, status_x, params_.start_minimized ? "C/A " : "COL AVOID ", params_.start_minimized);

  } else if (!collision_avoidance_enabled) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    mvwprintw(win, 0, status_x, "%s", disabled_text);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else if (bumper_active) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    mvwprintw(win, 0, alert_x, "%s", bumper_text);
    wattroff(win, A_BLINK);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else if (avoiding_collision) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    wattron(win, A_BLINK);
    mvwprintw(win, 0, alert_x, "%s", avoiding_text);
    wattroff(win, A_BLINK);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  } else {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    mvwprintw(win, 0, status_x, "%s", enabled_text);

    if (!params_.start_minimized) {
      mvwprintw(win, 0, uavs_x, "UAVs: ");
    }

    if (num_other_uavs == 0) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }

    printLimitedInt(win, 0, count_x, "%i", num_other_uavs, 100);

    if (num_other_uavs == 0) {
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }

    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  }

  int mins = secs_flown / 60;
  int secs = secs_flown % 60;

  if (have_uav_info) {
    mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  } else {
    // "ToF: NO DATA" (12 chars) would overlap the uav_name/type field starting at x=10.
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    mvwprintw(win, 0, 0, "ToF: ");
    wattron(win, A_BLINK);
    mvwprintw(win, 0, 5, "ERR");
    wattroff(win, A_BLINK);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  }
  wattroff(win, A_BOLD);

  // btop-style hotkey hints on the right of the top bar — the trigger key (the
  // red letter) maps directly to the STANDARD-mode key handler in uav_status.cpp.
  if (!params_.start_minimized && !in_remote_mode_) {
    int hx = 62;
    hx     = printHotkey(win, 0, hx, "menu");
    hx     = printHotkey(win, 0, hx, "goto");
    hx     = printHotkey(win, 0, hx, "Remote");
    hx     = printHotkey(win, 0, hx, "Mini");
    hx     = printHotkey(win, 0, hx, "Display");
    hx     = printHotkey(win, 0, hx, "pane");
    hx     = printHotkey(win, 0, hx, "help");
  }

  wnoutrefresh(win);
}

//}

// | --------------------- Bottom-window helpers --------------- |

/* blankBottomWindow() //{ */

void NcursesTui::blankBottomWindow() {
  if (snapshot_.now_seconds - bottom_window_clear_time_s_ > 3.0) {
    werase(bottom_window_.get());
  }
}

//}

/* renderServiceResult() //{ */

void NcursesTui::renderServiceResult(bool success, const std::string &message, double now_seconds) {
  printServiceResult(bottom_window_.get(), light_scheme_, success, message);
  // Stamped from the caller's post-call wall clock, not the stale tick-start snapshot_.now_seconds.
  bottom_window_clear_time_s_ = now_seconds;
}

//}

// | --------------------- Menu helpers ----------------------- |

/* isValidMenuIndex() //{ */

bool NcursesTui::isValidMenuIndex(int index, size_t container_size) {
  return index >= 0 && static_cast<size_t>(index) < container_size;
}

//}

/* clearMenus() //{ */

void NcursesTui::clearMenus() {
  menu_vec_.clear();
  submenu_vec_.clear();
}

//}

/* createSubMenu() //{ */

void NcursesTui::createSubMenu(std::vector<std::string> &submenu_entries) {
  submenu_vec_.clear();
  if (!submenu_entries.empty()) {

    int                  x;
    int                  y;
    [[maybe_unused]] int rows;
    int                  cols;

    getyx(menu_vec_[0].getWin(), x, y);
    getmaxyx(menu_vec_[0].getWin(), rows, cols);

    StatusWindow menu(x, 31 + cols, submenu_entries);
    submenu_vec_.push_back(menu);
  }
}

//}

// | --------------------- Main menu ----------------------- |

/* showMainMenu() //{ */

void NcursesTui::showMainMenu(const std::vector<std::string> &labels) {
  main_menu_text_ = labels;
  submenu_vec_.clear();
  StatusWindow menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* showSubMenu() //{ */

void NcursesTui::showSubMenu(const std::vector<std::string> &labels) {
  std::vector<std::string> entries = labels;
  createSubMenu(entries);
}

//}

/* closeSubMenu() //{ */

void NcursesTui::closeSubMenu() {
  submenu_vec_.clear();
}

//}

/* handleMainMenuKey() //{ */

MenuEvent NcursesTui::handleMainMenuKey(int key) {
  MenuEvent event;

  if (menu_vec_.empty()) {
    return event;
  }

  if (!submenu_vec_.empty()) {
    event.in_submenu = true;

    // Redraw the main menu underneath (key -1 == navigate nothing) so it doesn't go stale.
    menu_vec_[0].iterate(main_menu_text_, -1, true);

    const auto result = submenu_vec_[0].iterate(key, true);

    if (result.action == StatusWindow::Result::Action::Exit) {
      event.kind = MenuEvent::Kind::Exit;
      return event;
    }
    if (result.pressed_key == static_cast<int>(Key::Enter)) {
      event.kind  = MenuEvent::Kind::Selected;
      event.index = result.selected_line;
    }
    return event;
  }

  const auto result = menu_vec_[0].iterate(main_menu_text_, key, true);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    submenu_vec_.clear();
    event.kind = MenuEvent::Kind::Exit;
    return event;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter)) {
    event.kind  = MenuEvent::Kind::Selected;
    event.index = result.selected_line;
  }
  return event;
}

//}

// | --------------------- Goto menu ----------------------- |

/* showGotoMenu() //{ */

void NcursesTui::showGotoMenu(const std::vector<std::string> &labels, const std::vector<double> &initial_values) {
  goto_menu_inputs_.clear();
  goto_menu_text_ = labels;

  StatusWindow menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);

  for (std::size_t i = 0; i < 4; i++) {
    const double seed = (i < initial_values.size()) ? initial_values[i] : 0.0;
    goto_menu_inputs_.push_back(ControlBar(8, menu.getWin(), seed));
  }
}

//}

/* handleGotoMenuKey() //{ */

GotoEvent NcursesTui::handleGotoMenuKey(int key) {
  GotoEvent event;

  if (menu_vec_.empty()) {
    return event;
  }

  const auto result = menu_vec_[0].iterate(goto_menu_text_, key, false);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    event.kind = GotoEvent::Kind::Exit;
    return event;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter)) {
    event.kind    = GotoEvent::Kind::Committed;
    event.x       = goto_menu_inputs_[0].getDouble();
    event.y       = goto_menu_inputs_[1].getDouble();
    event.z       = goto_menu_inputs_[2].getDouble();
    event.heading = goto_menu_inputs_[3].getDouble();
    menu_vec_.clear();
    return event;

  } else if (isValidMenuIndex(result.selected_line, goto_menu_inputs_.size())) {

    goto_menu_inputs_[result.selected_line].process(result.pressed_key);
  }

  for (size_t i = 0; i < goto_menu_inputs_.size(); i++) {
    goto_menu_inputs_[i].print(i + 1, int(i) == menu_vec_[0].getLine());
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return event;
}

//}

// | --------------------- Display menu ----------------------- |

/* setupDisplayText() //{ */

void NcursesTui::setupDisplayText() {
  display_menu_text_.clear();

  char                     command[50] = "tmux list-windows | cut -d' ' -f-2";
  std::string              response    = utils::callTerminal(command);
  std::vector<std::string> results     = utils::splitByChar(response, '\n');

  const bool skip_last = !results.empty() && results.back().empty();
  const auto end_index = skip_last ? results.size() - 1 : results.size();
  for (size_t i = 0; i < end_index; i++) {
    display_menu_text_.push_back("[ ] " + results[i]);
  }

  for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
    // A persisted index may no longer be valid if the tmux window count changed since it was saved.
    if (selected_tmux_window_[i] >= 0 && static_cast<size_t>(selected_tmux_window_[i]) < display_menu_text_.size()) {
      display_menu_text_[selected_tmux_window_[i]][1] = '*';
    }
  }
}

//}

/* setupDisplayMenu() //{ */

void NcursesTui::setupDisplayMenu() {
  setupDisplayText();

  StatusWindow menu(1, 32, display_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* displayMenuHandler() //{ */

bool NcursesTui::displayMenuHandler(int key) {

  auto result = menu_vec_[0].iterate(display_menu_text_, key, false);

  if (result.action == StatusWindow::Result::Action::Exit) {
    menu_vec_.clear();
    return true;
  }

  if (result.pressed_key == static_cast<int>(Key::Enter)) {

    auto it = std::find(selected_tmux_window_.begin(), selected_tmux_window_.end(), result.selected_line);

    if (it != selected_tmux_window_.end()) {
      display_menu_text_[result.selected_line][1] = ' ';
      selected_tmux_window_.erase(it);
    } else if (int(selected_tmux_window_.size()) < MAX_SELECTED_TMUX_WINDOWS) {
      display_menu_text_[result.selected_line][1] = '*';
      selected_tmux_window_.push_back(result.selected_line);
    }

    std::ofstream outputFile(params_.display_config_filename, std::ofstream::out | std::ofstream::trunc);

    for (size_t i = 0; i < selected_tmux_window_.size(); i++) {
      outputFile << selected_tmux_window_[i] << '\n';
    }
    outputFile.close();
  }

  wnoutrefresh(menu_vec_[0].getWin());
  return false;
}

//}

/* loadDisplayConfig() //{ */

void NcursesTui::loadDisplayConfig() {
  if (!std::filesystem::exists(params_.display_config_filename)) {
    return;
  }

  selected_tmux_window_.clear();

  std::ifstream file(params_.display_config_filename);
  std::string   line;

  for (int i = 0; i < MAX_SELECTED_TMUX_WINDOWS; i++) {
    std::getline(file, line);
    try {
      selected_tmux_window_.push_back(std::stoi(line));
    }
    catch (const std::invalid_argument &e) {
    }
    catch (const std::out_of_range &e) {
    }
  }

  file.close();

  setupDisplayText();
}

//}

// | -------------------------- Remote -------------------------- |

/* renderRemoteBanner() //{ */

void NcursesTui::renderRemoteBanner(bool turbo, bool global) {
  WINDOW *win = top_bar_window_.get();

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  const int rem_x   = params_.start_minimized ? 33 : 62;
  const int mode_x  = params_.start_minimized ? 37 : 82;
  const int turbo_x = params_.start_minimized ? 39 : 74;

  const char *rem_text   = params_.start_minimized ? "REM" : "REMOTE MODE";
  const char *mode_text  = global ? (params_.start_minimized ? "G" : "GLOBAL MODE") : (params_.start_minimized ? "L" : "LOCAL MODE");
  const char *turbo_text = params_.start_minimized ? "!T!" : "!TURBO!";

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  mvwprintw(win, 0, rem_x, "%s", rem_text);
  mvwprintw(win, 0, mode_x, "%s", mode_text);

  if (turbo) {
    wattron(win, A_BLINK);
    mvwprintw(win, 0, turbo_x, "%s", turbo_text);
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  wattroff(win, A_BOLD);
}

//}

// | --------------------- Bottom debug window --------------------- |

/* renderTmuxOrHelp() //{ */

void NcursesTui::renderTmuxOrHelp() {
  if (params_.start_minimized) {
    return;
  }

  WINDOW *debug_window = debug_window_.get();
  WINDOW *sub1         = sub_tmux_window_1_.get();
  WINDOW *sub2         = sub_tmux_window_2_.get();

  // Remote mode overrides tmux dump / help entirely -- 'h' doesn't toggle help while in remote mode.
  if (in_remote_mode_) {
    printRemoteHelp(debug_window);
    return;
  }

  // The bottom region (y=13+) is now exclusively the help / tmux-dump overlay —
  // the pane panel moved up into the top-right, so there's no contention.
  if (!selected_tmux_window_.empty()) {
    const status::BorderStatus bs = snapshot_.border_status;
    printTmuxDump(debug_window, sub1, sub2, selected_tmux_window_, session_name_, display_menu_text_, MAX_SELECTED_TMUX_WINDOWS, bs.avoiding_collision,
                  bs.bumper_active, bs.can_takeoff, bs.null_tracker);
  } else {
    printHelp(debug_window, help_active_);
  }
}

//}

} // namespace mrs_uav_status::tui
