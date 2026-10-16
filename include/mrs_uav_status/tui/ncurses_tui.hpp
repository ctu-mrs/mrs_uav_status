#pragma once

/* includes //{ */

#include <string>
#include <vector>
#include <functional>
#include <memory>

// | --------------------- Internal Package Includes --------------------- |
#include <mrs_uav_status/tui/system_info.hpp>
#include <mrs_uav_status/tui/control_bar.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/status_window.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/colors.hpp>
#include <mrs_uav_status/tui/pane_box.hpp>

// <curses.h> (transitively included above by the NcursesTui helpers) #defines OK as 0 -- undo that
// here so it can't collide with an OK enumerator/identifier used by this header's includers.
#ifdef OK
#undef OK
#endif

#include <mrs_uav_status/utils/helpers.hpp>
#include <mrs_uav_status/utils/terminal.hpp>

#include <mrs_uav_status/status/data_types.hpp>
#include <mrs_uav_status/tui/tui_actions.hpp>

//}

namespace mrs_uav_status::tui
{

// Owns the ncurses windows and renders them from the status::RenderSnapshot pushed in once per
// tick by UavStatus. It stores no message state of its own -- UavStatusCore owns that.
class NcursesTui : public TuiActions {
public:
  struct Params
  {
    std::string colorscheme;
    bool        colorblind_mode;
    bool        start_minimized;
    std::string display_config_filename;
  };

  // Stores params_ and derives light_scheme_ from it; pane_box_ default-constructs itself.
  NcursesTui(const NcursesTui::Params &params);

  // Puts the terminal into ncurses raw/no-echo mode. Call once, before constructing any NcursesTui
  // and before any window is created.
  static void initTerminal();
  // Restores the terminal to normal mode. Call after the NcursesTui instance (and its windows) is
  // destroyed.
  static void shutdownTerminal();

  // Wraps getch(): reads one keypress (non-blocking, per initTerminal()'s nodelay(true)).
  int pollKey();
  // Wraps flushinp(): discards any buffered keypresses.
  void flushInput() override;
  // Wraps doupdate(): flushes all pending ncurses window updates to the physical screen.
  void commitFrame();

  // Installs the tick's render input. Called once per fast tick by UavStatus, before any
  // render*/handler call. snapshot_ is render-thread-private, so no locking is needed here or
  // in any handler that reads it.
  void setSnapshot(const status::RenderSnapshot &snapshot);

  // | --------------------- Window lifecycle ------------------- |
  // (Re)creates all ncurses windows, sized/positioned for the current minimized/full layout, and
  // reapplies the colorscheme.
  void setupWindows() override;
  // If the terminal size changed and is wide enough, resizes the ncurses term and calls setupWindows(). Returns whether it resized.
  bool resize();
  // Queries tmux for the current pane size; returns whether it changed since the last call.
  bool updateTermSize();
  // Flips between minimized and full layout (next setupWindows() picks it up).
  void toggleMini() override;
  // Flips the help overlay on/off.
  void toggleHelp() override;

  /* isMini() //{ */

  bool isMini() const {
    return params_.start_minimized;
  }

  //}

  void refreshTopBar();
  void setRemoteMode(bool in_remote_mode) override;

  // | --------------------- Window Handlers -------------------- |
  // Redraws the windows that need to update every tick: top bar, tmux/help overlay, UAV state box.
  void renderFast() override;
  // Redraws the throttled windows (called at update_rate_slow): counters/pruning, HW API, control manager, pane, general info.
  void renderSlow() override;
  // Draws position/heading, commanded-vs-estimated error, and current estimator names.
  void uavStateHandler();
  // Draws armed state, flight mode, battery, throttle, mass estimate, and GNSS/magnetometer readouts.
  void hwApiStateHandler();
  // Draws CPU load/frequency, RAM, and disk space.
  void generalInfoHandler();
  // Renders the currently selected cycleable pane, auto-switching to one whose wants_focus() just turned true.
  void paneHandler();
  // Draws the active controller/tracker/gains/constraints and callback/goal state.
  void controlManagerHandler();
  // Draws the top status bar: UAV name/type, collision-avoidance state, time-of-flight, hotkey hints.
  void topLineHandler();

  // Cycle the preset panel to the next preset (bound to the 'p' key).
  void cyclePanes() override;
  // Jump the preset panel directly to preset idx (bound to number keys). No-op if out of range.
  void selectPane(std::size_t idx) override;
  // Advances the 3-way rotation used to cycle which estimator name (hor/ver/hdg) uavStateHandler() shows.
  void tickSlowCounter();

  // | --------------------- Bottom-window helpers --------------- |
  // Clears the bottom window 3s after the last renderServiceResult(), so results don't linger forever.
  void blankBottomWindow();
  void refreshBottomWindow() override;
  // Prints a service call's success/failure message and marks the clear-time for blankBottomWindow().
  void renderServiceResult(bool success, const std::string &message, double now_seconds) override;

  // | ------------------- Menu (public entry) ------------------- |
  // Creates the main-menu window listing labels.
  void showMainMenu(const std::vector<std::string> &labels) override;
  // Creates the submenu window next to the main menu, listing labels.
  void showSubMenu(const std::vector<std::string> &labels) override;
  // Destroys the submenu window, leaving the main menu up.
  void closeSubMenu() override;
  // Drives main-menu/submenu navigation and reports the resulting MenuEvent.
  MenuEvent handleMainMenuKey(int key) override;
  // Creates the goto window (labels) and its 4 numeric fields (initial_values).
  void showGotoMenu(const std::vector<std::string> &labels, const std::vector<double> &initial_values) override;
  // Drives the 4 numeric input boxes; on Enter parses all of them into the returned GotoEvent.
  GotoEvent handleGotoMenuKey(int key) override;
  // Builds the tmux-window picker menu from the current tmux window list.
  void setupDisplayMenu() override;
  // Toggles a tmux window's selection (max MAX_SELECTED_TMUX_WINDOWS) and persists the choice to disk. Returns true when done.
  bool displayMenuHandler(int key) override;
  void clearMenus() override;
  // Reads the persisted tmux window selection from display_config_filename, if it exists.
  void loadDisplayConfig();
  // Draws the bottom debug window: remote-mode help, the selected tmux panes, or the keybinding help.
  void renderTmuxOrHelp();
  void refreshAfterMenu() override;

  // | -------------------------- Remote -------------------------- |
  // Draws the REMOTE/LOCAL-GLOBAL/TURBO banner over the top bar.
  void renderRemoteBanner(bool turbo, bool global) override;

private:
  // Everything the handlers render from, refreshed once per fast tick by setSnapshot().
  status::RenderSnapshot snapshot_;

  // The cycleable top-right box ('p' / number keys). Owns its own pane list.
  PaneBox pane_box_;

  Params params_;
  bool   light_scheme_   = false;
  bool   help_active_    = false;
  bool   in_remote_mode_ = false;

  // | ------------------- Menu (private helpers) --------------- |
  static bool isValidMenuIndex(int index, size_t container_size);
  // Creates the submenu window next to the main menu, listing submenu_entries.
  void createSubMenu(std::vector<std::string> &submenu_entries);

  // | ---------------------- TMUX & Misc ----------------------- |
  std::vector<int> selected_tmux_window_;
  std::string      session_name_;
  const int        MAX_SELECTED_TMUX_WINDOWS = 2;
  int              terminal_cols_ = 0, terminal_lines_ = 0;

  // Rebuilds display_menu_text_ from tmux's current window list, marking previously-selected windows.
  void setupDisplayText();

  int    estimator_display_counter_  = 0;
  bool   increment_counter_          = false;
  double bottom_window_clear_time_s_ = 0.0;

  // | ---------------------- Window Pointers ------------------- |
  // RAII wrapper for ncurses windows — delwin() called on destruction.
  struct WindowDeleter
  {
    /* operator() //{ */

    void operator()(WINDOW *w) const noexcept {
      if (w)
        delwin(w);
    }

    //}
  };

  using WindowPtr = std::unique_ptr<WINDOW, WindowDeleter>;

  WindowPtr uav_state_window_;
  WindowPtr control_manager_window_;
  WindowPtr hw_api_state_window_;
  WindowPtr top_bar_window_;
  WindowPtr bottom_window_;
  WindowPtr pane_window_; // cycleable panes (top-right): system detail / node CPU / GNSS / problems
  WindowPtr general_info_window_;
  WindowPtr debug_window_;
  WindowPtr sub_tmux_window_1_;
  WindowPtr sub_tmux_window_2_;

  // | ----------------------- Data Storage --------------------- |
  std::vector<tui::StatusWindow> menu_vec_;
  std::vector<tui::StatusWindow> submenu_vec_;
  std::vector<tui::ControlBar>   goto_menu_inputs_;

  // Backing label text for the widgets above.
  std::vector<std::string> main_menu_text_;
  std::vector<std::string> display_menu_text_;
  std::vector<std::string> goto_menu_text_;
};

} // namespace mrs_uav_status::tui
