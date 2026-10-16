#pragma once

/* includes //{ */

#include <cstddef>
#include <string>
#include <vector>

//}

namespace mrs_uav_status::tui
{

// One main-menu/submenu interaction, as reported by NcursesTui after it has handled a keypress. NcursesTui
// translates raw ncurses navigation into this; UavStatusCore decides what the selected row means.
struct MenuEvent
{
  enum class Kind
  {
    None,     // navigation only (arrows/j/k) -- nothing decided yet
    Selected, // Enter pressed on `index`
    Exit      // 'q'/Escape -- back out one level
  };

  Kind kind       = Kind::None;
  bool in_submenu = false; // which list `index` refers to / which level Exit backs out of
  int  index      = -1;
};

// One goto-menu interaction. On Kind::Committed, NcursesTui has already read and parsed all four
// ControlBars, so UavStatusCore only ever sees plain doubles.
struct GotoEvent
{
  enum class Kind
  {
    None,      // still editing
    Committed, // Enter pressed; x/y/z/heading are valid
    Exit       // 'q'/Escape
  };

  Kind   kind    = Kind::None;
  double x       = 0.0;
  double y       = 0.0;
  double z       = 0.0;
  double heading = 0.0;
};

// The subset of NcursesTui's public API UavStatusCore drives. NcursesTui implements this; tests substitute a
// fake instead of constructing a real (ncurses-backed) NcursesTui.
class TuiActions {
public:
  virtual ~TuiActions() = default;

  // | ----------------------- Main menu ------------------------ |
  // Creates the main-menu window listing labels (one row each).
  virtual void showMainMenu(const std::vector<std::string> &labels) = 0;
  // Creates the submenu window next to the main menu, listing labels.
  virtual void showSubMenu(const std::vector<std::string> &labels) = 0;
  // Destroys the submenu window, leaving the main menu up.
  virtual void closeSubMenu() = 0;
  // Applies one keypress to whichever menu level is on top and reports what happened.
  virtual MenuEvent handleMainMenuKey(int key) = 0;

  // | ------------------------- Goto --------------------------- |
  // Creates the goto window: labels are its text rows, initial_values seeds the 4 numeric fields.
  virtual void showGotoMenu(const std::vector<std::string> &labels, const std::vector<double> &initial_values) = 0;
  // Applies one keypress to the goto window/fields and reports what happened.
  virtual GotoEvent handleGotoMenuKey(int key) = 0;

  // | ----------------------- Display menu --------------------- |
  // Not migrated: tmux-window selection is local UI state, it calls no service.
  virtual void setupDisplayMenu()          = 0;
  virtual bool displayMenuHandler(int key) = 0;

  // | --------------------- Menu (shared) ---------------------- |
  // Common to every menu level (main/sub/goto/display), not just the one above.
  virtual void clearMenus()       = 0;
  virtual void refreshAfterMenu() = 0;

  // | -------------------------- Remote ------------------------ |
  // Latches whether the remote-mode overlay/banner should be drawn at all.
  virtual void setRemoteMode(bool in_remote_mode) = 0;
  // Draws the REMOTE / LOCAL-GLOBAL / TURBO banner over the top bar.
  virtual void renderRemoteBanner(bool turbo, bool global) = 0;

  // | -------------------------- Misc -------------------------- |
  // Prints a service call's success/failure message in the bottom window. now_seconds is the wall
  // clock read *after* the (blocking) service call returned -- the caller-side tick timestamp is
  // too stale to time the message's 3 s hold correctly.
  virtual void renderServiceResult(bool success, const std::string &message, double now_seconds) = 0;

  virtual void toggleHelp()                = 0;
  virtual void cyclePanes()                = 0;
  virtual void selectPane(std::size_t idx) = 0;
  virtual void toggleMini()                = 0;
  virtual void setupWindows()              = 0;
  virtual void renderFast()                = 0;
  virtual void renderSlow()                = 0;
  virtual void flushInput()                = 0;
  virtual void refreshBottomWindow()       = 0;
};

} // namespace mrs_uav_status::tui
