#pragma once

/* includes //{ */

#include <ncurses.h>

#include <functional>
#include <string>
#include <vector>

#include <mrs_uav_status/status/data_types.hpp>

//}

namespace mrs_uav_status::tui
{

// The cycleable top-right box. Holds a list of pluggable panes and renders whichever one is
// selected into the window it is handed. To add a pane, push one in PaneBox::PaneBox(): give it a
// title, a render callback that calls drawPaneChrome(win) itself (for the box + tab bar) before
// drawing its own content rows, and optionally a wants_focus() predicate to auto-switch to it
// when it has something important to show.
class PaneBox {
public:
  // Populates the pane list with the 4 built-in panes (Sensors, ROS Node CPU, GNSS & Strings,
  // Problems & Errors).
  PaneBox();

  // panes_ holds std::functions (including a wants_focus lambda) that capture `this` -- a copy or
  // move would silently leave every callback pointing at the source object.
  PaneBox(const PaneBox &)            = delete;
  PaneBox &operator=(const PaneBox &) = delete;
  PaneBox(PaneBox &&)                 = delete;
  PaneBox &operator=(PaneBox &&)      = delete;

  // Advance to the next pane (bound to the 'p' key).
  void cycle();

  // Jump directly to pane idx (bound to number keys). No-op if out of range.
  void select(std::size_t idx);

  // Draws the selected pane into win, first auto-switching to any pane whose wants_focus() just
  // turned true. snapshot/light_scheme/mini are only valid for the duration of this call.
  void render(WINDOW *win, const status::RenderSnapshot &snapshot, bool light_scheme, bool mini);

private:
  struct Pane
  {
    std::string                      title;
    std::function<void(WINDOW *win)> render;
    std::function<bool()>            wants_focus; // optional; may be nullptr
  };

  // box + numbered tab bar in the top border; returns the first usable content row.
  int drawPaneChrome(WINDOW *win);
  // Lists missing_for_automatic_takeoff/errors from GeneralRobotInfo.
  void renderProblemsPane(WINDOW *win);
  // Lists available_sensors from SystemHealthInfo, worst-severity first.
  void renderSensorsPane(WINDOW *win);
  // Lists per-node CPU load from SystemHealthInfo, highest first.
  void renderNodeCpuPane(WINDOW *win);
  // Shows GNSS fix/accuracy and the current display_string entries.
  void renderGnssStringsPane(WINDOW *win);

  std::vector<Pane> panes_;
  std::size_t       pane_idx_ = 0;
  std::vector<bool> pane_focus_prev_; // per-pane wants_focus() last state

  // Set at the top of every render() call and read by the pane callbacks below it. Never
  // dereferenced outside a render() call -- the pane callbacks are only invoked from there.
  const status::RenderSnapshot *snapshot_     = nullptr;
  bool                          light_scheme_ = false;
  bool                          mini_         = false;
};

} // namespace mrs_uav_status::tui
