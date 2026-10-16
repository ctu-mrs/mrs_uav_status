#pragma once

/* includes //{ */

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <mrs_uav_status/status/data_types.hpp>
#include <mrs_uav_status/status/string_info.hpp>
#include <mrs_uav_status/status/command_sink.hpp>
#include <mrs_uav_status/status/clock.hpp>
#include <mrs_uav_status/tui/tui_actions.hpp>

//}

namespace mrs_uav_status::status
{

enum class StatusState
{
  STANDARD,
  REMOTE,
  MAIN_MENU,
  GOTO_MENU,
  DISPLAY_MENU
};

// The package's decision layer: owns the STANDARD/REMOTE/menu state machine, every received
// message snapshot and all menu/goto/remote logic. No ROS, no ncurses -- it only ever drives an
// abstract tui::TuiActions, so tests can substitute a fake.
class UavStatusCore {
public:
  // Node params this layer needs. Loaded by UavStatus from mrs_uav_status/.
  struct Params
  {
    std::string         turbo_remote_constraints; // constraint set swapped in by turbo remote ("fast")
    std::vector<double> goto_values;              // initial X/Y/Z/heading offered by the goto menu
  };

  // All outbound ROS actions are dispatched through command_sink; clock is read for
  // post-service-call timestamps (defaults to always-0.0 if left unset, e.g. in tests).
  UavStatusCore(CommandSink command_sink, Params params, Clock clock = {});

  // Stores the per-topic freshness for this tick. Call once before every tick().
  void setFreshness(const Freshness &freshness);

  // Advances the state machine by one render tick: dispatches key through the current state and
  // drives tui accordingly. now_seconds is the node clock in plain seconds.
  void tick(double now_seconds, int key, tui::TuiActions &tui);

  /* state() //{ */

  StatusState state() const {
    return state_;
  }

  //}

  // | --------------------- Data push (thread-safe) --------------------- |
  // Called from ROS subscriber threads; each stores data into its last_*_ snapshot under
  // mutex_status_msg_ for the next snapshot() to read.
  void onGeneralRobotInfo(const GeneralRobotInfoData &data);
  void onStateEstimationInfo(const StateEstimationInfoData &data);
  void onControlInfo(const ControlInfoData &data);
  void onCollisionAvoidanceInfo(const CollisionAvoidanceInfoData &data);
  void onUavInfo(const UavInfoData &data);
  void onSystemHealthInfo(const SystemHealthInfoData &data);
  void onUavState(const StateData &data);

  // Parses an optional "-id <key>" / "-p" (persistent) preamble, then dedupes-or-appends the
  // remaining text into string_info_vec_ (shown in the GNSS & Strings pane). Called from a ROS
  // subscriber thread; now_seconds is the node clock at reception.
  void onString(double now_seconds, const std::string &data);

  // Deep-copies everything the renderer needs, under mutex_status_msg_. Call once per fast tick,
  // after setFreshness() and before any NcursesTui render call.
  RenderSnapshot snapshot(double now_seconds) const;

private:
  // True if ControlInfo reports flying_normally (gates remote mode and turbo remote).
  bool isFlyingNormally() const;

  // Evict non-persistent display_string entries older than 10 s. Runs at the top of every tick().
  void pruneStrings(double now_seconds);

  // | ----------------------- Menu / goto ---------------------- |
  // One main-menu row: its label, and the submenu it opens when selected. The submenu's contents
  // are built inside open_submenu (i.e. at selection time), so they always reflect live data.
  struct MainMenuRow
  {
    std::string                               label;
    std::function<void(tui::TuiActions &tui)> open_submenu;
  };

  // One submenu row: its label, and the service it calls when selected. A null action means
  // "no-op" and is used for the CANCEL row.
  struct SubMenuRow
  {
    std::string                                 label;
    std::function<CommandSink::ServiceResult()> action;
  };

  // Builds main_menu_rows_ from the configured services + the five setter submenus, then hands
  // the labels to tui.showMainMenu().
  void setupMainMenu(tui::TuiActions &tui);
  // Dispatches one keypress through tui.handleMainMenuKey(). Returns true when the whole menu closes.
  bool mainMenuHandler(int key, tui::TuiActions &tui);
  // Fills sub_menu_rows_ with one row per label, each calling call(label), then shows the submenu.
  void buildSubMenu(tui::TuiActions &tui, const std::vector<std::string> &labels, const std::function<CommandSink::ServiceResult(const std::string &)> &call);
  // Same, for a no-argument service; a "CANCEL" label becomes a no-op row instead of a call.
  void buildSubMenu(tui::TuiActions &tui, const std::vector<std::string> &labels, const std::function<CommandSink::ServiceResult()> &call);
  // Builds the goto window's labels (X/Y/Z/hdg + the current frame) and shows it seeded with goto_values_.
  void setupGotoMenu(tui::TuiActions &tui);
  // Dispatches one keypress through tui.handleGotoMenuKey(); on commit calls sendGoto. Returns true when done.
  bool gotoMenuHandler(int key, tui::TuiActions &tui);

  // | -------------------------- Remote ------------------------ |
  // Resets remote_hover_ when entering remote mode.
  void enterRemoteMode();
  // Dispatches one keypress in remote mode: 'T' toggles turbo, 'G' toggles local/global frame, else flies/hovers.
  void remoteHandler(int key, tui::TuiActions &tui);
  // Maps a keypress to a velocity command (wasd/hjkl/arrows for xy, r/f for z, q/e for heading); any
  // other key triggers hover() if a motion command was previously sent.
  void handleRemoteMotion(int key);
  // Toggles turbo_remote_constraints on/off via the set-constraints service, remembering the previous set.
  void toggleTurboRemote(tui::TuiActions &tui);
  // Sends one velocity reference, in the world frame if remote_global_ else the fcu_untilted frame.
  void remoteModeFly(double vx, double vy, double vz, double heading_rate);

  static bool isValidIndex(int index, std::size_t container_size);

  CommandSink command_sink_;
  Clock       clock_;
  Params      params_;

  std::vector<MainMenuRow> main_menu_rows_;
  std::vector<SubMenuRow>  sub_menu_rows_;
  std::vector<double>      goto_values_;

  bool        remote_hover_  = false;
  bool        turbo_remote_  = false;
  bool        remote_global_ = false;
  std::string old_constraints_;

  StatusState state_ = StatusState::STANDARD;

  // Threading invariant: last_*_ members and freshness_ below are written from ROS subscriber
  // threads (the on*() setters) and read by snapshot(), which deep-copies them out under this
  // lock exactly once per fast tick. tui::NcursesTui::snapshot_ (the copy snapshot() produces) is then
  // written and read only from the render thread, so NcursesTui itself needs no mutex for this data.
  mutable std::mutex         mutex_status_msg_;
  Freshness                  freshness_;
  GeneralRobotInfoData       last_general_robot_info_;
  StateEstimationInfoData    last_state_estimation_info_;
  ControlInfoData            last_control_info_;
  CollisionAvoidanceInfoData last_collision_avoidance_info_;
  UavInfoData                last_uav_info_;
  SystemHealthInfoData       last_system_health_info_;
  StateData                  last_uav_state_;

  // Custom strings published via std_msgs/String, guarded by mutex_status_msg_. Deduped by the id
  // parsed from "-id <key>"; entries older than 10 s are pruned unless marked persistent ("-p").
  std::vector<StringInfo> string_info_vec_;
};

} // namespace mrs_uav_status::status
