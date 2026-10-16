/* includes //{ */

#include <algorithm>
#include <iterator>
#include <sstream>
#include <utility>

#include <mrs_uav_status/status/uav_status_core.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/utils/helpers.hpp>

//}

namespace mrs_uav_status::status
{

/* UavStatusCore() //{ */

UavStatusCore::UavStatusCore(CommandSink command_sink, Params params, Clock clock)
    : command_sink_(std::move(command_sink)), clock_(std::move(clock)), params_(std::move(params)), goto_values_(params_.goto_values) {
  goto_values_.resize(4, 0.0);

  // clock_.now is a hard dependency of every service-result path (unlike command_sink_'s
  // std::function members, which are only reached via user interaction) -- guard against a
  // caller leaving it unset so those call sites don't throw std::bad_function_call.
  if (!clock_.now) {
    clock_.now = [] { return 0.0; };
  }
}

//}

/* setFreshness() //{ */

void UavStatusCore::setFreshness(const Freshness &freshness) {
  std::scoped_lock lock(mutex_status_msg_);
  freshness_ = freshness;
}

//}

// | --------------------- Data push (thread-safe) --------------------- |

/* on*() setters //{ */

void UavStatusCore::onGeneralRobotInfo(const GeneralRobotInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_general_robot_info_ = data;
}

void UavStatusCore::onStateEstimationInfo(const StateEstimationInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_state_estimation_info_ = data;
}

void UavStatusCore::onControlInfo(const ControlInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_control_info_ = data;
}

void UavStatusCore::onCollisionAvoidanceInfo(const CollisionAvoidanceInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_collision_avoidance_info_ = data;
}

void UavStatusCore::onUavInfo(const UavInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_info_ = data;
}

void UavStatusCore::onSystemHealthInfo(const SystemHealthInfoData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_system_health_info_ = data;
}

void UavStatusCore::onUavState(const StateData &data) {
  std::scoped_lock lock(mutex_status_msg_);
  last_uav_state_ = data;
}

//}

/* isFlyingNormally() //{ */

bool UavStatusCore::isFlyingNormally() const {
  std::scoped_lock lock(mutex_status_msg_);
  return last_control_info_.flying_normally;
}

//}

/* onString() //{ */

void UavStatusCore::onString(double now_seconds, const std::string &data) {
  // Parse leading flags ("-id <key>" optional dedupe key, "-p" mark persistent), rejoin
  // remaining tokens as the display text, then dedupe-or-append in string_info_vec_.
  std::stringstream                  ss(data);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string>           tokens(begin, end);
  if (tokens.empty()) {
    return;
  }

  std::string id;
  bool        persistent = false;
  bool        flags_done = false;
  size_t      i          = 0;
  while (!flags_done && i < tokens.size()) {
    if (tokens[i] == "-id" && i + 1 < tokens.size()) {
      id = tokens[i + 1];
      tokens.erase(tokens.begin() + i, tokens.begin() + i + 2);
    } else if (tokens[i] == "-p") {
      persistent = true;
      tokens.erase(tokens.begin() + i);
    } else if (!tokens[i].empty() && tokens[i].front() != '-') {
      flags_done = true;
    } else {
      ++i;
    }
  }

  std::string display;
  for (size_t k = 0; k < tokens.size(); ++k) {
    if (k > 0) {
      display += ' ';
    }
    display += tokens[k];
  }

  std::scoped_lock lock(mutex_status_msg_);
  // Dedupe by id.
  for (auto &entry : string_info_vec_) {
    if (entry.id == id) {
      entry.display_string = display;
      entry.persistent     = persistent;
      entry.last_time      = now_seconds;
      return;
    }
  }
  string_info_vec_.emplace_back(now_seconds, display, id, persistent);
}

//}

/* pruneStrings() //{ */

void UavStatusCore::pruneStrings(double now_seconds) {
  std::scoped_lock lock(mutex_status_msg_);
  for (auto it = string_info_vec_.begin(); it != string_info_vec_.end();) {
    if (!it->persistent && (now_seconds - it->last_time) > 10.0) {
      it = string_info_vec_.erase(it);
    } else {
      ++it;
    }
  }
}

//}

/* snapshot() //{ */

RenderSnapshot UavStatusCore::snapshot(double now_seconds) const {
  std::scoped_lock lock(mutex_status_msg_);

  RenderSnapshot out;
  out.now_seconds              = now_seconds;
  out.freshness                = freshness_;
  out.general_robot_info       = last_general_robot_info_;
  out.state_estimation_info    = last_state_estimation_info_;
  out.control_info             = last_control_info_;
  out.collision_avoidance_info = last_collision_avoidance_info_;
  out.uav_info                 = last_uav_info_;
  out.system_health_info       = last_system_health_info_;
  out.uav_state                = last_uav_state_;
  out.border_status            = BorderStatus{
                 .avoiding_collision = last_collision_avoidance_info_.avoiding_collision,
                 .bumper_active      = last_collision_avoidance_info_.bumper_active,
                 .can_takeoff        = last_general_robot_info_.ready_for_automatic_takeoff,
                 .null_tracker       = (last_control_info_.active_tracker == "NullTracker"),
  };
  out.display_strings.reserve(string_info_vec_.size());
  for (const auto &entry : string_info_vec_) {
    out.display_strings.push_back(entry.display_string);
  }
  return out;
}

//}

// | ----------------------- Menu / goto ---------------------- |

/* isValidIndex() //{ */

bool UavStatusCore::isValidIndex(int index, std::size_t container_size) {
  return index >= 0 && static_cast<std::size_t>(index) < container_size;
}

//}

/* buildSubMenu(const std::function<CommandSink::ServiceResult(const std::string &)> &call) //{ */

void UavStatusCore::buildSubMenu(tui::TuiActions &tui, const std::vector<std::string> &labels,
                                 const std::function<CommandSink::ServiceResult(const std::string &)> &call) {
  sub_menu_rows_.clear();
  for (const auto &label : labels) {
    sub_menu_rows_.push_back({label, [label, call]() { return call(label); }});
  }
  tui.showSubMenu(labels);
}

//}

/* buildSubMenu() //{ */

void UavStatusCore::buildSubMenu(tui::TuiActions &tui, const std::vector<std::string> &labels, const std::function<CommandSink::ServiceResult()> &call) {
  sub_menu_rows_.clear();
  for (const auto &label : labels) {
    if (label == "CANCEL") {
      sub_menu_rows_.push_back({label, nullptr});
      continue;
    }
    sub_menu_rows_.push_back({label, call});
  }
  tui.showSubMenu(labels);
}

//}

/* setupMainMenu() //{ */

void UavStatusCore::setupMainMenu(tui::TuiActions &tui) {
  main_menu_rows_.clear();
  sub_menu_rows_.clear();

  bool can_land, can_takeoff;
  {
    std::scoped_lock lock(mutex_status_msg_);
    can_land    = freshness_.uav_info && utils::isFlyingAutonomously(last_uav_info_.flight_state);
    can_takeoff = freshness_.uav_info && utils::isOnGround(last_uav_info_.flight_state);
  }

  // Config-driven Trigger services. Landing (incl. land home) only applies while the MRS system is
  // flying, taking off only on the ground; with no fresh UavInfo neither is offered.
  for (const auto &service : command_sink_.extra_services) {
    std::string name = service.display_name;
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    if (!can_land && (name.find("land") != std::string::npos)) {
      continue;
    }
    if (!can_takeoff && (name.find("takeoff") != std::string::npos)) {
      continue;
    }
    main_menu_rows_.push_back({service.display_name, [this, service](tui::TuiActions &t) { buildSubMenu(t, {"CANCEL", service.display_name}, service.call); }});
  }

  // Each of these reads the *live* available_* list when the row is selected, not now.
  main_menu_rows_.push_back({"Set Constraints", [this](tui::TuiActions &t) {
                               std::vector<std::string> labels;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 labels = utils::withActiveFirst(last_control_info_.active_constraints, last_control_info_.available_constraints);
                               }
                               buildSubMenu(t, labels, command_sink_.setConstraints);
                             }});

  main_menu_rows_.push_back({"Set Gains", [this](tui::TuiActions &t) {
                               std::vector<std::string> labels;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 labels = utils::withActiveFirst(last_control_info_.active_gains, last_control_info_.available_gains);
                               }
                               buildSubMenu(t, labels, command_sink_.setGains);
                             }});

  main_menu_rows_.push_back({"Set Controller", [this](tui::TuiActions &t) {
                               std::vector<std::string> labels;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 labels = utils::withActiveFirst(last_control_info_.active_controller, last_control_info_.available_controllers);
                               }
                               buildSubMenu(t, labels, command_sink_.setController);
                             }});

  main_menu_rows_.push_back({"Set Tracker", [this](tui::TuiActions &t) {
                               std::vector<std::string> labels;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 labels = utils::withActiveFirst(last_control_info_.active_tracker, last_control_info_.available_trackers);
                               }
                               buildSubMenu(t, labels, command_sink_.setTracker);
                             }});

  main_menu_rows_.push_back({"Set Estimator", [this](tui::TuiActions &t) {
                               std::vector<std::string> labels;
                               {
                                 std::scoped_lock lock(mutex_status_msg_);
                                 labels =
                                     utils::withActiveFirst(last_state_estimation_info_.current_estimator, last_state_estimation_info_.switchable_estimators);
                               }
                               buildSubMenu(t, labels, command_sink_.setEstimator);
                             }});

  std::vector<std::string> labels;
  labels.reserve(main_menu_rows_.size());
  for (const auto &row : main_menu_rows_) {
    labels.push_back(row.label);
  }
  tui.showMainMenu(labels);
}

//}

/* mainMenuHandler() //{ */

bool UavStatusCore::mainMenuHandler(int key, tui::TuiActions &tui) {

  const tui::MenuEvent event = tui.handleMainMenuKey(key);

  if (event.in_submenu) {

    if (event.kind == tui::MenuEvent::Kind::Exit) {
      // Escape here only backs out of the submenu, back to the main menu.
      tui.closeSubMenu();
      return false;
    }

    if (event.kind == tui::MenuEvent::Kind::Selected && isValidIndex(event.index, sub_menu_rows_.size())) {
      const SubMenuRow &row = sub_menu_rows_[event.index];
      tui.closeSubMenu();

      if (!row.action) {
        // Cancel backs out to the main menu, it shouldn't close the whole thing.
        return false;
      }

      const auto result = row.action();
      tui.renderServiceResult(result.success, result.message, clock_.now());
      sub_menu_rows_.clear();
      return true;
    }
    return false;
  }

  if (event.kind == tui::MenuEvent::Kind::Exit) {
    return true;
  }

  if (event.kind == tui::MenuEvent::Kind::Selected && isValidIndex(event.index, main_menu_rows_.size())) {
    main_menu_rows_[event.index].open_submenu(tui);
  }

  return false;
}

//}

/* setupGotoMenu() //{ */

void UavStatusCore::setupGotoMenu(tui::TuiActions &tui) {
  std::string odom_frame;
  {
    std::scoped_lock lock(mutex_status_msg_);
    odom_frame = last_state_estimation_info_.frame_id;
  }

  const std::vector<std::string> labels{
      " X:                ", " Y:                ", " Z:                ", " hdg:              ", " " + odom_frame + " ",
  };

  tui.showGotoMenu(labels, goto_values_);
}

//}

/* gotoMenuHandler() //{ */

bool UavStatusCore::gotoMenuHandler(int key, tui::TuiActions &tui) {

  const tui::GotoEvent event = tui.handleGotoMenuKey(key);

  if (event.kind == tui::GotoEvent::Kind::Exit) {
    return true;
  }

  if (event.kind == tui::GotoEvent::Kind::Committed) {
    // Remembered so the next 'g' reopens pre-filled with what was last entered.
    goto_values_ = {event.x, event.y, event.z, event.heading};

    std::string frame_id;
    {
      std::scoped_lock lock(mutex_status_msg_);
      frame_id = last_state_estimation_info_.frame_id;
    }

    const auto result = command_sink_.sendGoto(event.x, event.y, event.z, event.heading, frame_id);
    tui.renderServiceResult(result.success, result.message, clock_.now());
    return true;
  }

  return false;
}

//}

// | -------------------------- Remote ------------------------ |

/* enterRemoteMode() //{ */

void UavStatusCore::enterRemoteMode() {
  remote_hover_ = false;
}

//}

/* remoteHandler() //{ */

void UavStatusCore::remoteHandler(int key, tui::TuiActions &tui) {
  tui.renderRemoteBanner(turbo_remote_, remote_global_);

  if (key == 'T') {
    toggleTurboRemote(tui);
    return;
  }

  if (key == 'G') {
    if (isFlyingNormally()) {
      remote_global_ = !remote_global_;
    }
    return;
  }

  handleRemoteMotion(key);
}

//}

/* handleRemoteMotion() //{ */

void UavStatusCore::handleRemoteMotion(int key) {
  const double xy_step  = turbo_remote_ ? 5.0 : 2.0;
  const double z_step   = turbo_remote_ ? 2.0 : 1.0;
  const double hdg_step = turbo_remote_ ? 1.0 : 0.5;

  auto fly = [&](double vx, double vy, double vz, double vhdg) {
    remoteModeFly(vx, vy, vz, vhdg);
    remote_hover_ = true;
  };

  switch (key) {
  case 'w':
  case 'k':
  case static_cast<int>(tui::Key::Up):
    fly(xy_step, 0, 0, 0);
    break;
  case 's':
  case 'j':
  case static_cast<int>(tui::Key::Down):
    fly(-xy_step, 0, 0, 0);
    break;
  case 'a':
  case 'h':
  case static_cast<int>(tui::Key::Left):
    fly(0, xy_step, 0, 0);
    break;
  case 'd':
  case 'l':
  case static_cast<int>(tui::Key::Right):
    fly(0, -xy_step, 0, 0);
    break;

  case 'r':
    fly(0, 0, z_step, 0);
    break;
  case 'f':
    fly(0, 0, -z_step, 0);
    break;

  case 'q':
    fly(0, 0, 0, hdg_step);
    break;
  case 'e':
    fly(0, 0, 0, -hdg_step);
    break;

  default:
    if (remote_hover_) {
      command_sink_.hover();
      remote_hover_ = false;
    }
    break;
  }
}

//}

/* toggleTurboRemote() //{ */

void UavStatusCore::toggleTurboRemote(tui::TuiActions &tui) {
  if (!isFlyingNormally()) {
    return;
  }

  if (turbo_remote_) {
    // Toggle down turbo remote after new pressed T
    turbo_remote_     = false;
    const auto result = command_sink_.setConstraints(old_constraints_);
    tui.renderServiceResult(result.success, result.message, clock_.now());
    return;
  }

  // Enable turbo remote constraints
  turbo_remote_ = true;
  {
    std::scoped_lock lock(mutex_status_msg_);
    old_constraints_ = last_control_info_.active_constraints;
  }
  const auto result = command_sink_.setConstraints(params_.turbo_remote_constraints);
  tui.renderServiceResult(result.success, result.message, clock_.now());
}

//}

/* remoteModeFly() //{ */

void UavStatusCore::remoteModeFly(double vx, double vy, double vz, double heading_rate) {
  std::string uav_name;
  {
    std::scoped_lock lock(mutex_status_msg_);
    uav_name = last_general_robot_info_.robot_name;
  }

  const std::string frame_id = uav_name + (remote_global_ ? "/world_origin" : "/fcu_untilted");

  command_sink_.sendVelocityReference(vx, vy, vz, heading_rate, frame_id);
}

//}

// | -------------------------- Tick -------------------------- |

/* tick() //{ */

void UavStatusCore::tick(double now_seconds, int key, tui::TuiActions &tui) {

  pruneStrings(now_seconds);

  switch (state_) {

  case StatusState::STANDARD: {
    switch (key) {

    case 'R': {
      if (isFlyingNormally()) {
        enterRemoteMode();
        tui.setRemoteMode(true);
        state_ = StatusState::REMOTE;
      }
      break;
    }

    case 'm':
      setupMainMenu(tui);
      state_ = StatusState::MAIN_MENU;
      break;

    case 'g':
      setupGotoMenu(tui);
      state_ = StatusState::GOTO_MENU;
      break;

    case 'h':
      tui.toggleHelp();
      break;

    case 'p':
      tui.cyclePanes();
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      tui.selectPane(static_cast<std::size_t>(key - '1'));
      break;

    case 'M':
      tui.toggleMini();
      tui.setupWindows();
      tui.renderFast();
      tui.renderSlow();
      break;

    case 'D':
      tui.setupDisplayMenu();
      state_ = StatusState::DISPLAY_MENU;
      break;

    default:
      tui.flushInput();
      break;
    }
    break;
  }

  case StatusState::REMOTE: {
    tui.flushInput();
    remoteHandler(key, tui);
    if (key == 'R' || key == static_cast<int>(mrs_uav_status::tui::Key::Escape)) {
      tui.setRemoteMode(false);
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::MAIN_MENU: {
    tui.flushInput();
    if (mainMenuHandler(key, tui)) {
      tui.clearMenus();
      tui.refreshAfterMenu();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::GOTO_MENU: {
    tui.flushInput();
    if (gotoMenuHandler(key, tui)) {
      tui.clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }

  case StatusState::DISPLAY_MENU: {
    tui.flushInput();
    if (tui.displayMenuHandler(key)) {
      tui.clearMenus();
      state_ = StatusState::STANDARD;
    }
    break;
  }
  }

  if (state_ != StatusState::MAIN_MENU && state_ != StatusState::GOTO_MENU && state_ != StatusState::DISPLAY_MENU) {
    tui.refreshBottomWindow();
  }
}

//}

} // namespace mrs_uav_status::status
