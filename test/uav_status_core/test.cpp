/* includes //{ */

#include <gtest/gtest.h>

#include <algorithm>

#include <mrs_uav_status/status/uav_status_core.hpp>
#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/utils/helpers.hpp>

//}

namespace mrs_uav_status::status
{

namespace
{

// | ----------------------- Test doubles ---------------------- |

class FakeTui : public tui::TuiActions {
public:
  // programmable responses
  tui::MenuEvent next_menu_event;
  tui::GotoEvent next_goto_event;
  bool           display_menu_should_close = false;

  // recorded calls
  std::vector<std::string> main_menu_labels;
  std::vector<std::string> sub_menu_labels;
  std::vector<std::string> goto_menu_labels;
  std::vector<double>      goto_initial_values;
  std::vector<std::string> service_results;

  int  show_main_menu_calls        = 0;
  int  show_sub_menu_calls         = 0;
  int  close_sub_menu_calls        = 0;
  int  show_goto_menu_calls        = 0;
  int  setup_display_menu_calls    = 0;
  int  clear_menus_calls           = 0;
  int  refresh_after_menu_calls    = 0;
  int  refresh_bottom_window_calls = 0;
  int  toggle_help_calls           = 0;
  int  cycle_panes_calls           = 0;
  int  select_pane_last_idx        = -1;
  int  toggle_mini_calls           = 0;
  int  flush_input_calls           = 0;
  int  remote_banner_calls         = 0;
  bool remote_banner_turbo         = false;
  bool remote_banner_global        = false;
  bool remote_mode_active          = false;

  /* showMainMenu() //{ */

  void showMainMenu(const std::vector<std::string> &labels) override {
    main_menu_labels = labels;
    show_main_menu_calls++;
  }

  //}

  /* showSubMenu() //{ */

  void showSubMenu(const std::vector<std::string> &labels) override {
    sub_menu_labels = labels;
    show_sub_menu_calls++;
  }

  //}

  /* closeSubMenu() //{ */

  void closeSubMenu() override {
    close_sub_menu_calls++;
  }

  //}

  /* handleMainMenuKey() //{ */

  tui::MenuEvent handleMainMenuKey(int) override {
    return next_menu_event;
  }

  //}

  /* showGotoMenu() //{ */

  void showGotoMenu(const std::vector<std::string> &labels, const std::vector<double> &initial_values) override {
    goto_menu_labels    = labels;
    goto_initial_values = initial_values;
    show_goto_menu_calls++;
  }

  //}

  /* handleGotoMenuKey() //{ */

  tui::GotoEvent handleGotoMenuKey(int) override {
    return next_goto_event;
  }

  //}

  /* setupDisplayMenu() //{ */

  void setupDisplayMenu() override {
    setup_display_menu_calls++;
  }

  //}

  /* displayMenuHandler() //{ */

  bool displayMenuHandler(int) override {
    return display_menu_should_close;
  }

  //}

  /* clearMenus() //{ */

  void clearMenus() override {
    clear_menus_calls++;
  }

  //}

  /* refreshAfterMenu() //{ */

  void refreshAfterMenu() override {
    refresh_after_menu_calls++;
  }

  //}

  /* setRemoteMode() //{ */

  void setRemoteMode(bool in_remote_mode) override {
    remote_mode_active = in_remote_mode;
  }

  //}

  /* renderRemoteBanner() //{ */

  void renderRemoteBanner(bool turbo, bool global) override {
    remote_banner_turbo  = turbo;
    remote_banner_global = global;
    remote_banner_calls++;
  }

  //}

  /* renderServiceResult() //{ */

  // NOTE: TuiActions::renderServiceResult gained a third parameter (`now_seconds`, the
  // post-service-call wall clock used to stamp the bottom-window clear timer) after this test's
  // literal brief text was written. Overriding a pure virtual with a mismatched signature is a
  // compile error, so the third parameter is added here and ignored -- no test below inspects it.
  void renderServiceResult(bool success, const std::string &message, double /*now_seconds*/) override {
    service_results.push_back((success ? "OK: " : "FAIL: ") + message);
  }

  //}

  /* toggleHelp() //{ */

  void toggleHelp() override {
    toggle_help_calls++;
  }

  //}

  /* cyclePanes() //{ */

  void cyclePanes() override {
    cycle_panes_calls++;
  }

  //}

  /* selectPane() //{ */

  void selectPane(std::size_t idx) override {
    select_pane_last_idx = static_cast<int>(idx);
  }

  //}

  /* toggleMini() //{ */

  void toggleMini() override {
    toggle_mini_calls++;
  }

  //}

  /* setupWindows() //{ */

  void setupWindows() override {
  }

  //}

  /* renderFast() //{ */

  void renderFast() override {
  }

  //}

  /* renderSlow() //{ */

  void renderSlow() override {
  }

  //}

  /* flushInput() //{ */

  void flushInput() override {
    flush_input_calls++;
  }

  //}

  /* refreshBottomWindow() //{ */

  void refreshBottomWindow() override {
    refresh_bottom_window_calls++;
  }

  //}
};

// Records every outbound service call the model makes.
struct SinkLog
{
  std::vector<std::string> calls;

  double      vx = 0.0, vy = 0.0, vz = 0.0, vhdg = 0.0;
  std::string velocity_frame;

  double      goto_x = 0.0, goto_y = 0.0, goto_z = 0.0, goto_hdg = 0.0;
  std::string goto_frame;
};

/* makeSink() //{ */

CommandSink makeSink(SinkLog &log) {
  CommandSink sink;

  sink.sendGoto = [&log](double x, double y, double z, double heading, const std::string &frame_id) {
    log.goto_x     = x;
    log.goto_y     = y;
    log.goto_z     = z;
    log.goto_hdg   = heading;
    log.goto_frame = frame_id;
    log.calls.push_back("sendGoto");
    return CommandSink::ServiceResult{true, "goto ok"};
  };

  sink.sendVelocityReference = [&log](double vx, double vy, double vz, double heading_rate, const std::string &frame_id) {
    log.vx             = vx;
    log.vy             = vy;
    log.vz             = vz;
    log.vhdg           = heading_rate;
    log.velocity_frame = frame_id;
    log.calls.push_back("sendVelocityReference");
  };

  auto recorder = [&log](const std::string &name) {
    return [&log, name](const std::string &value) {
      log.calls.push_back(name + "(" + value + ")");
      return CommandSink::ServiceResult{true, value};
    };
  };
  sink.setConstraints = recorder("setConstraints");
  sink.setGains       = recorder("setGains");
  sink.setController  = recorder("setController");
  sink.setTracker     = recorder("setTracker");
  sink.setEstimator   = recorder("setEstimator");

  sink.hover = [&log]() {
    log.calls.push_back("hover");
    return CommandSink::ServiceResult{true, "hovering"};
  };

  sink.extra_services.push_back({"Land", [&log]() {
                                   log.calls.push_back("Land");
                                   return CommandSink::ServiceResult{true, "landing"};
                                 }});
  sink.extra_services.push_back({"Takeoff", [&log]() {
                                   log.calls.push_back("Takeoff");
                                   return CommandSink::ServiceResult{true, "taking off"};
                                 }});

  return sink;
}

//}

/* defaultParams() //{ */

// Mirrors config/public/default.yaml.
UavStatusCore::Params defaultParams() {
  return UavStatusCore::Params{.turbo_remote_constraints = "fast", .goto_values = {0.0, 0.0, 2.0, 1.57}};
}

//}

/* flyingControlInfo() //{ */

// A ControlInfo that reads as "airborne, under MpcTracker, on the 'medium' constraint set".
ControlInfoData flyingControlInfo() {
  ControlInfoData ci;
  ci.flying_normally       = true;
  ci.active_tracker        = "MpcTracker";
  ci.available_trackers    = {"MpcTracker", "LandoffTracker"};
  ci.active_constraints    = "medium";
  ci.available_constraints = {"slow", "medium", "fast"};
  ci.active_gains          = "supersoft";
  ci.available_gains       = {"supersoft", "soft"};
  ci.active_controller     = "Se3Controller";
  ci.available_controllers = {"Se3Controller", "MpcController"};
  return ci;
}

//}

/* uavInfoIn() //{ */

// A UavInfo reporting DiagnosticsManager's given flight_state.
UavInfoData uavInfoIn(const std::string &flight_state) {
  UavInfoData ui;
  ui.flight_state = flight_state;
  return ui;
}

//}

/* tick() //{ */

void tick(UavStatusCore &model, FakeTui &tui, int key, double now_seconds = 0.0, const Freshness &freshness = {}) {
  model.setFreshness(freshness);
  model.tick(now_seconds, key, tui);
}

//}

/* openMainMenuWithFreshUavInfo() //{ */

// Opens the main menu with UavInfo fresh, so the flight_state-gated rows (Land, Takeoff) are decided by it.
void openMainMenuWithFreshUavInfo(UavStatusCore &model, FakeTui &tui) {
  tick(model, tui, 'm', 0.0, Freshness{.uav_info = true});
}

//}

/* indexOf() //{ */

int indexOf(const std::vector<std::string> &labels, const std::string &label) {
  const auto it = std::find(labels.begin(), labels.end(), label);
  return (it == labels.end()) ? -1 : static_cast<int>(std::distance(labels.begin(), it));
}

//}

/* selectMain() //{ */

tui::MenuEvent selectMain(int index) {
  return tui::MenuEvent{tui::MenuEvent::Kind::Selected, false, index};
}

//}

/* selectSub() //{ */

tui::MenuEvent selectSub(int index) {
  return tui::MenuEvent{tui::MenuEvent::Kind::Selected, true, index};
}

//}

} // namespace

// | --------------------- Top-level FSM ---------------------- |

/* TEST(UavStatusCore, StartsInStandardState) //{ */

TEST(UavStatusCore, StartsInStandardState) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

//}

/* TEST(UavStatusCore, UnrecognizedKeyInStandardFlushesInput) //{ */

TEST(UavStatusCore, UnrecognizedKeyInStandardFlushesInput) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, 'z');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.flush_input_calls, 1);
}

//}

/* TEST(UavStatusCore, NumberKeySelectsPaneByZeroBasedIndex) //{ */

TEST(UavStatusCore, NumberKeySelectsPaneByZeroBasedIndex) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, '3');

  EXPECT_EQ(tui.select_pane_last_idx, 2);
}

//}

/* TEST(UavStatusCore, RefreshesBottomWindowOnlyOutsideMenuStates) //{ */

TEST(UavStatusCore, RefreshesBottomWindowOnlyOutsideMenuStates) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, 'h'); // STANDARD -> STANDARD
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1);
  EXPECT_EQ(tui.toggle_help_calls, 1);

  tick(sm, tui, 'm');                            // STANDARD -> MAIN_MENU
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1); // unchanged: now in a menu state
}

//}

/* TEST(UavStatusCore, DisplayMenuOpensAndClosesOnHandlerReturningTrue) //{ */

TEST(UavStatusCore, DisplayMenuOpensAndClosesOnHandlerReturningTrue) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, 'D');
  ASSERT_EQ(sm.state(), StatusState::DISPLAY_MENU);
  EXPECT_EQ(tui.setup_display_menu_calls, 1);

  tui.display_menu_should_close = true;
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  // Unlike MAIN_MENU, DISPLAY_MENU close must not call refreshAfterMenu().
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

//}

// | ------------------------ Main menu ------------------------ |

/* TEST(UavStatusCore, MainMenuHidesTakeoffWhileFlying) //{ */

TEST(UavStatusCore, MainMenuHidesTakeoffWhileFlying) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  sm.onUavInfo(uavInfoIn("HOVER"));

  openMainMenuWithFreshUavInfo(sm, tui);

  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.show_main_menu_calls, 1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Land"), -1);
  EXPECT_EQ(indexOf(tui.main_menu_labels, "Takeoff"), -1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Set Constraints"), -1);
}

//}

/* TEST(UavStatusCore, MainMenuHidesLandOnTheGround) //{ */

TEST(UavStatusCore, MainMenuHidesLandOnTheGround) {
  for (const std::string state : {"DISARMED", "ARMED", "OFFBOARD"}) {
    SinkLog       log;
    UavStatusCore sm(makeSink(log), defaultParams());
    FakeTui       tui;

    // A tracker left active (e.g. after a pilot took over and landed) must not make Land available.
    sm.onControlInfo(flyingControlInfo());
    sm.onUavInfo(uavInfoIn(state));

    openMainMenuWithFreshUavInfo(sm, tui);

    EXPECT_EQ(indexOf(tui.main_menu_labels, "Land"), -1) << state;
    EXPECT_NE(indexOf(tui.main_menu_labels, "Takeoff"), -1) << state;
  }
}

//}

/* TEST(UavStatusCore, MainMenuOffersLandWhileMrsIsFlying) //{ */

TEST(UavStatusCore, MainMenuOffersLandWhileMrsIsFlying) {
  for (const std::string state : {"TAKEOFF", "LAND", "HOVER", "GOTO", "TRAJECTORY", "MIDAIR", "RC_MODE", "EHOVER", "ELAND", "FAILSAFE"}) {
    SinkLog       log;
    UavStatusCore sm(makeSink(log), defaultParams());
    FakeTui       tui;

    sm.onUavInfo(uavInfoIn(state));

    openMainMenuWithFreshUavInfo(sm, tui);

    EXPECT_NE(indexOf(tui.main_menu_labels, "Land"), -1) << state;
    EXPECT_EQ(indexOf(tui.main_menu_labels, "Takeoff"), -1) << state;
  }
}

//}

/* TEST(UavStatusCore, MainMenuHidesLandAndTakeoffWhenMrsIsNotInCommand) //{ */

TEST(UavStatusCore, MainMenuHidesLandAndTakeoffWhenMrsIsNotInCommand) {
  // a pilot flying, no link, and unknown/unrecognised states
  for (const std::string state : {"MANUAL", "NO_LINK", "UNKNOWN", "unknown", ""}) {
    SinkLog       log;
    UavStatusCore sm(makeSink(log), defaultParams());
    FakeTui       tui;

    sm.onControlInfo(flyingControlInfo());
    sm.onUavInfo(uavInfoIn(state));

    openMainMenuWithFreshUavInfo(sm, tui);

    EXPECT_EQ(indexOf(tui.main_menu_labels, "Land"), -1) << state;
    EXPECT_EQ(indexOf(tui.main_menu_labels, "Takeoff"), -1) << state;
  }
}

//}

/* TEST(UavStatusCore, MainMenuHidesLandAndTakeoffWithoutFreshUavInfo) //{ */

TEST(UavStatusCore, MainMenuHidesLandAndTakeoffWithoutFreshUavInfo) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  // The last UavInfo said HOVER, but it has gone stale (or never arrived).
  sm.onControlInfo(flyingControlInfo());
  sm.onUavInfo(uavInfoIn("HOVER"));

  tick(sm, tui, 'm');

  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(indexOf(tui.main_menu_labels, "Land"), -1);
  EXPECT_EQ(indexOf(tui.main_menu_labels, "Takeoff"), -1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Set Constraints"), -1);
}

//}

/* TEST(UavStatusCore, SelectingASetterRowOpensASubmenuWithTheActiveOptionFirst) //{ */

TEST(UavStatusCore, SelectingASetterRowOpensASubmenuWithTheActiveOptionFirst) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.show_sub_menu_calls, 1);
  EXPECT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"medium", "slow", "fast"}));
}

//}

/* TEST(UavStatusCore, SubmenuLabelsAreBuiltAtSelectionTimeNotMenuBuildTime) //{ */

TEST(UavStatusCore, SubmenuLabelsAreBuiltAtSelectionTimeNotMenuBuildTime) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  // A new ControlInfo arrives while the menu is open.
  ControlInfoData ci       = flyingControlInfo();
  ci.active_constraints    = "slow";
  ci.available_constraints = {"slow", "fast"};
  sm.onControlInfo(ci);

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"slow", "fast"}));
}

//}

/* TEST(UavStatusCore, SubmenuSelectionCallsTheServiceAndClosesTheWholeMenu) //{ */

TEST(UavStatusCore, SubmenuSelectionCallsTheServiceAndClosesTheWholeMenu) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));
  ASSERT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"medium", "slow", "fast"}));

  tui.next_menu_event = selectSub(2); // "fast"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"setConstraints(fast)"}));
  EXPECT_EQ(tui.service_results, (std::vector<std::string>{"OK: fast"}));
  EXPECT_EQ(tui.close_sub_menu_calls, 1);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

//}

/* TEST(UavStatusCore, SubmenuCancelRowBacksOutToTheMainMenu) //{ */

TEST(UavStatusCore, SubmenuCancelRowBacksOutToTheMainMenu) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  sm.onUavInfo(uavInfoIn("HOVER"));
  openMainMenuWithFreshUavInfo(sm, tui);

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Land"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));
  ASSERT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"CANCEL", "Land"}));

  tui.next_menu_event = selectSub(0); // "CANCEL"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_TRUE(log.calls.empty());
  EXPECT_EQ(tui.close_sub_menu_calls, 1);

  tui.next_menu_event = selectSub(1); // "Land"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"Land"}));
}

//}

/* TEST(UavStatusCore, EscapeInASubmenuClosesOnlyTheSubmenu) //{ */

TEST(UavStatusCore, EscapeInASubmenuClosesOnlyTheSubmenu) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Gains"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  tui.next_menu_event = tui::MenuEvent{tui::MenuEvent::Kind::Exit, true, -1};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.close_sub_menu_calls, 1);
  EXPECT_EQ(tui.clear_menus_calls, 0);
}

//}

/* TEST(UavStatusCore, EscapeInTheMainMenuClosesEverything) //{ */

TEST(UavStatusCore, EscapeInTheMainMenuClosesEverything) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');
  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);

  tui.next_menu_event = tui::MenuEvent{tui::MenuEvent::Kind::Exit, false, -1};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

//}

// | -------------------------- Goto --------------------------- |

/* TEST(UavStatusCore, GotoMenuSeedsFromParamsAndLabelsTheCurrentFrame) //{ */

TEST(UavStatusCore, GotoMenuSeedsFromParamsAndLabelsTheCurrentFrame) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  StateEstimationInfoData est;
  est.frame_id = "uav1/world_origin";
  sm.onStateEstimationInfo(est);

  tick(sm, tui, 'g');

  ASSERT_EQ(sm.state(), StatusState::GOTO_MENU);
  EXPECT_EQ(tui.show_goto_menu_calls, 1);
  ASSERT_EQ(tui.goto_menu_labels.size(), 5u);
  EXPECT_EQ(tui.goto_menu_labels[4], " uav1/world_origin ");
  EXPECT_EQ(tui.goto_initial_values, (std::vector<double>{0.0, 0.0, 2.0, 1.57}));
}

//}

/* TEST(UavStatusCore, GotoCommitSendsTheReferenceInTheCurrentFrameAndIsRemembered) //{ */

TEST(UavStatusCore, GotoCommitSendsTheReferenceInTheCurrentFrameAndIsRemembered) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  StateEstimationInfoData est;
  est.frame_id = "uav1/world_origin";
  sm.onStateEstimationInfo(est);

  tick(sm, tui, 'g');

  tui.next_goto_event = tui::GotoEvent{tui::GotoEvent::Kind::Committed, 1.5, -2.5, 3.0, 0.75};
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendGoto"}));
  EXPECT_DOUBLE_EQ(log.goto_x, 1.5);
  EXPECT_DOUBLE_EQ(log.goto_y, -2.5);
  EXPECT_DOUBLE_EQ(log.goto_z, 3.0);
  EXPECT_DOUBLE_EQ(log.goto_hdg, 0.75);
  EXPECT_EQ(log.goto_frame, "uav1/world_origin");
  EXPECT_EQ(tui.service_results, (std::vector<std::string>{"OK: goto ok"}));

  // Reopening offers what was last entered, not the config defaults.
  tui.next_goto_event = tui::GotoEvent{};
  tick(sm, tui, 'g');
  EXPECT_EQ(tui.goto_initial_values, (std::vector<double>{1.5, -2.5, 3.0, 0.75}));
}

//}

/* TEST(UavStatusCore, GotoEscapeClosesWithoutCallingTheService) //{ */

TEST(UavStatusCore, GotoEscapeClosesWithoutCallingTheService) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, 'g');

  tui.next_goto_event = tui::GotoEvent{tui::GotoEvent::Kind::Exit, 0.0, 0.0, 0.0, 0.0};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_TRUE(log.calls.empty());
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

//}

// | ------------------------- Remote -------------------------- |

/* TEST(UavStatusCore, EntersRemoteModeOnRWhenFlyingNormally) //{ */

TEST(UavStatusCore, EntersRemoteModeOnRWhenFlyingNormally) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_TRUE(tui.remote_mode_active);
}

//}

/* TEST(UavStatusCore, DoesNotEnterRemoteModeWhenNotFlyingNormally) //{ */

TEST(UavStatusCore, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  tick(sm, tui, 'R'); // flying_normally defaults to false

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

//}

/* TEST(UavStatusCore, ExitsRemoteModeOnSecondR) //{ */

TEST(UavStatusCore, ExitsRemoteModeOnSecondR) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

//}

/* TEST(UavStatusCore, ExitsRemoteModeOnEscape) //{ */

TEST(UavStatusCore, ExitsRemoteModeOnEscape) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

//}

/* TEST(UavStatusCore, RemoteMotionSendsVelocityInTheFcuFrameAndDrawsTheBanner) //{ */

TEST(UavStatusCore, RemoteMotionSendsVelocityInTheFcuFrameAndDrawsTheBanner) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'w');

  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendVelocityReference"}));
  EXPECT_DOUBLE_EQ(log.vx, 2.0);
  EXPECT_DOUBLE_EQ(log.vy, 0.0);
  EXPECT_EQ(log.velocity_frame, "uav1/fcu_untilted");
  EXPECT_GE(tui.remote_banner_calls, 1);
  EXPECT_FALSE(tui.remote_banner_turbo);
  EXPECT_FALSE(tui.remote_banner_global);
}

//}

/* TEST(UavStatusCore, ArrowKeysMapToTheSameMotionsAsWasd) //{ */

TEST(UavStatusCore, ArrowKeysMapToTheSameMotionsAsWasd) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, static_cast<int>(tui::Key::Right));

  EXPECT_DOUBLE_EQ(log.vy, -2.0);
}

//}

/* TEST(UavStatusCore, GlobalToggleSwitchesTheVelocityFrame) //{ */

TEST(UavStatusCore, GlobalToggleSwitchesTheVelocityFrame) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'G');
  tick(sm, tui, 'w');

  EXPECT_EQ(log.velocity_frame, "uav1/world_origin");
  EXPECT_TRUE(tui.remote_banner_global);
}

//}

/* TEST(UavStatusCore, TurboSwapsConstraintsAndRestoresThePreviousSet) //{ */

TEST(UavStatusCore, TurboSwapsConstraintsAndRestoresThePreviousSet) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onControlInfo(flyingControlInfo()); // active_constraints == "medium"

  tick(sm, tui, 'R');
  tick(sm, tui, 'T');

  EXPECT_EQ(log.calls, (std::vector<std::string>{"setConstraints(fast)"}));

  // renderRemoteBanner() runs at the top of remoteHandler(), before the key is applied, so the
  // new turbo state only shows on the following tick (matches the real render loop, and predates
  // this refactor -- see NcursesTui::remoteHandler()/drawRemoteBanner() before the logic moved here).
  tick(sm, tui, 'w');
  EXPECT_DOUBLE_EQ(log.vx, 5.0); // turbo steps are larger
  EXPECT_TRUE(tui.remote_banner_turbo);

  tick(sm, tui, 'T');
  EXPECT_EQ(log.calls.back(), "setConstraints(medium)");

  tick(sm, tui, 'w');
  EXPECT_DOUBLE_EQ(log.vx, 2.0); // back to the non-turbo step
  EXPECT_FALSE(tui.remote_banner_turbo);
}

//}

/* TEST(UavStatusCore, UnmappedRemoteKeyHoversOnceAfterAMotionCommand) //{ */

TEST(UavStatusCore, UnmappedRemoteKeyHoversOnceAfterAMotionCommand) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'z'); // nothing flown yet -> no hover
  EXPECT_TRUE(log.calls.empty());

  tick(sm, tui, 'w');
  tick(sm, tui, 'z');
  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendVelocityReference", "hover"}));

  tick(sm, tui, 'z'); // hover is not repeated until something is flown again
  EXPECT_EQ(log.calls.size(), 2u);
}

//}

// | --------------------- Snapshot / strings ------------------ |

/* TEST(UavStatusCore, SnapshotCarriesLatestDataFreshnessAndBorderStatus) //{ */

TEST(UavStatusCore, SnapshotCarriesLatestDataFreshnessAndBorderStatus) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());

  GeneralRobotInfoData gri;
  gri.robot_name                  = "uav1";
  gri.ready_for_automatic_takeoff = true;
  sm.onGeneralRobotInfo(gri);

  ControlInfoData ci;
  ci.active_tracker = "NullTracker";
  sm.onControlInfo(ci);

  CollisionAvoidanceInfoData cai;
  cai.bumper_active = true;
  sm.onCollisionAvoidanceInfo(cai);

  Freshness freshness;
  freshness.general_robot_info = true;
  sm.setFreshness(freshness);

  const RenderSnapshot snap = sm.snapshot(12.5);

  EXPECT_DOUBLE_EQ(snap.now_seconds, 12.5);
  EXPECT_EQ(snap.general_robot_info.robot_name, "uav1");
  EXPECT_TRUE(snap.freshness.general_robot_info);
  EXPECT_FALSE(snap.freshness.uav_info);
  EXPECT_TRUE(snap.border_status.null_tracker);
  EXPECT_TRUE(snap.border_status.can_takeoff);
  EXPECT_TRUE(snap.border_status.bumper_active);
}

//}

/* TEST(UavStatusCore, DisplayStringIsStoredWithoutItsFlagPreamble) //{ */

TEST(UavStatusCore, DisplayStringIsStoredWithoutItsFlagPreamble) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());

  sm.onString(1.0, "-id gps -p hello world");

  const RenderSnapshot snap = sm.snapshot(1.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "hello world");
}

//}

/* TEST(UavStatusCore, DisplayStringIsDedupedById) //{ */

TEST(UavStatusCore, DisplayStringIsDedupedById) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());

  sm.onString(1.0, "-id gps first");
  sm.onString(2.0, "-id gps second");

  const RenderSnapshot snap = sm.snapshot(2.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "second");
}

//}

/* TEST(UavStatusCore, NonPersistentDisplayStringExpiresAfterTenSeconds) //{ */

TEST(UavStatusCore, NonPersistentDisplayStringExpiresAfterTenSeconds) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onString(1.0, "-id gps transient");
  ASSERT_EQ(sm.snapshot(1.0).display_strings.size(), 1u);

  tick(sm, tui, -1, 12.0); // pruning runs at the top of tick()

  EXPECT_TRUE(sm.snapshot(12.0).display_strings.empty());
}

//}

/* TEST(UavStatusCore, PersistentDisplayStringSurvivesExpiry) //{ */

TEST(UavStatusCore, PersistentDisplayStringSurvivesExpiry) {
  SinkLog       log;
  UavStatusCore sm(makeSink(log), defaultParams());
  FakeTui       tui;

  sm.onString(1.0, "-id gps -p forever");

  tick(sm, tui, -1, 120.0);

  ASSERT_EQ(sm.snapshot(120.0).display_strings.size(), 1u);
  EXPECT_EQ(sm.snapshot(120.0).display_strings[0], "forever");
}

//}

/* flight state colour //{ */

TEST(Helpers, FlightStateHighlightsOnlyUnhealthyStates) {
  // DiagnosticsManager's UavInfo.flight_state: normal ground and flight states are not highlighted
  for (const std::string state : {"DISARMED", "ARMED", "OFFBOARD", "TAKEOFF", "HOVER", "GOTO", "TRAJECTORY", "LAND", "MIDAIR", "RC_MODE"}) {
    EXPECT_FALSE(utils::isFlightStateUnhealthy(state)) << state;
  }
  // no link, a pilot flying, emergencies, and anything not recognised (incl. the "unknown" default)
  for (const std::string state : {"UNKNOWN", "NO_LINK", "MANUAL", "EHOVER", "ELAND", "FAILSAFE", "unknown", ""}) {
    EXPECT_TRUE(utils::isFlightStateUnhealthy(state)) << state;
  }
}

//}

} // namespace mrs_uav_status::status
