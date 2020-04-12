/* INCLUDES //{ */

#include <status_window.h>
#include <menu.h>
#include <input_box.h>
#include <commons.h>

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include <topic_tools/shape_shifter.h>  // for generic topic subscribers

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/GainManagerDiagnostics.h>
#include <mrs_msgs/ConstraintManagerDiagnostics.h>

#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/String.h>

#include <std_msgs/String.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/BatteryState.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

using namespace std;
using topic_tools::ShapeShifter;

//}

typedef enum
{
  STANDARD,
  RETARD,
  MAIN_MENU,
  GOTO_MENU,
} status_state;

/* class MrsStatus //{ */

class MrsStatus {

public:
  MrsStatus();

private:
  ros::NodeHandle nh_;

  ros::Timer status_timer_;

  void UavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  void MavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  void MavrosAttitudeCallback(const mavros_msgs::AttitudeTargetConstPtr& msg);
  void BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg);
  void ControlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  void GainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg);
  void ConstraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg);
  void StringCallback(const ros::MessageEvent<std_msgs::String const>& event);
  void SetServiceCallback(const std_msgs::String& msg);

  void GenericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id);

  void statusTimer(const ros::TimerEvent& event);

  void PrintLimitedInt(WINDOW* win, int y, int x, string str_in, int num, int limit);
  void PrintLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit);
  void PrintLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit);
  void PrintServiceResult(bool success, string msg);

  void PrintNoData(WINDOW* win, int y, int x);
  void PrintNoData(WINDOW* win, int y, int x, string text);

  void PrintCpuLoad(WINDOW* win);
  void PrintCpuFreq(WINDOW* win);
  void PrintMemLoad(WINDOW* win);
  void PrintDiskSpace(WINDOW* win);

  void GenericTopicHandler(WINDOW* win, double rate, short color, int topic);
  void UavStateHandler(WINDOW* win, double rate, short color, int topic);
  void MavrosStateHandler(WINDOW* win, double rate, short color, int topic);
  void ControlManagerHandler(WINDOW* win, double rate, short color, int topic);
  void GeneralInfoHandler(WINDOW* win, double rate, short color, int topic);
  void StringHandler(WINDOW* win, double rate, short color, int topic);

  void flightTimeHandler(WINDOW* win);

  void SetupMainMenu();
  bool MainMenuHandler(int key_in);

  void SetupGotoMenu();
  bool GotoMenuHandler(int key_in);

  void RetardHandler(int key, WINDOW* win);

  ros::Subscriber uav_state_subscriber_;
  ros::Subscriber mpc_diag_subscriber_;

  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber mavros_attitude_subscriber_;
  ros::Subscriber battery_subscriber_;
  ros::Subscriber control_manager_subscriber_;
  ros::Subscriber gain_manager_subscriber_;
  ros::Subscriber constraint_manager_subscriber_;

  ros::Subscriber string_subscriber_;
  ros::Subscriber set_service_subscriber_;

  ros::ServiceClient service_goto_reference_;
  ros::ServiceClient service_goto_fcu_;
  ros::ServiceClient service_set_constraints_;
  ros::ServiceClient service_set_gains_;
  ros::ServiceClient service_set_controller_;
  ros::ServiceClient service_hover_;

  string _uav_name_;
  string _uav_type_;
  string _sensors_;
  bool   _pixgarm_;

  bool initialized_ = false;

  long last_idle_  = 0;
  long last_total_ = 0;
  long last_gigas_ = 0;

  mrs_msgs::UavState uav_state_;
  /* shared_ptr<int> uav_state_counter_ptr_ = make_shared<int>(0); */

  mavros_msgs::State          mavros_state_;
  mavros_msgs::AttitudeTarget mavros_attitude_;
  sensor_msgs::BatteryState   battery_;

  mrs_msgs::ControlManagerDiagnostics    control_manager_;
  mrs_msgs::GainManagerDiagnostics       gain_manager_;
  mrs_msgs::ConstraintManagerDiagnostics constraint_manager_;

  StatusWindow* uav_state_window_;
  vector<topic> uav_state_topic_;

  StatusWindow* mavros_state_window_;
  vector<topic> mavros_state_topic_;

  StatusWindow* control_manager_window_;
  vector<topic> control_manager_topic_;

  StatusWindow* general_info_window_;
  vector<topic> general_info_topic_;

  StatusWindow*       string_window_;
  vector<string_info> string_info_vec_;
  vector<topic>       string_topic_;

  WINDOW* top_bar_window_;

  WINDOW*   bottom_window_;
  ros::Time bottom_window_clear_time_ = ros::Time::now();

  WINDOW* tmp_window_;

  StatusWindow*           generic_topic_window_;
  vector<topic>           generic_topic_vec_;
  vector<string>          generic_topic_input_vec_;
  vector<ros::Subscriber> generic_subscriber_vec_;

  vector<Menu> menu_vec_;
  vector<Menu> submenu_vec_;

  vector<service> service_vec_;
  vector<string>  service_input_vec_;
  vector<string>  main_menu_text_;
  vector<string>  constraints_text_;
  vector<string>  gains_text_;
  vector<string>  controllers_text_;

  vector<double>   goto_double_vec_;
  vector<string>   goto_menu_text_;
  vector<InputBox> goto_menu_inputs_;

  bool retard_hover_ = false;
  bool turbo_retard_ = false;

  string old_constraints;

  bool is_flying_   = false;
  bool NullTracker_ = true;

  unsigned long     secs_flown = 0;
  ros::Time         last_flight_time_;
  const std::string _time_filename_ = "/tmp/mrs_status_flight_time.txt";


  status_state state = STANDARD;
};

//}

/* MrsStatus() //{ */

MrsStatus::MrsStatus() {

  // initialize node and create no handle
  nh_ = ros::NodeHandle("~");

  mrs_lib::ParamLoader param_loader(nh_, "MrsStatus");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("uav_type", _uav_type_);
  param_loader.loadParam("sensors", _sensors_);
  param_loader.loadParam("pixgarm", _pixgarm_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsStatus]: Could not load all parameters!");
    ros::shutdown();
  } else {
    ROS_INFO("[MrsStatus]: All params loaded!");
  }

  // TIMERS
  status_timer_ = nh_.createTimer(ros::Rate(10), &MrsStatus::statusTimer, this);

  // SUBSCRIBERS
  uav_state_subscriber_          = nh_.subscribe("uav_state_in", 1, &MrsStatus::UavStateCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_state_subscriber_       = nh_.subscribe("mavros_state_in", 1, &MrsStatus::MavrosStateCallback, this, ros::TransportHints().tcpNoDelay());
  mavros_attitude_subscriber_    = nh_.subscribe("mavros_attitude_in", 1, &MrsStatus::MavrosAttitudeCallback, this, ros::TransportHints().tcpNoDelay());
  battery_subscriber_            = nh_.subscribe("battery_in", 1, &MrsStatus::BatteryCallback, this, ros::TransportHints().tcpNoDelay());
  control_manager_subscriber_    = nh_.subscribe("control_manager_in", 1, &MrsStatus::ControlManagerCallback, this, ros::TransportHints().tcpNoDelay());
  gain_manager_subscriber_       = nh_.subscribe("gain_manager_in", 1, &MrsStatus::GainManagerCallback, this, ros::TransportHints().tcpNoDelay());
  constraint_manager_subscriber_ = nh_.subscribe("constraint_manager_in", 1, &MrsStatus::ConstraintManagerCallback, this, ros::TransportHints().tcpNoDelay());
  string_subscriber_             = nh_.subscribe("string_in", 1, &MrsStatus::StringCallback, this, ros::TransportHints().tcpNoDelay());
  set_service_subscriber_        = nh_.subscribe("set_service_in", 1, &MrsStatus::SetServiceCallback, this, ros::TransportHints().tcpNoDelay());

  // SERVICES
  service_goto_reference_  = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference_out");
  service_goto_fcu_        = nh_.serviceClient<mrs_msgs::Vec4>("goto_fcu_out");
  service_set_constraints_ = nh_.serviceClient<mrs_msgs::String>("set_constraints_out");
  service_set_gains_       = nh_.serviceClient<mrs_msgs::String>("set_gains_out");
  service_set_controller_  = nh_.serviceClient<mrs_msgs::String>("set_controller_out");
  service_hover_           = nh_.serviceClient<std_srvs::Trigger>("hover_out");

  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(0.0);
  goto_double_vec_.push_back(2.0);
  goto_double_vec_.push_back(1.57);


  service_input_vec_.push_back("uav_manager/land Land");
  service_input_vec_.push_back("uav_manager/land_home Land Home");
  service_input_vec_.push_back("uav_manager/takeoff Takeoff");

  if (boost::filesystem::exists(_time_filename_)) {
    ifstream file(_time_filename_);
    string   line;
    getline(file, line);

    try {
      secs_flown = stoi(line);
    }
    catch (const invalid_argument& e) {
      secs_flown = 0;
    }
    file.close();
  }

  /* Generic topic definitions //{ */

  vector<string> results;
  split(results, _sensors_, boost::is_any_of(", "), boost::token_compress_on);

  vector<string> generic_topic_input_vec_;

  if (_pixgarm_) {
    generic_topic_input_vec_.push_back("mavros/distance_sensor/garmin Garmin_pix 80+");
  }

  for (int i = 0; i < results.size(); i++) {
    if (results[i] == "garmin_down" && _pixgarm_ == false) {
      generic_topic_input_vec_.push_back("garmin/range Garmin_Down 80+");

    } else if (results[i] == "garmin_up") {
      generic_topic_input_vec_.push_back("garmin/range_up Garmin_Up 80+");

    } else if (results[i] == "realsense_brick") {
      generic_topic_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Brick 25+");

    } else if (results[i] == "realsense_front") {
      generic_topic_input_vec_.push_back("rs_d435/depth/camera_info Realsense_Front 25+");

    } else if (results[i] == "bluefox_brick") {
      generic_topic_input_vec_.push_back("bluefox_brick/camera_info Bluefox_Brick 25+");

    } else if (results[i] == "bluefox_optflow") {
      generic_topic_input_vec_.push_back("bluefox_optflow/camera_info Bluefox_Optflow 60+");
      generic_topic_input_vec_.push_back("optic_flow/velocity Optic_flow 60+");

    } else if (results[i] == "trinocular_thermal") {
      generic_topic_input_vec_.push_back("thermal/top/rgb_image Thermal_Top 15+");
      generic_topic_input_vec_.push_back("thermal/middle/rgb_image Thermal_Middle 15+");
      generic_topic_input_vec_.push_back("thermal/bottom/rgb_image Thermal_Bottom 15+");

    } else if (results[i] == "rplidar") {
      generic_topic_input_vec_.push_back("rplidar/scan Rplidar 10+");
    }
  }

  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;  // generic callback

  for (unsigned long i = 0; i < generic_topic_input_vec_.size(); i++) {

    vector<string> results;
    boost::split(results, generic_topic_input_vec_[i], [](char c) { return c == ' '; });  // split the input string into words and put them in results vector
    if (results[2].back() == '+') {
      // TODO handle the + sign
      results[2].pop_back();
    }

    topic tmp_topic(results[0], results[1], stoi(results[2]));

    generic_topic_vec_.push_back(tmp_topic);

    int    id         = i;  // id to identify which topic called the generic callback
    string topic_name = "/" + _uav_name_ + "/" + generic_topic_vec_[i].topic_name;

    callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { GenericCallback(msg, topic_name, id); };
    ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

    generic_subscriber_vec_.push_back(tmp_subscriber);
  }

  //}


  uav_state_topic_.push_back(topic{100.0});

  uav_state_window_ = new StatusWindow(6, 30, 5, 1, uav_state_topic_);

  control_manager_topic_.push_back(topic{10.0});
  control_manager_topic_.push_back(topic{1.0});
  control_manager_topic_.push_back(topic{1.0});

  control_manager_window_ = new StatusWindow(4, 30, 1, 1, control_manager_topic_);

  mavros_state_topic_.push_back(topic{100.0});
  mavros_state_topic_.push_back(topic{1.0});
  mavros_state_topic_.push_back(topic{100.0});

  mavros_state_window_ = new StatusWindow(6, 30, 5, 31, mavros_state_topic_);

  general_info_window_ = new StatusWindow(4, 30, 1, 31, general_info_topic_);

  generic_topic_window_ = new StatusWindow(10, 30, 1, 61, generic_topic_vec_);

  string_window_ = new StatusWindow(10, 32, 1, 91, string_topic_);

  top_bar_window_ = newwin(1, 120, 0, 1);
  bottom_window_  = newwin(1, 120, 11, 1);

  tmp_window_ = newwin(10, 120, 12, 1);

  initialized_ = true;
  ROS_INFO("[MrsStatus]: Node initialized!");

  refresh();
}

//}

/* statusTimer //{ */

void MrsStatus::statusTimer([[maybe_unused]] const ros::TimerEvent& event) {

  wclear(top_bar_window_);

  if ((ros::Time::now() - bottom_window_clear_time_).toSec() > 3.0) {
    wclear(bottom_window_);
  }

  flightTimeHandler(top_bar_window_);

  uav_state_window_->Redraw(&MrsStatus::UavStateHandler, this);
  mavros_state_window_->Redraw(&MrsStatus::MavrosStateHandler, this);
  control_manager_window_->Redraw(&MrsStatus::ControlManagerHandler, this);
  generic_topic_window_->Redraw(&MrsStatus::GenericTopicHandler, this);
  general_info_window_->Redraw(&MrsStatus::GeneralInfoHandler, this);
  string_window_->Redraw(&MrsStatus::StringHandler, this);

  int key_in = getch();

  switch (state) {

    case STANDARD:

      switch (key_in) {

        case 'R':
          retard_hover_ = false;
          state         = RETARD;
          break;

        case 'm':
          SetupMainMenu();
          state = MAIN_MENU;
          break;

        case 'g':
          SetupGotoMenu();
          state = GOTO_MENU;
          break;
        default:
          flushinp();
          break;
      }

      break;

    case RETARD:
      flushinp();
      RetardHandler(key_in, top_bar_window_);
      if (key_in == 'R' || key_in == KEY_ESC) {

        if (turbo_retard_) {

          turbo_retard_ = false;
          mrs_msgs::String string_service;
          string_service.request.value = old_constraints;
          service_set_constraints_.call(string_service);
          PrintServiceResult(string_service.response.success, string_service.response.message);
        }

        state = STANDARD;
      }
      break;

    case MAIN_MENU:
      if (MainMenuHandler(key_in)) {
        state = STANDARD;
      }
      break;

    case GOTO_MENU:
      if (GotoMenuHandler(key_in)) {
        state = STANDARD;
      }
      break;
  }

  refresh();
  wrefresh(top_bar_window_);
  wrefresh(bottom_window_);
  wrefresh(tmp_window_);
}


//}

/* HANDLERS //{ */


/* MainMenuHandler() //{ */

bool MrsStatus::MainMenuHandler(int key_in) {

  /* SUBMENU IS OPEN //{ */

  if (!submenu_vec_.empty()) {
    // SUBMENU IS OPEN

    menu_vec_[0].iterate(main_menu_text_, -1, true);
    optional<tuple<int, int>> ret;

    switch (submenu_vec_[0].getId()) {
      case 1:
        // CONSTRAINTS
        ret = submenu_vec_[0].iterate(constraints_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = constraints_text_[line];
            service_set_constraints_.call(string_service);
            PrintServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;

      case 2:
        // GAINS
        ret = submenu_vec_[0].iterate(gains_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = gains_text_[line];
            service_set_gains_.call(string_service);
            PrintServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;

      case 3:
        // CONTROLLERS
        ret = submenu_vec_[0].iterate(controllers_text_, key_in, true);

        if (ret.has_value()) {

          int line = get<0>(ret.value());
          int key  = get<1>(ret.value());

          if (line == 666 && key == 666) {

            submenu_vec_.clear();
            return false;

          } else if (key == KEY_ENT) {

            mrs_msgs::String string_service;
            string_service.request.value = controllers_text_[line];
            service_set_controller_.call(string_service);
            PrintServiceResult(string_service.response.success, string_service.response.message);

            submenu_vec_.clear();
            return true;
          }
        }
        break;
    }

    return false;

    //}

    /* NORMAL CASE //{ */

  } else {
    // NORMAL CASE - NO SUBMENU

    optional<tuple<int, int>> ret = menu_vec_[0].iterate(main_menu_text_, key_in, true);

    if (ret.has_value()) {

      int line = get<0>(ret.value());
      int key  = get<1>(ret.value());

      if (line == 666 && key == 666) {

        menu_vec_.clear();
        return true;

      } else if (key == KEY_ENT) {

        if (line < service_vec_.size()) {

          std_srvs::Trigger trig;
          service_vec_[line].service_client.call(trig);
          PrintServiceResult(trig.response.success, trig.response.message);

          menu_vec_.clear();
          return true;

        } else if (line == main_menu_text_.size() - 3) {
          // SET CONSTRAINTS

          constraints_text_.clear();

          for (unsigned long i = 0; i < constraint_manager_.available.size(); i++) {
            constraints_text_.push_back(constraint_manager_.available[i]);
          }

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, constraints_text_, 1);
          submenu_vec_.push_back(menu);

        } else if (line == main_menu_text_.size() - 2) {
          // SET GAINS

          gains_text_.clear();

          for (unsigned long i = 0; i < gain_manager_.available.size(); i++) {
            gains_text_.push_back(gain_manager_.available[i]);
          }

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, gains_text_, 2);
          submenu_vec_.push_back(menu);

        } else if (line == main_menu_text_.size() - 1) {
          // SET CONTROLLER

          controllers_text_.clear();
          controllers_text_.push_back("So3Controller");
          controllers_text_.push_back("MpcController");

          int x;
          int y;
          int rows;
          int cols;

          getyx(menu_vec_[0].getWin(), x, y);
          getmaxyx(menu_vec_[0].getWin(), rows, cols);

          Menu menu(x, 31 + cols, controllers_text_, 3);
          submenu_vec_.push_back(menu);

        } else {
          PrintServiceResult(false, "undefined");
        }
      }
    }
    return false;
  }

  //}
}

//}

/* GotoMenuHandler() //{ */

bool MrsStatus::GotoMenuHandler(int key_in) {

  optional<tuple<int, int>> ret = menu_vec_[0].iterate(goto_menu_text_, key_in, false);

  if (ret.has_value()) {
    int line = get<0>(ret.value());
    int key  = get<1>(ret.value());

    if (line == 666 && key == 666) {

      menu_vec_.clear();
      return true;

    } else if (key == KEY_ENT) {

      goto_double_vec_[0] = goto_menu_inputs_[0].getDouble();
      goto_double_vec_[1] = goto_menu_inputs_[1].getDouble();
      goto_double_vec_[2] = goto_menu_inputs_[2].getDouble();
      goto_double_vec_[3] = goto_menu_inputs_[3].getDouble();

      mrs_msgs::ReferenceStampedSrv reference;

      reference.request.reference.position.x = goto_double_vec_[0];
      reference.request.reference.position.y = goto_double_vec_[1];
      reference.request.reference.position.z = goto_double_vec_[2];
      reference.request.reference.heading    = goto_double_vec_[3];
      reference.request.header.frame_id      = uav_state_.header.frame_id;
      reference.request.header.stamp         = ros::Time::now();

      service_goto_reference_.call(reference);

      PrintServiceResult(reference.response.success, reference.response.message);

      menu_vec_.clear();
      return true;

    } else if (line < goto_menu_inputs_.size()) {

      goto_menu_inputs_[line].Process(key);
    }
  }


  for (unsigned long i = 0; i < goto_menu_inputs_.size(); i++) {
    if (i == menu_vec_[0].getLine()) {
      goto_menu_inputs_[i].Print(i + 1, true);
    } else {
      goto_menu_inputs_[i].Print(i + 1, false);
    }
  }

  wrefresh(menu_vec_[0].getWin());
}

//}

/* RetardHandler() //{ */

void MrsStatus::RetardHandler(int key, WINDOW* win) {

  wattron(win, A_BOLD);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, 0, 20, "RETARD MODE IS ACTIVE, YOU HAVE CONTROL");

  if (turbo_retard_) {
    wattron(win, A_BLINK);
    mvwprintw(win, 0, 12, "!TURBO!");
    mvwprintw(win, 0, 60, "!TURBO!");
    wattroff(win, A_BLINK);
  }

  wattroff(win, COLOR_PAIR(RED));

  mrs_msgs::Vec4    goal;
  mrs_msgs::String  string_service;
  std_srvs::Trigger trig;

  goal.request.goal[0] = 0.0;
  goal.request.goal[1] = 0.0;
  goal.request.goal[2] = 0.0;
  goal.request.goal[3] = 0.0;

  switch (key) {

    case 'w':
    case 'k':
    case KEY_UP:
      goal.request.goal[0] = 2.0;

      if (turbo_retard_) {
        goal.request.goal[0] = 5.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 's':
    case 'j':
    case KEY_DOWN:
      goal.request.goal[0] = -2.0;

      if (turbo_retard_) {
        goal.request.goal[0] = -5.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'a':
    case 'h':
    case KEY_LEFT:
      goal.request.goal[1] = 2.0;

      if (turbo_retard_) {
        goal.request.goal[1] = 5.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'd':
    case 'l':
    case KEY_RIGHT:
      goal.request.goal[1] = -2.0;

      if (turbo_retard_) {
        goal.request.goal[1] = -5.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'r':
      goal.request.goal[2] = 1.0;

      if (turbo_retard_) {
        goal.request.goal[2] = 2.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'f':
      goal.request.goal[2] = -1.0;

      if (turbo_retard_) {
        goal.request.goal[2] = -2.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'q':
      goal.request.goal[3] = 0.5;

      if (turbo_retard_) {
        goal.request.goal[3] = 1.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'e':
      goal.request.goal[3] = -0.5;

      if (turbo_retard_) {
        goal.request.goal[3] = -1.0;
      }

      service_goto_fcu_.call(goal);
      retard_hover_ = true;
      break;

    case 'T':

      if (turbo_retard_) {

        turbo_retard_                = false;
        string_service.request.value = old_constraints;
        service_set_constraints_.call(string_service);
        PrintServiceResult(string_service.response.success, string_service.response.message);

      } else {

        turbo_retard_                = true;
        old_constraints              = constraint_manager_.current_name;
        string_service.request.value = constraint_manager_.available[constraint_manager_.available.size() - 1];
        service_set_constraints_.call(string_service);
        PrintServiceResult(string_service.response.success, string_service.response.message);
      }

      break;

    default:
      if (retard_hover_) {

        service_hover_.call(trig);
        retard_hover_ = false;
      }
      break;
  }
  wattroff(win, A_BOLD);
}

//}

/* StringHandler() //{ */

void MrsStatus::StringHandler(WINDOW* win, double rate, short color, int topic) {

  if (string_info_vec_.empty()) {
    wclear(win);
  }

  for (unsigned long i = 0; i < string_info_vec_.size(); i++) {

    if ((ros::Time::now() - string_info_vec_[i].last_time).toSec() > 10.0) {
      string_info_vec_.erase(string_info_vec_.begin() + i);
    } else {

      PrintLimitedString(win, 1 + (3 * i), 1, string_info_vec_[i].publisher_name + ": ", 30);

      int    tmp_color          = NORMAL;
      string tmp_display_string = string_info_vec_[i].display_string;

      if (tmp_display_string.at(0) == '-') {

        if (tmp_display_string.at(1) == 'r') {
          tmp_color = RED;
        }

        else if (tmp_display_string.at(1) == 'y') {
          tmp_color = YELLOW;
        }

        else if (tmp_display_string.at(1) == 'g') {
          tmp_color = GREEN;
        }

        if (tmp_color != NORMAL) {
          tmp_display_string.erase(0, 3);
        }
      }

      wattron(win, COLOR_PAIR(tmp_color));
      PrintLimitedString(win, 2 + (3 * i), 1, tmp_display_string, 30);
      wattroff(win, COLOR_PAIR(tmp_color));
    }
  }
}

//}

/* GeneralInfoHandler() //{ */

void MrsStatus::GeneralInfoHandler(WINDOW* win, double rate, short color, int topic) {

  PrintCpuLoad(win);
  PrintMemLoad(win);
  PrintCpuFreq(win);
  PrintDiskSpace(win);
}

//}

/* GenericTopicHandler() //{ */

void MrsStatus::GenericTopicHandler(WINDOW* win, double rate, short color, int topic) {

  if (!generic_topic_vec_.empty()) {

    PrintLimitedString(win, 1 + topic, 1, generic_topic_vec_[topic].topic_display_name, 19);
    PrintLimitedDouble(win, 1 + topic, 21, "%5.1f Hz", rate, 1000);

  } else {

    wclear(win);
  }
}

//}

/* UavStateHandler() //{ */

void MrsStatus::UavStateHandler(WINDOW* win, double rate, short color, int topic) {

  PrintLimitedDouble(win, 0, 16, "Odom %5.1f Hz", rate, 1000);

  if (rate == 0) {

    PrintNoData(win, 0, 1);

  } else {

    double heading = mrs_lib::AttitudeConverter(uav_state_.pose.orientation).getHeading();

    PrintLimitedDouble(win, 1, 2, "X %7.2f", uav_state_.pose.position.x, 1000);
    PrintLimitedDouble(win, 2, 2, "Y %7.2f", uav_state_.pose.position.y, 1000);
    PrintLimitedDouble(win, 3, 2, "Z %7.2f", uav_state_.pose.position.z, 1000);
    PrintLimitedDouble(win, 4, 2, "Yaw %5.2f", heading, 1000);

    int pos = uav_state_.header.frame_id.find("/") + 1;
    PrintLimitedString(win, 1, 14, uav_state_.header.frame_id.substr(pos, uav_state_.header.frame_id.length()), 15);

    PrintLimitedString(win, 2, 14, "Hori: " + uav_state_.estimator_horizontal.name, 15);
    PrintLimitedString(win, 3, 14, "Vert: " + uav_state_.estimator_vertical.name, 15);
    PrintLimitedString(win, 4, 14, "Head: " + uav_state_.estimator_heading.name, 15);
  }
}

//}

/* MavrosStateHandler() //{ */

void MrsStatus::MavrosStateHandler(WINDOW* win, double rate, short color, int topic) {

  string tmp_string;

  switch (topic) {
    case 0:  // mavros state
      PrintLimitedDouble(win, 0, 14, "Mavros %5.1f Hz", rate, 1000);

      if (rate == 0) {

        PrintNoData(win, 0, 1);

      } else {

        if (mavros_state_.armed) {
          tmp_string = "ARMED";
        } else {
          tmp_string = "DISARMED";
          wattron(win, COLOR_PAIR(RED));
        }

        PrintLimitedString(win, 1, 1, "State: " + tmp_string, 15);
        wattron(win, COLOR_PAIR(color));

        if (mavros_state_.mode != "OFFBOARD") {
          wattron(win, COLOR_PAIR(RED));
        }

        PrintLimitedString(win, 2, 1, "Mode:  " + mavros_state_.mode, 15);
        wattron(win, COLOR_PAIR(color));
      }

      break;

    case 1:  // battery
      if (rate == 0) {

        PrintNoData(win, 3, 1, "Batt:  ");

      } else {

        double voltage = battery_.voltage;
        (voltage > 17.0) ? (voltage = voltage / 6) : (voltage = voltage / 4);

        if (voltage < 3.6) {
          wattron(win, COLOR_PAIR(RED));
        } else if (voltage < 3.7 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }

        PrintLimitedDouble(win, 3, 1, "Batt:  %4.2f V", voltage, 10);
        wattron(win, COLOR_PAIR(color));

        PrintLimitedDouble(win, 3, 15, "%5.2f A", battery_.current, 100);
      }
      break;

    case 2:  // mavros attitude
      if (rate == 0) {

        PrintNoData(win, 4, 1, "Thrst: ");

      } else {

        if (mavros_attitude_.thrust > 0.75) {
          wattron(win, COLOR_PAIR(RED));
        } else if (mavros_attitude_.thrust > 0.65 && color != RED) {
          wattron(win, COLOR_PAIR(YELLOW));
        }
        PrintLimitedDouble(win, 4, 1, "Thrst: %4.2f", mavros_attitude_.thrust, 1.01);
        wattron(win, COLOR_PAIR(color));
      }
      break;
  }
}

//}

/* ControlManagerHandler() //{ */

void MrsStatus::ControlManagerHandler(WINDOW* win, double rate, short color, int topic) {

  string controller;
  string tracker;
  controller = control_manager_.active_controller;
  tracker    = control_manager_.active_tracker;

  switch (topic) {
    case 0:  // mavros state
      PrintLimitedString(win, 0, 14, "Control Manager", 15);

      if (rate == 0) {

        PrintNoData(win, 0, 1);

        wattron(win, COLOR_PAIR(RED));
        mvwprintw(win, 1, 1, "NO_CONTROLLER");
        mvwprintw(win, 2, 1, "NO_TRACKER");
        wattron(win, COLOR_PAIR(color));

      } else {
        if (controller != "So3Controller") {
          if (controller != "MpcController") {
            wattron(win, COLOR_PAIR(RED));
          }
          PrintLimitedString(win, 1, 1, controller, 29);
        } else {
          mvwprintw(win, 1, 1, controller.c_str());
          wattron(win, COLOR_PAIR(NORMAL));
          mvwprintw(win, 1, 1 + controller.length(), "%s", "/");
        }

        wattron(win, COLOR_PAIR(color));

        if (tracker != "MpcTracker") {
          if (tracker == "LandoffTracker" && color != RED) {
            wattron(win, COLOR_PAIR(YELLOW));
          } else {
            wattron(win, COLOR_PAIR(RED));
          }

          PrintLimitedString(win, 2, 1, tracker, 29);

        } else {
          mvwprintw(win, 2, 1, tracker.c_str());
          wattron(win, COLOR_PAIR(NORMAL));
          mvwprintw(win, 2, 1 + tracker.length(), "%s", "/");
          wattron(win, COLOR_PAIR(color));
        }
      }
      break;

    case 1:  // mavros state

      if (controller == "So3Controller") {

        if (rate == 0) {
          PrintNoData(win, 1, 2 + controller.length());
        } else {
          PrintLimitedString(win, 1, 2 + controller.length(), gain_manager_.current_name, 10);
        }
      }
      break;

    case 2:  // mavros state

      if (tracker == "MpcTracker") {
        if (rate == 0) {
          PrintNoData(win, 2, 2 + tracker.length());
        } else {
          PrintLimitedString(win, 2, 2 + tracker.length(), constraint_manager_.current_name, 10);
        }
      }
      break;
  }
}

//}

/* flightTimeHandler() //{ */

void MrsStatus::flightTimeHandler(WINDOW* win) {

  if (NullTracker_) {

    is_flying_ = false;

  } else {

    if (!is_flying_) {

      is_flying_        = true;
      last_flight_time_ = ros::Time::now();

    } else {

      int secs_passed = int((ros::Time::now() - last_flight_time_).toSec());

      if (secs_passed > 0) {

        secs_flown += secs_passed;
        last_flight_time_ = last_flight_time_ + ros::Duration(secs_passed);

        ofstream outputFile(_time_filename_);
        outputFile << secs_flown;
        outputFile.close();
      }
    }
  }

  wattron(win, A_BOLD);
  PrintLimitedInt(win, 0, 0, "ToF: %i", secs_flown, 1000);

  int mins = secs_flown / 60;
  /* int tens_secs = ((secs_flown % 60) / 10) % 10; */
  int secs = secs_flown % 60;

  mvwprintw(win, 0, 0, "ToF: %i:%02i", mins, secs);
  wattroff(win, A_BOLD);
}

//}


//}

/* MENU SETUP //{ */

/* SetupMainMenu() //{ */

void MrsStatus::SetupMainMenu() {

  service_vec_.clear();

  for (unsigned long i = 0; i < service_input_vec_.size(); i++) {

    if (NullTracker_ && (i == 0 || i == 1)) {
      continue;  // disable land and land home if we are not flying
    }

    if (!NullTracker_ && i == 2) {

      continue;  // disable takeoff if flying
    }

    std::vector<std::string> results;
    boost::split(results, service_input_vec_[i], [](char c) { return c == ' '; });  // split the input string into words and put them in results vector

    for (int j = 2; j < results.size(); j++) {
      results[1] = results[1] + " " + results[j];
    }

    string service_name;

    if (results[0].at(0) == '/') {
      service_name = results[0];

    } else {
      service_name = "/" + _uav_name_ + "/" + results[0];
    }

    service tmp_service(service_name, results[1]);
    tmp_service.service_client = nh_.serviceClient<std_srvs::Trigger>(service_name);

    service_vec_.push_back(tmp_service);
  }

  main_menu_text_.clear();

  for (unsigned long i = 0; i < service_vec_.size(); i++) {
    main_menu_text_.push_back(service_vec_[i].service_display_name);
  }

  main_menu_text_.push_back("Set Constraints");
  main_menu_text_.push_back("Set Gains");
  main_menu_text_.push_back("Set Controller");

  Menu menu(1, 32, main_menu_text_);
  menu_vec_.push_back(menu);
}

//}

/* SetupGotoMenu() //{ */

void MrsStatus::SetupGotoMenu() {

  goto_menu_inputs_.clear();
  goto_menu_text_.clear();
  goto_menu_text_.push_back(" X:                ");
  goto_menu_text_.push_back(" Y:                ");
  goto_menu_text_.push_back(" Z:                ");
  goto_menu_text_.push_back(" Yaw:              ");
  goto_menu_text_.push_back(" " + uav_state_.header.frame_id + " ");

  Menu menu(1, 32, goto_menu_text_);
  menu_vec_.push_back(menu);


  for (int i = 0; i < 4; i++) {
    InputBox tmpbox(8, menu.getWin(), goto_double_vec_[i]);
    goto_menu_inputs_.push_back(tmpbox);
  }
}

//}

//}

/* PRINT FUNCTIONS //{ */

/* PrintMemLoad() //{ */

void MrsStatus::PrintMemLoad(WINDOW* win) {

  ifstream file("/proc/meminfo");
  string   line1, line2, line3, line4;
  getline(file, line1);
  getline(file, line2);
  getline(file, line3);
  getline(file, line4);
  file.close();

  vector<string> results;
  boost::split(results, line1, [](char c) { return c == ' '; });

  double ram_total;
  double ram_free;
  double ram_used;
  double buffers;

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_total = double(stol(results[i])) / 1000000;
      }
      catch (const invalid_argument& e) {
        ram_total = 0.0;
      }
      break;
    }
  }

  boost::split(results, line3, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        ram_free = double(stol(results[i])) / 1000000;
      }
      catch (const invalid_argument& e) {
        ram_free = 0.0;
      }
      break;
    }
  }

  boost::split(results, line4, [](char c) { return c == ' '; });

  for (unsigned long i = 1; i < results.size(); i++) {

    if (isdigit(results[i].front())) {
      try {
        buffers = double(stol(results[i])) / 1000000;
      }
      catch (const invalid_argument& e) {
        buffers = 0.0;
      }
      break;
    }
  }

  ram_used = ram_total - (ram_free + buffers);

  int    tmp_color = GREEN;
  double ram_ratio = ram_used / ram_total;
  if (ram_ratio > 0.8) {
    tmp_color = RED;
  } else if (ram_ratio > 0.6) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));

  PrintLimitedDouble(win, 2, 1, "RAM free: %4.1f G", (ram_free + buffers), 100);
}

//}

/* PrintCpuLoad() //{ */

void MrsStatus::PrintCpuLoad(WINDOW* win) {

  ifstream file("/proc/stat");
  string   line;
  getline(file, line);
  file.close();

  vector<string> results;
  boost::split(results, line, [](char c) { return c == ' '; });

  long idle;
  long non_idle;
  long total;

  try {
    idle     = stol(results[5]) + stol(results[6]);
    non_idle = stol(results[2]) + stol(results[3]) + stol(results[4]) + stol(results[7]) + stol(results[8]) + stol(results[9]);
    total    = idle + non_idle;
  }
  catch (const invalid_argument& e) {
    idle     = 0;
    non_idle = 0;
    total    = 0;
  }

  long total_diff = total - last_total_;
  long idle_diff  = idle - last_idle_;

  double cpu_load = 100 * (double(total_diff - idle_diff) / double(total_diff));

  last_total_ = total;
  last_idle_  = idle;

  int tmp_color = GREEN;
  if (cpu_load > 80.0) {
    tmp_color = RED;
  } else if (cpu_load > 60.0) {
    tmp_color = YELLOW;
  }

  wattron(win, COLOR_PAIR(tmp_color));

  PrintLimitedDouble(win, 1, 1, "CPU load: %4.1f %%", cpu_load, 99.9);
}

//}

/* PrintCpuFreq() //{ */

void MrsStatus::PrintCpuFreq(WINDOW* win) {

  ifstream file("/sys/devices/system/cpu/online");
  string   line;
  getline(file, line);
  file.close();

  vector<string> results;
  boost::split(results, line, [](char c) { return c == '-'; });

  int num_cores;

  try {
    num_cores = stoi(results[1]) + 1;
  }
  catch (const invalid_argument& e) {
    num_cores = 0;
  }

  long cpu_freq = 0;

  for (int i = 0; i < num_cores; i++) {
    string   filename = "/sys/devices/system/cpu/cpu" + to_string(i) + "/cpufreq/scaling_cur_freq";
    ifstream file(filename.c_str());
    getline(file, line);
    file.close();
    try {
      cpu_freq += stol(line);
    }
    catch (const invalid_argument& e) {
      cpu_freq = 0;
    }
  }

  double avg_cpu_ghz = double(cpu_freq / num_cores) / 1000000;


  wattron(win, COLOR_PAIR(GREEN));
  PrintLimitedDouble(win, 1, 21, "%4.2f GHz", avg_cpu_ghz, 10);
}

//}

/* PrintDiskSpace() //{ */

void MrsStatus::PrintDiskSpace(WINDOW* win) {

  boost::filesystem::space_info si = boost::filesystem::space(".");

  int gigas = round(si.available / 100000000);

  wattron(win, COLOR_PAIR(GREEN));
  if (gigas < 200 || gigas != last_gigas_) {
    wattron(win, COLOR_PAIR(YELLOW));
  }
  if (gigas < 100) {
    wattron(win, COLOR_PAIR(RED));
    PrintLimitedDouble(win, 2, 19, "HDD: %3.1f G", double(gigas) / 10, 10);
  } else {
    if (gigas < 1000) {
      PrintLimitedInt(win, 2, 19, "HDD:  %i G", gigas / 10, 1000);
    } else {
      PrintLimitedInt(win, 2, 19, "HDD: %i G", gigas / 10, 1000);
    }
  }
  last_gigas_ = gigas;
}

//}

/* PrintServiceResult() //{ */

void MrsStatus::PrintServiceResult(bool success, string msg) {

  wclear(bottom_window_);

  wattron(bottom_window_, A_BOLD);
  wattron(bottom_window_, COLOR_PAIR(GREEN));

  if (success) {

    PrintLimitedString(bottom_window_, 0, 0, "Service call success: " + msg, 120);

  } else {

    wattron(bottom_window_, COLOR_PAIR(RED));

    PrintLimitedString(bottom_window_, 0, 0, "Service call failed: " + msg, 120);

    wattroff(bottom_window_, COLOR_PAIR(RED));
  }

  bottom_window_clear_time_ = ros::Time::now();

  wattroff(bottom_window_, COLOR_PAIR(GREEN));
  wattroff(bottom_window_, A_BOLD);
}

//}

/* PrintLimitedInt() //{ */

void MrsStatus::PrintLimitedInt(WINDOW* win, int y, int x, string str_in, int num, int limit) {

  if (abs(num) > limit) {

    // if the number is larger than limit, replace it with scientific notation - 1e+01 to fit the screen
    for (unsigned long i = 0; i < str_in.length() - 2; i++) {
      if (str_in[i] == '.' && str_in[i + 2] == 'i') {
        str_in[i + 1] = '0';
        str_in[i + 2] = 'e';
        break;
      }
    }
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* PrintLimitedDouble() //{ */

void MrsStatus::PrintLimitedDouble(WINDOW* win, int y, int x, string str_in, double num, double limit) {

  if (fabs(num) > limit) {

    // if the number is larger than limit, replace it with scientific notation - 1e+01 to fit the screen
    for (unsigned long i = 0; i < str_in.length() - 2; i++) {
      if (str_in[i] == '.' && str_in[i + 2] == 'f') {
        str_in[i + 1] = '0';
        str_in[i + 2] = 'e';
        break;
      }
    }
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format, num);
}

//}

/* PrintLimitedString() //{ */

void MrsStatus::PrintLimitedString(WINDOW* win, int y, int x, string str_in, unsigned long limit) {

  if (str_in.length() > limit) {
    str_in.resize(limit);
  }

  const char* format = str_in.c_str();

  mvwprintw(win, y, x, format);
}

//}

/* PrintNoData() //{ */

void MrsStatus::PrintNoData(WINDOW* win, int y, int x) {

  wattron(win, A_BLINK);
  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, "!NO DATA!");
  wattroff(win, COLOR_PAIR(RED));
  wattroff(win, A_BLINK);
}

void MrsStatus::PrintNoData(WINDOW* win, int y, int x, string text) {

  wattron(win, COLOR_PAIR(RED));
  mvwprintw(win, y, x, text.c_str());
  PrintNoData(win, y, x + text.length());
}

//}

//}

/* CALLBACKS //{ */

/* UavStateCallback() //{ */

void MrsStatus::UavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {
  uav_state_topic_[0].counter++;
  uav_state_ = *msg;
}

//}

/* MavrosStateCallback() //{ */

void MrsStatus::MavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
  mavros_state_topic_[0].counter++;
  mavros_state_ = *msg;
}

//}

/* BatteryCallback() //{ */

void MrsStatus::BatteryCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
  mavros_state_topic_[1].counter++;
  battery_ = *msg;
}

//}

/* MavrosAttitudeCallback() //{ */

void MrsStatus::MavrosAttitudeCallback(const mavros_msgs::AttitudeTargetConstPtr& msg) {
  mavros_state_topic_[2].counter++;
  mavros_attitude_ = *msg;
}

//}

/* ControlManagerCallback() //{ */

void MrsStatus::ControlManagerCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[0].counter++;
  control_manager_                                                = *msg;
  control_manager_.active_tracker == "NullTracker" ? NullTracker_ = true : NullTracker_ = false;
}

//}

/* GainManagerCallback() //{ */

void MrsStatus::GainManagerCallback(const mrs_msgs::GainManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[1].counter++;
  gain_manager_ = *msg;
}

//}

/* ConstraintManagerCallback() //{ */

void MrsStatus::ConstraintManagerCallback(const mrs_msgs::ConstraintManagerDiagnosticsConstPtr& msg) {
  control_manager_topic_[2].counter++;
  constraint_manager_ = *msg;
}

//}

/* SetServiceCallback() //{ */

void MrsStatus::SetServiceCallback(const std_msgs::String& msg) {

  if (std::find(service_input_vec_.begin(), service_input_vec_.end(), msg.data) == service_input_vec_.end()) {
    service_input_vec_.push_back(msg.data);
  }
}
//}

/* StringCallback() //{ */

void MrsStatus::StringCallback(const ros::MessageEvent<std_msgs::String const>& event) {

  std::string pub_name = event.getPublisherName();
  std::string msg_str  = event.getMessage()->data;

  bool contains = false;

  for (unsigned long i = 0; i < string_info_vec_.size(); i++) {
    if (string_info_vec_[i].publisher_name == pub_name) {
      contains                           = true;
      string_info_vec_[i].display_string = msg_str;
      string_info_vec_[i].last_time      = ros::Time::now();
      break;
    }
  }

  if (!contains) {
    string_info tmp(pub_name, msg_str);
    string_info_vec_.push_back(tmp);
  }
}
//}

/* GenericCallback() //{ */

void MrsStatus::GenericCallback(const ShapeShifter::ConstPtr& msg, const string& topic_name, const int id) {
  generic_topic_vec_[id].counter++;
}

//}

//}

/* main() //{ */

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrs_status");

  initscr();
  start_color();
  cbreak();
  noecho();
  clear();
  nodelay(stdscr, true);
  keypad(stdscr, true);
  timeout(0);
  curs_set(0);  // disable cursor
  set_escdelay(0);
  use_default_colors();

  init_pair(NORMAL, COLOR_WHITE, BACKGROUND_DEFAULT);
  init_pair(FIELD, COLOR_WHITE, 235);
  init_pair(RED, COLOR_NICE_RED, BACKGROUND_DEFAULT);
  init_pair(YELLOW, COLOR_NICE_YELLOW, BACKGROUND_DEFAULT);
  init_pair(GREEN, COLOR_NICE_GREEN, BACKGROUND_DEFAULT);
  init_pair(BLUE, COLOR_NICE_BLUE, BACKGROUND_DEFAULT);

  attron(A_BOLD);

  /* DEBUG COLOR RAINBOW //{ */

  /* for (int j = 0; j < 256; j++) { */
  /*   init_pair(j, COLOR_WHITE, j); */
  /* } */

  /* int k = 0; */
  /* for (int j = 0; j < 256; j++) { */
  /*   attron(COLOR_PAIR(j)); */
  /*   mvwprintw(stdscr, k, (3 * j + 1) - k * 60 * 3, "%i", j); */
  /*   k = int(j / 60); */
  /*   attroff(COLOR_PAIR(j)); */
  /* } */
  /* refresh(); */
  //}

  MrsStatus status;

  while (ros::ok()) {

    ros::spin();
    return 0;
  }
}

//
