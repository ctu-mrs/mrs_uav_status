/* includes //{ */

#include <mrs_uav_status/tui/pane_box.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <mrs_uav_status/tui/constants.hpp>
#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/utils/helpers.hpp>

//}

namespace mrs_uav_status::tui
{

/* PaneBox() //{ */

PaneBox::PaneBox() {
  // Default pane: available sensors (dynamic, plugin-driven) — name/rate/status
  panes_.push_back({"Sensors", [this](WINDOW *win) { renderSensorsPane(win); }, nullptr});
  // ROS per-node CPU usage (was its own top-right box)
  panes_.push_back({"ROS Node CPU", [this](WINDOW *win) { renderNodeCpuPane(win); }, nullptr});
  // GNSS fix + custom display strings
  panes_.push_back({"GNSS & Strings", [this](WINDOW *win) { renderGnssStringsPane(win); }, nullptr});
  // Problems + errors. Auto-focused when either becomes non-empty
  panes_.push_back({"Problems & Errors", [this](WINDOW *win) { renderProblemsPane(win); },
                    [this]() {
                      // Avoid auto-focusing on frozen stale problems/errors.
                      return snapshot_->freshness.general_robot_info &&
                             (!snapshot_->general_robot_info.missing_for_automatic_takeoff.empty() || !snapshot_->general_robot_info.errors.empty());
                    }});

  pane_focus_prev_.assign(panes_.size(), false);
  pane_idx_ = 0;
}

//}

/* cycle() //{ */

void PaneBox::cycle() {
  if (panes_.empty()) {
    return;
  }
  pane_idx_ = (pane_idx_ + 1) % panes_.size();
}

//}

/* select() //{ */

void PaneBox::select(std::size_t idx) {
  if (idx < panes_.size()) {
    pane_idx_ = idx;
  }
}

//}

/* render() //{ */

void PaneBox::render(WINDOW *win, const status::RenderSnapshot &snapshot, bool light_scheme, bool mini) {
  if (!win || panes_.empty()) {
    return;
  }

  snapshot_     = &snapshot;
  light_scheme_ = light_scheme;
  mini_         = mini;

  // auto-focus: when a pane wants_focus switching to it
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool wants = panes_[i].wants_focus && panes_[i].wants_focus();
    if (wants && !pane_focus_prev_[i]) {
      pane_idx_ = i;
    }
    pane_focus_prev_[i] = wants;
  }

  panes_[pane_idx_].render(win);

  snapshot_ = nullptr;
}

//}

/* drawPaneChrome() //{ */

int PaneBox::drawPaneChrome(WINDOW *win) {
  const status::BorderStatus bs = snapshot_->border_status;

  werase(win);
  wattron(win, A_BOLD);
  wattroff(win, A_STANDOUT);
  printBox(win, bs.avoiding_collision, bs.bumper_active, bs.can_takeoff, bs.null_tracker);

  // Tab bar in the top border (row 0): numbered tabs. The active tab shows its
  // number + name in brackets (green); the others show just their number in red
  // to signal they're switchable. Drawn before standout, like the top-bar hotkeys.
  int x = 2;
  for (std::size_t i = 0; i < panes_.size(); ++i) {
    const bool        active = (i == pane_idx_);
    const std::string num    = std::to_string(i + 1);

    if (active) {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
      const std::string tab = num + " [" + panes_[i].title + "]";
      mvwaddstr(win, 0, x, tab.c_str());
      x += static_cast<int>(tab.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
    } else {
      wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
      mvwaddstr(win, 0, x, num.c_str());
      x += static_cast<int>(num.size());
      wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    }
    x += 1; // single-space gap between tabs
  }

  if (light_scheme_) {
    wattron(win, A_STANDOUT);
  }

  return 1;
}

//}

/* renderGnssStringsPane() //{ */

void PaneBox::renderGnssStringsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  constexpr int MAX_STRING_ROWS = 9;
  constexpr int TEXT_WIDTH      = 80;

  uint8_t gnss_fix_type      = 0;
  uint8_t gnss_num_sats      = 0;
  double  gnss_pos_acc       = 100.0;
  double  gnss_status_rate   = 0.0;
  bool    gnss_status_msg_ok = false;

  const bool have_system_health_info = snapshot_->freshness.system_health_info;
  if (const auto *gnss = utils::findSensor(snapshot_->system_health_info.available_sensors, status::SENSOR_TYPE_GNSS); gnss) {
    const std::string fix_type_raw = utils::lookupDetail(gnss->details, "fix_type");
    gnss_status_msg_ok             = (fix_type_raw != "nan");
    gnss_fix_type                  = static_cast<uint8_t>(utils::parseLongOr(fix_type_raw, 0));
    gnss_num_sats                  = static_cast<uint8_t>(utils::parseLongOr(utils::lookupDetail(gnss->details, "num_satellites"), 0));
    gnss_pos_acc                   = utils::parseDoubleOr(utils::lookupDetail(gnss->details, "position_accuracy"), 100.0);
    gnss_status_rate               = gnss->rate;
  }

  // gnss_status_rate/gnss_status_msg_ok are cached and never reset, so gate on freshness too.
  const bool gnss_available = have_system_health_info && gnss_status_rate > 0.0 && gnss_status_msg_ok;

  std::string fix_label;
  bool        bad_fix = true;
  if (gnss_available) {
    bad_fix = true;
    switch (gnss_fix_type) {
    case 0:
      fix_label = "NO GNSS";
      break;
    case 1:
      fix_label = "NO FIX";
      break;
    case 2:
      fix_label = "2D FIX";
      bad_fix   = false;
      break;
    case 3:
      fix_label = "3D FIX";
      bad_fix   = false;
      break;
    case 4:
      fix_label = "3D SBAS FIX";
      bad_fix   = false;
      break;
    case 5:
      fix_label = "RTK FLOAT";
      bad_fix   = false;
      break;
    case 6:
      fix_label = "RTK FIX (INT)";
      bad_fix   = false;
      break;
    case 7:
      fix_label = "STATIC - BASESTATION";
      bad_fix   = false;
      break;
    case 8:
      fix_label = "PPP 3D FIX";
      bad_fix   = false;
      break;
    default:
      fix_label = "UNKNOWN";
      break;
    }
  } else {
    fix_label = "NO DATA";
  }

  const auto gnss_color = bad_fix ? ColorPair::Red : ColorPair::Green;
  wattron(win, COLOR_PAIR(static_cast<int>(gnss_color)));
  printLimitedString(win, row++, 1, "GNSS: " + fix_label, TEXT_WIDTH);
  wattroff(win, COLOR_PAIR(static_cast<int>(gnss_color)));

  if (gnss_available) {
    std::string gnss_acc_string;
    if (gnss_pos_acc >= 100.0) {
      gnss_acc_string = "N/A";
    } else {
      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << gnss_pos_acc;
      gnss_acc_string = stream.str();
    }
    printLimitedString(win, row++, 1, "Num sats: " + std::to_string(gnss_num_sats) + " Acc: " + gnss_acc_string + " m", TEXT_WIDTH);
  }

  // Eviction already happened in UavStatusCore::pruneStrings() (top of every tick());
  // snapshot_->display_strings is whatever's currently live for display.
  const auto &display_strings = snapshot_->display_strings;
  if (display_strings.empty()) {
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  ++row; // blank separator
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row++, 1, "Strings: " + std::to_string(display_strings.size()), TEXT_WIDTH);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

  struct ParsedLine
  {
    std::string text;
    int         color;
    bool        blink;
  };
  std::vector<ParsedLine> parsed;
  parsed.reserve(display_strings.size());
  for (const auto &raw : display_strings) {
    int         tmp_color = static_cast<int>(ColorPair::Normal);
    bool        blink     = false;
    std::string display   = raw;

    // In-band colour tag: leading "-R"/"-r"/"-Y"/"-y"/"-G"/"-g" (uppercase blinks).
    if (display.size() >= 3 && display[0] == '-') {
      switch (display[1]) {
      case 'R':
        blink = true;
        [[fallthrough]];
      case 'r':
        tmp_color = static_cast<int>(ColorPair::Red);
        break;
      case 'Y':
        blink = true;
        [[fallthrough]];
      case 'y':
        tmp_color = static_cast<int>(ColorPair::Yellow);
        break;
      case 'G':
        blink = true;
        [[fallthrough]];
      case 'g':
        tmp_color = static_cast<int>(ColorPair::Green);
        break;
      }
      if (tmp_color != static_cast<int>(ColorPair::Normal)) {
        display.erase(0, 3);
      }
    }

    parsed.push_back({display, tmp_color, blink});
  }

  row = renderCappedList(
      win, row, MAX_STRING_ROWS, parsed.size(), [&](std::size_t i) { return static_cast<int>(wrapText("- " + parsed[i].text, TEXT_WIDTH).size()); },
      [&](int r, std::size_t i, int max_r) {
        const auto &p = parsed[i];
        if (p.blink) {
          wattron(win, A_BLINK);
        }
        wattron(win, COLOR_PAIR(p.color));
        r = printWrappedString(win, r, 1, "- " + p.text, TEXT_WIDTH, max_r);
        wattroff(win, COLOR_PAIR(p.color));
        wattroff(win, A_BLINK);
        return r;
      },
      40);

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderProblemsPane() //{ */

void PaneBox::renderProblemsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  const std::vector<std::string> &problems                = snapshot_->general_robot_info.missing_for_automatic_takeoff;
  const std::vector<std::string> &raw_errors              = snapshot_->general_robot_info.errors;
  const bool                      have_general_robot_info = snapshot_->freshness.general_robot_info;

  // Empty vectors are indistinguishable from "genuinely zero problems" without this check.
  if (!have_general_robot_info) {
    printNoData(win, row, 1, false);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  constexpr int MAX_PROBLEM_ROWS = 9;
  constexpr int TEXT_WIDTH       = 80;
  constexpr int MAX_ENTRY_LINES  = 3; // caps a single verbose entry so it can't crowd out the rest of the list

  auto cappedEntryLines = [&](const std::string &entry) { return capWrappedLines("- " + entry, TEXT_WIDTH, MAX_ENTRY_LINES); };
  auto printCappedEntry = [&](int r, const std::string &entry, int max_r) {
    for (const auto &line : cappedEntryLines(entry)) {
      if (r > max_r) {
        break;
      }
      mvwprintw(win, r++, 1, "%s", line.c_str());
    }
    return r;
  };

  // What is missing for automatic takeoff no longer applies once a real tracker (not NullTracker) is active.
  const bool is_flying = snapshot_->freshness.control_info && !snapshot_->border_status.null_tracker;

  // Errors render first: they matter even mid-flight. Reserve 2 rows (blank separator + header)
  // so a long errors list can't push the "Missing for automatic takeoff" header off the pane. Keyed
  // off !is_flying alone, matching the header's own render condition below.
  const int errors_max_row = !is_flying ? MAX_PROBLEM_ROWS - 2 : MAX_PROBLEM_ROWS;

  const utils::SplitErrors        split              = utils::splitNotResponding(raw_errors);
  const std::vector<std::string> &errors             = split.specific;
  const std::size_t               num_not_responding = split.not_responding.size();
  const bool                      has_not_responding = num_not_responding > 0;

  std::string errors_header = "Errors: " + std::to_string(errors.size());
  if (has_not_responding) {
    errors_header += " (+" + std::to_string(num_not_responding) + " not responding)";
  }

  const auto errors_color = raw_errors.empty() ? ColorPair::Green : ColorPair::Red;
  wattron(win, COLOR_PAIR(static_cast<int>(errors_color)));
  printLimitedString(win, row++, 1, errors_header, TEXT_WIDTH);
  wattroff(win, COLOR_PAIR(static_cast<int>(errors_color)));

  // Wrapped and capped like any other entry; its rows are reserved below the specific errors.
  std::string not_responding_entry;
  if (has_not_responding) {
    not_responding_entry = "Not responding (" + std::to_string(num_not_responding) + "): ";
    for (std::size_t i = 0; i < num_not_responding; ++i) {
      not_responding_entry += (i > 0 ? ", " : "") + split.not_responding[i];
    }
  }
  const int not_responding_rows = has_not_responding ? static_cast<int>(cappedEntryLines(not_responding_entry).size()) : 0;

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  row = renderCappedList(
      win, row, errors_max_row - not_responding_rows, errors.size(), [&](std::size_t i) { return static_cast<int>(cappedEntryLines(errors[i]).size()); },
      [&](int r, std::size_t i, int max_r) { return printCappedEntry(r, errors[i], max_r); }, TEXT_WIDTH);

  if (has_not_responding && row <= errors_max_row) {
    row = printCappedEntry(row, not_responding_entry, errors_max_row);
  }
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));

  if (!is_flying) {
    if (row <= MAX_PROBLEM_ROWS) {
      ++row; // blank separator
    }

    if (row <= MAX_PROBLEM_ROWS) {
      const auto problems_color = problems.empty() ? ColorPair::Green : ColorPair::Red;
      wattron(win, COLOR_PAIR(static_cast<int>(problems_color)));
      printLimitedString(win, row++, 1, "Missing for automatic takeoff: " + std::to_string(problems.size()), TEXT_WIDTH);
      wattroff(win, COLOR_PAIR(static_cast<int>(problems_color)));
    }

    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    row = renderCappedList(
        win, row, MAX_PROBLEM_ROWS, problems.size(), [&](std::size_t i) { return static_cast<int>(cappedEntryLines(problems[i]).size()); },
        [&](int r, std::size_t i, int max_r) { return printCappedEntry(r, problems[i], max_r); }, TEXT_WIDTH);
    wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderSensorsPane() //{ */

void PaneBox::renderSensorsPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<status::SensorStatusData> sensors                 = snapshot_->system_health_info.available_sensors;
  const bool                            have_system_health_info = snapshot_->freshness.system_health_info;

  if (!have_system_health_info) {
    // Otherwise the last real sensor list/rates/statuses render forever, looking healthy.
    printNoData(win, row, 1, mini_);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  // Column header.
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "sensor", 36);
  printLimitedString(win, row, 42, "rate", 8);
  printLimitedString(win, row, 55, "status", 12);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  // Worst-first: ERROR, WARN, STALE, then OK -- so the sensors most worth seeing
  // survive the MAX_SENSOR_ROWS cutoff below when the list is longer than it.
  auto severity_rank = [](uint8_t level) -> int {
    switch (level) {
    case status::SENSOR_STATUS_ERROR:
      return 0;
    case status::SENSOR_STATUS_WARN:
      return 1;
    case status::SENSOR_STATUS_STALE:
      return 2;
    default: // OK
      return 3;
    }
  };
  std::stable_sort(sensors.begin(), sensors.end(), [&severity_rank](const auto &a, const auto &b) { return severity_rank(a.level) < severity_rank(b.level); });

  constexpr int MAX_SENSOR_ROWS   = 9;
  constexpr int MESSAGE_WIDTH     = 80;
  constexpr int MAX_MESSAGE_LINES = 3; // caps a single verbose message so it can't crowd out every other sensor

  auto cappedMessageLines = [&](const std::string &message) { return capWrappedLines("    -> " + message, MESSAGE_WIDTH, MAX_MESSAGE_LINES); };

  row = renderCappedList(
      win, row, MAX_SENSOR_ROWS, sensors.size(),
      [&](std::size_t i) {
        int rows = 1;
        if (sensors[i].level != status::SENSOR_STATUS_OK) {
          rows += static_cast<int>(cappedMessageLines(sensors[i].message).size());
        }
        return rows;
      },
      [&](int r, std::size_t i, int max_r) {
        const auto &s = sensors[i];

        printLimitedString(win, r, 1, s.name, 36);
        printLimitedDouble(win, r, 40, "%6.1f", s.rate, 100000);

        // SensorStatus.level: 0=OK, 1=WARN, 2=ERROR, 3=STALE.
        int         color = static_cast<int>(ColorPair::Normal);
        std::string label = "?";
        switch (s.level) {
        case 0:
          color = static_cast<int>(ColorPair::Green);
          label = "OK";
          break;
        case 1:
          color = static_cast<int>(ColorPair::Yellow);
          label = "WARN";
          break;
        case 2:
          color = static_cast<int>(ColorPair::Red);
          label = "ERROR";
          break;
        case 3:
          color = static_cast<int>(ColorPair::Yellow);
          label = "STALE";
          break;
        default:
          break;
        }
        wattron(win, COLOR_PAIR(color));
        printLimitedString(win, r, 55, label, 12);
        ++r;

        if (s.level != status::SENSOR_STATUS_OK) {
          for (const auto &line : cappedMessageLines(s.message)) {
            if (r > max_r) {
              break;
            }
            mvwprintw(win, r++, 1, "%s", line.c_str());
          }
        }
        wattroff(win, COLOR_PAIR(color));
        return r;
      },
      40);

  if (sensors.empty()) {
    printLimitedString(win, row, 1, "no sensors reported", 40);
  }

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

/* renderNodeCpuPane() //{ */

void PaneBox::renderNodeCpuPane(WINDOW *win) {
  int row = drawPaneChrome(win);

  std::vector<status::CpuLoadData> node_cpu_loads          = snapshot_->system_health_info.onboard_computer_info.node_cpu_loads;
  const bool                       have_system_health_info = snapshot_->freshness.system_health_info;

  if (!have_system_health_info) {
    // Otherwise an empty node_cpu_loads sums to 0.0, rendering as a healthy-looking 0% CPU.
    printNoData(win, row, 1, mini_);
    wattroff(win, A_BOLD);
    wnoutrefresh(win);
    return;
  }

  // Aggregate (single-core %) total — OnboardComputerInfo doesn't expose it.
  double cpu_load_total = 0.0;
  for (const auto &n : node_cpu_loads) {
    cpu_load_total += n.cpu_load;
  }

  constexpr int MAX_NODE_ROWS = 9;

  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 42, "Total CPU", 20);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedDouble(win, row, 55, "%5.1f", cpu_load_total, 9999);
  printLimitedString(win, row, 61, "%", 5);
  ++row;

  mvwhline(win, row, 55, ACS_HLINE, 7);
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedString(win, row, 1, "node", 36);
  wattroff(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  ++row;

  // Sort by CPU load descending, then name ascending for tie-breaking.
  std::sort(node_cpu_loads.begin(), node_cpu_loads.end(),
            [](const auto &a, const auto &b) { return (a.cpu_load > b.cpu_load) || ((a.cpu_load == b.cpu_load) && (a.node_name < b.node_name)); });

  row = renderCappedList(
      win, row, MAX_NODE_ROWS, node_cpu_loads.size(), [&](std::size_t) { return 1; },
      [&](int r, std::size_t i, int) {
        const auto &n = node_cpu_loads[i];
        printLimitedString(win, r, 1, n.node_name, 36);

        short tmp_color = static_cast<int>(ColorPair::Green);
        if (n.cpu_load > 99.9) {
          tmp_color = static_cast<int>(ColorPair::Red);
        } else if (n.cpu_load > 49.9) {
          tmp_color = static_cast<int>(ColorPair::Yellow);
        }
        wattron(win, COLOR_PAIR(tmp_color));
        printLimitedDouble(win, r, 55, "%5.1f", n.cpu_load, 9999);
        wattroff(win, COLOR_PAIR(tmp_color));
        return r + 1;
      },
      40);

  wattroff(win, A_BOLD);
  wnoutrefresh(win);
}

//}

} // namespace mrs_uav_status::tui
