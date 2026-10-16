#pragma once

/* includes //{ */

#include <ncurses.h>

#include <mrs_uav_status/tui/print_helpers.hpp>
#include <mrs_uav_status/tui/constants.hpp>

//}

namespace mrs_uav_status::tui
{

/* printCpuLoad() //{ */

// Prints CPU load %, colored green/yellow/red as it rises past 60/80%.
inline void printCpuLoad(WINDOW *win, double cpu_load, bool mini) {
  int tmp_color = static_cast<int>(ColorPair::Green);
  if (cpu_load > 80.0) {
    tmp_color = static_cast<int>(ColorPair::Red);
  } else if (cpu_load > 60.0) {
    tmp_color = static_cast<int>(ColorPair::Yellow);
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini) {
    printLimitedString(win, 1, 1, "CPU", 3);
  } else {
    printLimitedDouble(win, 1, 1, "CPU: %4.1f %%", cpu_load, 99.9);
  }
}

//}

/* printCpuFreq() //{ */

// Prints the CPU frequency in GHz.
inline void printCpuFreq(WINDOW *win, double cpu_ghz) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));
  printLimitedDouble(win, 1, 16, "%4.2f GHz", cpu_ghz, 10);
}

//}

/* printMemLoad() //{ */

// Prints free RAM in GiB, colored green/yellow/red as used-ram ratio rises past 50/70%.
inline void printMemLoad(WINDOW *win, double free_ram, double total_ram, bool mini) {
  double used_ram  = total_ram - free_ram;
  double ram_ratio = used_ram / total_ram;

  int tmp_color = static_cast<int>(ColorPair::Green);
  if (ram_ratio > 0.7) {
    tmp_color = static_cast<int>(ColorPair::Red);
    wattron(win, A_BLINK);
  } else if (ram_ratio > 0.5) {
    tmp_color = static_cast<int>(ColorPair::Yellow);
  }

  wattron(win, COLOR_PAIR(tmp_color));
  if (mini) {
    printLimitedString(win, 2, 1, "RAM", 3);
  } else {
    printLimitedDouble(win, 2, 1, "RAM: %4.1f G", free_ram, 100);
  }
  wattroff(win, A_BLINK);
}

//}

/* printDiskSpace() //{ */

// Prints free disk space (G, or T above 1024G), yellow below 20G and red below 10G.
inline void printDiskSpace(WINDOW *win, int free_hdd, bool mini) {
  wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Green)));

  if (free_hdd < 20) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Yellow)));
  }

  // Mutually exclusive: separate unconditional ifs here used to all fire for a low free_hdd,
  // overwriting each other's text and leaving stray leftover characters on screen.
  if (free_hdd < 10) {
    wattron(win, COLOR_PAIR(static_cast<int>(ColorPair::Red)));
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedDouble(win, 2, 5, "%3.1f", double(free_hdd), 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f G", double(free_hdd), 10);
    }
  } else if (free_hdd < 1024) {
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i", free_hdd, 1000);
    } else {
      printLimitedInt(win, 2, 14, "HDD: %i G", free_hdd, 1000);
    }
  } else {
    if (mini) {
      printLimitedString(win, 1, 5, "HDD", 3);
      printLimitedInt(win, 2, 5, "%i T", free_hdd / 1024, 10);
    } else {
      printLimitedDouble(win, 2, 14, "HDD: %3.1f T", free_hdd / 1024.0, 1000);
    }
  }
}

//}

} // namespace mrs_uav_status::tui
