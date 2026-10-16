#pragma once

/* includes //{ */

#include <ncurses.h>
#include <string>

#include <mrs_uav_status/tui/constants.hpp>

//}

namespace mrs_uav_status::tui
{

/* setupColors() //{ */

// Returns whether the light colorscheme is active.
inline bool setupColors(const std::string &colorscheme, bool colorblind_mode) {
  bool light = false;

  init_pair(static_cast<int>(ColorPair::Normal), COLOR_WHITE, static_cast<int>(BackgroundColor::Default));
  init_pair(static_cast<int>(ColorPair::Field), COLOR_WHITE, 235);
  init_pair(static_cast<int>(ColorPair::Red), static_cast<int>(Color::NiceRed), static_cast<int>(BackgroundColor::Default));
  init_pair(static_cast<int>(ColorPair::Yellow), static_cast<int>(Color::NiceYellow), static_cast<int>(BackgroundColor::Default));

  if (colorblind_mode) {
    init_pair(static_cast<int>(ColorPair::Green), static_cast<int>(Color::NiceBlue), static_cast<int>(BackgroundColor::Default));
  } else {
    init_pair(static_cast<int>(ColorPair::Green), static_cast<int>(Color::NiceGreen), static_cast<int>(BackgroundColor::Default));
  }

  if (colorscheme.find("COLORSCHEME_LIGHT") != std::string::npos) {
    init_pair(static_cast<int>(ColorPair::Normal), COLOR_BLACK, static_cast<int>(BackgroundColor::Default));
    init_pair(static_cast<int>(ColorPair::Field), COLOR_WHITE, 237);
    init_pair(static_cast<int>(ColorPair::Yellow), static_cast<int>(Color::DarkYellow), static_cast<int>(BackgroundColor::Default));
    if (colorblind_mode) {
      init_pair(static_cast<int>(ColorPair::Green), static_cast<int>(Color::DarkBlue), static_cast<int>(BackgroundColor::Default));
    } else {
      init_pair(static_cast<int>(ColorPair::Green), static_cast<int>(Color::DarkGreen), static_cast<int>(BackgroundColor::Default));
    }
    light = true;
  }

  return light;
}

//}

} // namespace mrs_uav_status::tui
