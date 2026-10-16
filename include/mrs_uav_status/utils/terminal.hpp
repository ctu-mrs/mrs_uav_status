#pragma once

/* includes //{ */

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

//}

namespace mrs_uav_status::utils
{

/* callTerminal() //{ */

// Runs cmd in a shell and returns its captured stdout.
inline std::string callTerminal(const char *cmd) {
  std::array<char, 128>                  buffer;
  std::string                            result;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(cmd, "r"), static_cast<int (*)(FILE *)>(pclose));

  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}

//}

} // namespace mrs_uav_status::utils
