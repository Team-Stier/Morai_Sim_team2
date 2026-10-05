#pragma once

#include <regex>
#include <string>

namespace visualization_pkg {
inline std::string plannerStatusText(const std::string& reason, bool stop_required) {
  std::smatch mode, zone;
  if (!std::regex_search(reason, mode, std::regex("(?:^|;\\s*)planner_mode=([a-z_]+)(?:;|$)")))
    return "Planner: WAITING";
  const std::string value = mode[1];
  const std::string name = value == "hybrid_astar" ? "Hybrid A*" :
                           value == "frenet" ? "Frenet (RDDF)" : "UNKNOWN";
  std::string text = "Planner: " + name;
  if (std::regex_search(reason, zone, std::regex("(?:^|;\\s*)zone=([A-Za-z0-9_]+)(?:;|$)")))
    text += " | " + zone[1].str();
  if (stop_required) text += " | STOP";
  return text;
}
}  // namespace visualization_pkg
