// Copyright (C) 2026 LEIDOS.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARMA_ROS2_UTILS__LOGGING_UTILS_HPP_
#define CARMA_ROS2_UTILS__LOGGING_UTILS_HPP_

#include <algorithm>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>

#include <boost/algorithm/string.hpp>
#include <rcutils/logging.h>

namespace carma_ros2_utils
{

// Convert a ROS2 fully qualified node name (e.g. /guidance/plugins/yield_plugin)
// to the dot-separated logger name used by rcutils (guidance.plugins.yield_plugin).
inline std::string fqn_to_logger_name(const std::string & fqn)
{
  std::string name = fqn;
  if (!name.empty() && name[0] == '/') {
    name = name.substr(1);
  }
  std::replace(name.begin(), name.end(), '/', '.');
  return name;
}

// Convert a log-level string (case-insensitive) to the matching rcutils severity.
inline RCUTILS_LOG_SEVERITY log_level_to_severity(const std::string & level_str)
{
  std::string level = level_str;
  boost::algorithm::to_lower(level);
  if (level == "debug") return RCUTILS_LOG_SEVERITY_DEBUG;
  if (level == "info")  return RCUTILS_LOG_SEVERITY_INFO;
  if (level == "error") return RCUTILS_LOG_SEVERITY_ERROR;
  if (level == "fatal") return RCUTILS_LOG_SEVERITY_FATAL;
  return RCUTILS_LOG_SEVERITY_WARN;
}

// Parse the flat JSON dict produced by generate_log_levels.py into a string map.
// Format: {"key": "VALUE", "key2": "VALUE2"}
inline std::map<std::string, std::string> parse_log_levels_json(const std::string & json)
{
  std::map<std::string, std::string> result;

  std::string content = json;
  content.erase(std::remove(content.begin(), content.end(), '{'), content.end());
  content.erase(std::remove(content.begin(), content.end(), '}'), content.end());

  auto strip = [](std::string & s) {
    s.erase(
      std::remove_if(
        s.begin(), s.end(),
        [](char c) {
          return c == '"' || c == ' ' || c == '\t' || c == '\n' || c == '\r';
        }),
      s.end());
  };

  std::stringstream ss(content);
  std::string pair;
  while (std::getline(ss, pair, ',')) {
    const auto colon_pos = pair.find(':');
    if (colon_pos == std::string::npos) continue;

    std::string key   = pair.substr(0, colon_pos);
    std::string value = pair.substr(colon_pos + 1);
    strip(key);
    strip(value);

    if (!key.empty() && !value.empty()) {
      result[key] = value;
    }
  }

  return result;
}

// Determine the configured log level for a node given its fully-qualified logger name
// (e.g. guidance.plugins.yield_plugin) and leaf name (e.g. yield_plugin).
//
// Lookup priority:
//   1. Exact FQN match  (guidance.plugins.yield_plugin)
//   2. Leaf name match  (yield_plugin) — convenient: one conf entry covers all namespaces
//   3. default_level entry
//   4. WARN as hard fallback
inline RCUTILS_LOG_SEVERITY resolve_node_log_level(
  const std::map<std::string, std::string> & levels,
  const std::string & fqn_logger,
  const std::string & leaf_name)
{
  auto it = levels.find(fqn_logger);
  if (it != levels.end()) return log_level_to_severity(it->second);

  it = levels.find(leaf_name);
  if (it != levels.end()) return log_level_to_severity(it->second);

  it = levels.find("default_level");
  if (it != levels.end()) return log_level_to_severity(it->second);

  return RCUTILS_LOG_SEVERITY_WARN;
}

// Read CARMA_ROS_LOGGING_CONFIG (produced by generate_log_levels.py) and call
// rcutils_logging_set_logger_level for every explicitly-named logger.
//
// This covers library loggers (e.g. carma_wm, basic_autonomy) that are not
// tied to any ROS node and cannot receive --log-level through the launch system.
// Call this once in the component-manager constructor, before any components load.
inline void apply_logger_levels_from_env()
{
  const char * env_val = std::getenv("CARMA_ROS_LOGGING_CONFIG");
  if (!env_val) return;

  const auto levels = parse_log_levels_json(std::string(env_val));
  for (const auto & kv : levels) {
    if (kv.first == "default_level") continue;
    rcutils_logging_set_logger_level(kv.first.c_str(), log_level_to_severity(kv.second));
  }
}

}  // namespace carma_ros2_utils

#endif  // CARMA_ROS2_UTILS__LOGGING_UTILS_HPP_
