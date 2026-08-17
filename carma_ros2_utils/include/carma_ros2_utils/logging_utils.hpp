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

// Helpers for resolving and applying per-logger log levels from the
// CARMA_ROS_LOGGING_CONFIG env var, which carries the flat JSON object produced by
// generate_log_levels.py (parsed from a carma_rosconsole.conf file).
// Used to set node logger levels at startup and to support library loggers
// (e.g. carma_wm, basic_autonomy) that have no ROS node of their own.
namespace carma_ros2_utils
{

// Convert a ROS2 fully qualified node name (e.g. /guidance/plugins/yield_plugin)
// to the dot-separated logger name used by rcutils (guidance.plugins.yield_plugin).
// \param fqn Fully qualified node name, optionally leading with '/'.
// \return Dot-separated logger name.
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
// \param level_str Level name (DEBUG/INFO/WARN/ERROR/FATAL), any case.
// \return Matching RCUTILS_LOG_SEVERITY, or WARN if unrecognized.
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

// Parse the flat JSON object produced by generate_log_levels.py (e.g.
// {"default_level": "WARN", "yield_plugin": "DEBUG"}) into a string map.

// NOTE: Implementation is explicitly defined in logging_utils.cpp file
// This is because the boost::property_tree JSON parser transitively
// drags in boost::bind, which collides with std::placeholders in some translation
// units, so keeping it in the .cpp file to prevent the leak into this header
// \param json_str Flat JSON object string of logger_name -> level.
// \return Map of logger_name -> level string. Empty if json_str is malformed.
std::map<std::string, std::string> parse_log_levels_json(const std::string & json_str);

// Determine the configured log level for a node given its fully-qualified logger name
// (e.g. guidance.plugins.yield_plugin) and leaf name (e.g. yield_plugin).
//
// Lookup priority:
//   1. Fully-qualified dot name  (e.g. guidance.plugins.yield_plugin)
//   2. Leaf node name            (e.g. yield_plugin) — one conf entry covers all namespaces
//                                Because it is convenient to use only the node name instead
//                                of the fully-qualified name. Typically, node name is
//                                unique enough to avoid collisions.
//   3. default_level
//   4. WARN as hard fallback
// \param levels Parsed logger_name -> level map (see parse_log_levels_json).
// \param fqn_logger Fully qualified dot-separated logger name.
// \param leaf_name Node's leaf (unqualified) name.
// \return Resolved RCUTILS_LOG_SEVERITY.
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

// Apply conf entries scoped to this node's leaf name (e.g. "yield_plugin.carma_wm")
// to the node's actual runtime child logger name (e.g.
// "guidance.plugins.yield_plugin.carma_wm").
//
// rcutils only walks left-anchored prefixes of a logger's fully-qualified name when
// resolving its effective level, so a key like "yield_plugin.carma_wm" is never a
// prefix of the node's real child logger name and can never match on its own -- it has
// to be rewritten in terms of this node's fqn_logger before being set.
// \param levels Parsed logger_name -> level map (see parse_log_levels_json).
// \param fqn_logger Fully qualified dot-separated logger name of this node.
// \param leaf_name Node's leaf (unqualified) name.
inline void apply_scoped_child_logger_levels(
  const std::map<std::string, std::string> & levels,
  const std::string & fqn_logger,
  const std::string & leaf_name)
{
  const std::string prefix = leaf_name + ".";
  for (const auto & kv : levels) {
    if (kv.first.compare(0, prefix.size(), prefix) == 0) {
      const std::string child_logger = fqn_logger + "." + kv.first.substr(prefix.size());
      rcutils_logging_set_logger_level(child_logger.c_str(), log_level_to_severity(kv.second));
    }
  }
}

// Apply "library name" conf entries (bare keys with no dot, e.g. "carma_wm",
// "basic_autonomy") to this node's own child logger for that library (e.g.
// "guidance.plugins.yield_plugin.carma_wm"). Library code commonly logs through a
// node-scoped child logger (node_logger.get_child("carma_wm")) rather than the bare
// global one, so without this a plain "carma_wm=DEBUG" would only ever reach the bare
// "carma_wm" logger and not each plugin's own instance -- this makes it cascade to
// every loaded node, matching what the conf file documents ("applies to all plugins
// that link to that library").
// \param levels Parsed logger_name -> level map (see parse_log_levels_json).
// \param fqn_logger Fully qualified dot-separated logger name of this node.
// \param leaf_name Node's leaf (unqualified) name.
inline void apply_library_logger_levels_for_node(
  const std::map<std::string, std::string> & levels,
  const std::string & fqn_logger,
  const std::string & leaf_name)
{
  for (const auto & kv : levels) {
    const std::string & key = kv.first;
    if (key == "default_level") continue;
    if (key == fqn_logger || key == leaf_name) continue;
    if (key.find('.') != std::string::npos) continue;  // scoped keys handled elsewhere

    const std::string child_logger = fqn_logger + "." + key;
    rcutils_logging_set_logger_level(child_logger.c_str(), log_level_to_severity(kv.second));
  }
}

// Read CARMA_ROS_LOGGING_CONFIG (produced by generate_log_levels.py) and call
// rcutils_logging_set_logger_level for every explicitly-named logger.
//
// This covers library loggers (e.g. carma_wm, basic_autonomy) that are not
// tied to any ROS node and cannot receive --log-level through the launch system.
// Call this once in the component-manager constructor, before any components load.
// Reads CARMA_ROS_LOGGING_CONFIG from the environment; no-op if unset.
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
