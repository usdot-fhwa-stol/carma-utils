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

#include <carma_ros2_utils/logging_utils.hpp>

#include <sstream>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace carma_ros2_utils
{

std::map<std::string, std::string> parse_log_levels_json(const std::string & json_str)
{
  std::map<std::string, std::string> result;

  boost::property_tree::ptree tree;
  std::istringstream iss(json_str);
  try {
    boost::property_tree::json_parser::read_json(iss, tree);
  } catch (const boost::property_tree::json_parser::json_parser_error &) {
    return result;
  }

  for (const auto & kv : tree) {
    result[kv.first] = kv.second.get_value<std::string>();
  }

  return result;
}

}  // namespace carma_ros2_utils
