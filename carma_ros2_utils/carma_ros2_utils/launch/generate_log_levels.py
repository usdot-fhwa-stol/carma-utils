#!/usr/bin/env python3

# Copyright (C) 2021-2026 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''
Generates a json dictionary of logger names with log levels.
The default level will be at key default_level.
Possible levels [ DEBUG, INFO, WARN, ERROR, FATAL ]

Supported config file formats
------------------------------
ros=WARN                          # sets the default level
yield_plugin=DEBUG                # exact logger name (rclcpp::get_logger("yield_plugin")) OR
                                  # the node name without the full namespace for convenience
                                  # (without `guidance.plugins.yield_plugin` prefix)
carma_wm=INFO                     # library loggers (applies to all nodes using the library)
yield_plugin.carma_wm=DEBUG       # child loggers:
                                  # - scopes to only yield_plugin's carma_wm logger
                                  # - setting the parent (yield_plugin=DEBUG) will also affect
                                  # the child loggers
guidance.plugins.yield_plugin=DEBUG  # full namespace node logger

param: config_file_path  Path to a .conf file in either format.
'''

def generate_log_levels_impl(config_file_path):

    levels = {'default_level': 'WARN'}  # Default log level will be WARN

    with open(config_file_path, 'r') as config_file:

        for line in config_file:

            no_ws_line = "".join(line.split())  # Remove whitespace

            if not no_ws_line or no_ws_line.startswith('#'):
                continue

            # ---- Format: logger_key=LEVEL --------------------------------
            parts = no_ws_line.split('=')
            if len(parts) != 2:
                print("Failed to process line: " + str(no_ws_line))
                continue

            key, log_level = parts[0], parts[1]

            if not key or not log_level:
                print("Failed to process line: " + str(no_ws_line))
                continue

            if key == 'ros':
                levels['default_level'] = log_level
            else:
                levels[key] = log_level

    return levels


def generate_log_levels(config_file_path):
    # Convert dictionary to JSON string (replace single quotes with double quotes)
    return str(generate_log_levels_impl(config_file_path)).replace("'", '"')
