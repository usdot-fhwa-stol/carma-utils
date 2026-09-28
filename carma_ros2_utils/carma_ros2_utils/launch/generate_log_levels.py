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

import json

'''
Generates a json dictionary of logger names with log levels.
The default level will be at key default_level.
The output is typically used in the ros2 launch file (e.g. carma_docker_launch.py)
to set the log levels for the nodes in the launch file.
Possible levels [ DEBUG, INFO, WARN, ERROR, FATAL ]
Expects a .conf file in either format:
    logger_key=LEVEL
    logger_key1=LEVEL
    logger_key2=LEVEL
    ...
@param: config_file_path  Path to a .conf file in either format
@return: A json dictionary of logger names with log levels.
         The default level will be at key default_level.
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
    return json.dumps(generate_log_levels_impl(config_file_path))
