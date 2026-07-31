#!/usr/bin/env python3

# Copyright (C) 2021 LEIDOS.
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
New simple format (preferred):
    ros=WARN                          # sets the default level
    yield_plugin=DEBUG                # exact logger name
    carma_wm=INFO                     # library logger
    carma_wm.route=DEBUG              # child logger (dot hierarchy)
    guidance.plugins.yield_plugin=DEBUG  # fully-qualified node logger

Legacy log4j format (still accepted for backward compatibility):
    log4j.logger.ros=WARN
    log4j.logger.ros.<logger_name>=<level>

param: config_file_path  Path to a .conf file in either format.
'''


def generate_log_levels_impl(config_file_path):

    levels = {'default_level': 'WARN'}  # Default log level will be WARN

    with open(config_file_path, 'r') as config_file:

        for line in config_file:

            no_ws_line = "".join(line.split())  # Remove whitespace

            if not no_ws_line or no_ws_line.startswith('#'):
                continue

            # ---- Legacy log4j format ----------------------------------------
            if no_ws_line.startswith('log4j'):
                parts = no_ws_line.split('=')
                if len(parts) != 2:
                    print("Failed to process line: " + str(no_ws_line))
                    continue

                full_logger_package = parts[0]
                log_level = parts[1]
                package_parts = full_logger_package.split('.')

                if len(package_parts) < 3:
                    print("Failed to process line: " + str(no_ws_line))
                    continue

                # log4j.logger.ros=LEVEL  → default
                if len(package_parts) == 3 and package_parts[2] == 'ros':
                    levels['default_level'] = log_level
                elif len(package_parts) >= 4:
                    # log4j.logger.ros.<name>=LEVEL  → join remaining parts with dots
                    levels['.'.join(package_parts[3:])] = log_level
                else:
                    print("Failed to process line: " + str(no_ws_line))
                continue

            # ---- New simple format: key=LEVEL --------------------------------
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
