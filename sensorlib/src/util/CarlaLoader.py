# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import fnmatch
import os
import sys


class CarlaLoader:
    # Constants
    HOME = os.environ.get("HOME")
    DEFAULT_CARLA_EGG_DIR = os.path.join(HOME, "carla")

    @staticmethod
    def load_carla_lib():

        # Get environment variables
        LOAD_CARLA_EGG = os.environ.get("LOAD_CARLA_EGG")
        CARLA_VERSION = os.environ.get("CARLA_VERSION")
        CARLA_EGG_DIR = os.environ.get("CARLA_EGG_DIR")

        # Load the .egg by default (if unset), or explicitly true
        if LOAD_CARLA_EGG is None or LOAD_CARLA_EGG == "" or bool(LOAD_CARLA_EGG):
            load_carla_egg = True
        else:
            load_carla_egg = False

        # Get carla version
        if CARLA_VERSION is None or CARLA_VERSION == "":
            carla_version = ""
        else:
            carla_version = CARLA_VERSION

        # Get carla egg dir
        if CARLA_EGG_DIR is None or CARLA_EGG_DIR == "":
            carla_egg_dir = CarlaLoader.DEFAULT_CARLA_EGG_DIR
        else:
            carla_egg_dir = CARLA_EGG_DIR

        # Abort .egg loading
        if not load_carla_egg:
            print("\nLOAD_CARLA_EGG set to False, skipping .egg loading.")
            return

        # Log the requested CARLA_VERSION
        print(f"\nSearching for CARLA_VERSION \"{carla_version}\"")

        # Log the requested CARLA_EGG_DIR
        if CARLA_EGG_DIR is None:
            print(
                f"\nCARLA_EGG_DIR not set. Searching directory DEFAULT_CARLA_EGG_DIR \"{CarlaLoader.DEFAULT_CARLA_EGG_DIR}\"")
        else:
            print(f"\nSearching directory CARLA_EGG_DIR \"{carla_egg_dir}\"")

        # Search for the .egg file and load it
        carla_egg_file = CarlaLoader.__find_carla_egg(carla_version, carla_egg_dir)
        print(f"\nLoading CARLA egg file: \"{carla_egg_file}\"")
        sys.path.append(carla_egg_file)

    @staticmethod
    def __find_carla_egg(carla_version, carla_egg_dir):

        # Search for the CARLA python .egg file
        try:
            carla_egg_name = "carla-" + carla_version + "*" + str(sys.version_info.major) + "*-" + str(
                "win-amd64" if os.name == "nt" else "linux-x86_64") + ".egg"
            print("CARLA egg search string: " + carla_egg_name)
            carla_egg_locations = CarlaLoader.__find_file(carla_egg_name, carla_egg_dir)
            print("Found carla egg(s): " + str(carla_egg_locations))

            if len(carla_egg_locations) == 0:
                print("\nUNABLE TO FIND CARLA EGG")
                sys.exit()
            else:
                return carla_egg_locations[0]

        except IndexError:
            print("\nUNABLE TO FIND CARLA EGG")
            pass

    @staticmethod
    def __find_file(pattern, path):
        result = []
        for root, dirs, files in os.walk(path):
            for name in files:
                if fnmatch.fnmatch(name, pattern):
                    result.append(os.path.join(root, name))
        return result
