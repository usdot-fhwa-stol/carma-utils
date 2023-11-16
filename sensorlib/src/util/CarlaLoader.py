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
    LOAD_CARLA_EGG = os.environ.get("LOAD_CARLA_EGG")
    CARLA_VERSION = os.environ.get("CARLA_VERSION")
    HOME = os.environ.get("HOME")
    CARLA_EGG_DIR = os.environ.get("CARLA_EGG_DIR")
    DEFAULT_CARLA_EGG_DIR = os.path.join(HOME, "carla")

    @staticmethod
    def load_carla_lib(carla_version=None):
        if CarlaLoader.LOAD_CARLA_EGG is None:
            print("\nLOAD_CARLA_EGG not set, skipping CarlaLoader.")
            return
        elif carla_version is None and CarlaLoader.CARLA_VERSION is None:
            print("\nCARLA VERSION NOT SET. EXITING.")
            sys.exit()
        elif carla_version is None:
            carla_version = CarlaLoader.CARLA_VERSION
        carla_egg_file = CarlaLoader.__find_carla_egg(carla_version)
        sys.path.append(carla_egg_file)

    @staticmethod
    def __find_file(pattern, path):
        result = []
        for root, dirs, files in os.walk(path):
            for name in files:
                if fnmatch.fnmatch(name, pattern):
                    result.append(os.path.join(root, name))
        return result

    @staticmethod
    def __find_carla_egg(carla_version):

        if CarlaLoader.CARLA_EGG_DIR is None:
            print(f"\nCARLA_EGG_DIR not set, using default {CarlaLoader.DEFAULT_CARLA_EGG_DIR}")
            carla_egg_dir = CarlaLoader.DEFAULT_CARLA_EGG_DIR
        else:
            carla_egg_dir = CarlaLoader.CARLA_EGG_DIR

        # Search for the CARLA python .egg file
        try:
            carla_egg_name = "carla-" + carla_version + "*" + str(sys.version_info.major) + "*-" + str(
                "win-amd64" if os.name == "nt" else "linux-x86_64") + ".egg"
            print("Looking for CARLA egg: " + carla_egg_name)
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
