# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import os
import sys

import fnmatch


class CarlaLoader:
    # Constants
    HomeDir = os.environ.get('HOME')
    DefaultCarlaEggDir = os.path.join(HomeDir, "carla")

    @staticmethod
    def load_carla_lib(carla_version="0.9.14"):
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
    def __find_carla_egg(carla_version="0.9.14"):

        carla_egg_dir = os.getenv("CARLA_EGG_DIR")

        if not carla_egg_dir:
            print(f"\nCARLA_EGG_DIR not set, using default {CarlaLoader.DefaultCarlaEggDir}")
            carla_egg_dir = CarlaLoader.DefaultCarlaEggDir

        # this looks for the carla python API .egg file
        try:

            carla_egg_name = 'carla-' + carla_version + '*' + str(sys.version_info.major) + '*-' + str(
                'win-amd64' if os.name == 'nt' else 'linux-x86_64') + '.egg'
            print("Looking for CARLA egg: " + carla_egg_name)
            carla_egg_locations = CarlaLoader.__find_file(carla_egg_name, carla_egg_dir)
            print("Found carla egg(s): " + str(carla_egg_locations))

            if len(carla_egg_locations) == 0:
                print("\nUNABLE TO FIND CARLA EGG")
                sys.exit()
            elif len(carla_egg_locations) == 1:
                carla_egg_to_use = carla_egg_locations[0]
            else:
                print("\nFound multiple carla egg files: ")
                for i, egg_found in enumerate(carla_egg_locations):
                    print("[" + str(i + 1) + "]    " + egg_found)

                egg_selected = input("\nSelect a carla egg file to use: ")

                try:
                    egg_selected = int(egg_selected)
                except:
                    print("\nInvalid selection, please try again")
                    sys.exit()

                if egg_selected <= len(carla_egg_locations):
                    carla_egg_to_use = carla_egg_locations[egg_selected - 1]
                else:
                    print("\nInvalid selection, please try again")
                    sys.exit()

            return carla_egg_to_use

        except IndexError:
            print("\nUNABLE TO FIND CARLA EGG")

            pass
