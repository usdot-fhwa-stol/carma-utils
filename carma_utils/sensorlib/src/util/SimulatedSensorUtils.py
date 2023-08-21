# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import yaml


class SimulatedSensorUtils:

    # ------------------------------------------------------------------------------
    # Configuration file reading
    # ------------------------------------------------------------------------------

    @staticmethod
    def load_config_from_file(config_filepath):
        with open(config_filepath, "r") as file:
            config = yaml.safe_load(file)
            return config
