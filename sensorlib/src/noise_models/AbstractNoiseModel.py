# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from abc import abstractmethod


class AbstractNoiseModel:
    """Base class for noise models, providing interface to apply noise by adjusting detected object data."""

    @abstractmethod
    def apply_position_noise(self, object_list):
        """
        Apply noise to the object positions.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_orientation_noise(self, object_list):
        """
        Apply noise to the object orientations.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_type_noise(self, object_list):
        """
        Adjust the type assigned to each object, for example changing a vehicle to a pedestrian. Designed to simulate
        sensor misidentification. param object_list: List of DetectedObject objects to apply noise to. return: List
        of DetectedObject objects with noise applied.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_list_inclusion_noise(self, object_list):
        """
        Adjust which objects are included in the final list based on applied noise. Designed to simulate sensor dropout.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_position_covariance_noise(self, object_list):
        """
        Apply noise related to position covariance to the object positions.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_orientation_covariance_noise(self, object_list):
        """
        Apply noise related to orientation covariance to the object orientations.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_linear_velocity_covariance_noise(self, object_list):
        """
        Apply noise related to velocity covariance to the object velocities.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass

    @abstractmethod
    def apply_angular_velocity_covariance_noise(self, object_list):
        """
        Apply noise related to angular velocity covariance to the object angular velocities.

        :param object_list: List of DetectedObject objects to apply noise to.
        :return: List of DetectedObject objects with noise applied.
        """
        pass


