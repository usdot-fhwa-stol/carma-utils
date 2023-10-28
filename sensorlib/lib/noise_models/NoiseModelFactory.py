# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from noise_models.GaussianNoiseModel import GaussianNoiseModel


class NoiseModelFactory:
    """Factory to generate noise models of various types."""

    @staticmethod
    def get_noise_model(noise_model_name, noise_model_config):
        """
        Build a noise model of the requested type.
        :param noise_model_name: Name of the noise model.
        :param noise_model_config: Dictionary containing the noise model configuration.
        :return: The noise model instance.
        """
        if noise_model_name == "GaussianNoiseModel":
            return GaussianNoiseModel(noise_model_config)
        else:
            raise NotImplementedError("Noise model type {} not implemented.".format(noise_model_name))
