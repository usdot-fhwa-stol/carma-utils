class NoiseModelFactory:

    @staticmethod
    def get_noise_model(noise_model_name, noise_model_config):
        if noise_model_name == "GaussianNoiseModel":
            from src.noise_models.GaussianNoiseModel import GaussianNoiseModel
            return GaussianNoiseModel(noise_model_config)
        else:
            raise NotImplementedError("Noise model type {} not implemented.".format(noise_model_name))
