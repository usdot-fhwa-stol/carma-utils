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
