from abc import abstractmethod


class AbstractNoiseModel:
    @abstractmethod
    def apply_position_noise(self, object_list):
        pass

    @abstractmethod
    def apply_orientation_noise(self, object_list):
        pass

    @abstractmethod
    def apply_type_noise(self, object_list):
        pass

    @abstractmethod
    def apply_list_inclusion_noise(self, object_list):
        pass
