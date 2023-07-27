class AbstractNoiseModel:
    def apply_position_noise(self, object_list):
        pass

    def apply_orientation_noise(self, object_list):
        pass

    def apply_type_noise(self, object_list):
        pass

    def apply_list_inclusion_noise(self, object_list, excluded_object_list):
        pass
