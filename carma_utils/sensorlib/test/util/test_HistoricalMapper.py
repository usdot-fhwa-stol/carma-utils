import unittest

from src.util.HistoricalMapper import HistoricalMapper


class TestHistoricalMapper(unittest.TestCase):

    def setUp(self):
        self.data_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 999]


    # ------------------------------------------------------------------------------
    # Unit tests
    # ------------------------------------------------------------------------------

    def test_push(self):
        mapper = HistoricalMapper(3)

        # Build 0 queue
        assert len(mapper.get_queue(0)) == 0
        mapper.push(0, 0)
        assert len(mapper.get_queue(0)) == 1
        assert list(mapper.get_queue(0)) == [0]
        mapper.push(0, 0)
        assert len(mapper.get_queue(0)) == 2
        assert list(mapper.get_queue(0)) == [0, 0]

        # Queues should not interfere with each other
        assert len(mapper.get_queue(1)) == 0
        mapper.push(0, 0)
        assert len(mapper.get_queue(0)) == 1
        assert list(mapper.get_queue(0)) == [0]


    def test_get(self):
        pass
    def test_get_queue(self):
        pass
    def test_remove(self):
        pass


    # ------------------------------------------------------------------------------
    # Integration tests
    # ------------------------------------------------------------------------------

    def test_push_singular_map_repeatedly(self):
        mapper = HistoricalMapper(3)
        data = {0: 0, 1: 1, 2: 2}
        for i in range(0, 3):
            for k, v in data.items():
                mapper.push(k, v)

        assert len(mapper._HistoricalMapper__trailing_dictionary) == 3

        assert len(mapper.get_queue(0)) == 0
        assert len(mapper.get_queue(1)) == 0
        assert len(mapper.get_queue(2)) == 0

        assert mapper.get(0, 0) == 0
        assert mapper.get(1, 0) == 1
        assert mapper.get(2, 0) == 2

        assert mapper.get(0, 1) == 0
        assert mapper.get(1, 1) == 1
        assert mapper.get(2, 1) == 2

        assert mapper.get(0, 2) == 0
        assert mapper.get(1, 2) == 1
        assert mapper.get(2, 2) == 2

    def test_stairstep_down(self):

        mapper = HistoricalMapper(3)

        mapper.push(0, 0)
        mapper.push(1, 1)
        mapper.push(2, 2)

        mapper.push(0, 0)
        mapper.push(1, 1)

        mapper.push(0, 0)

        q0 = mapper.get_queue(0)
        q1 = mapper.get_queue(1)
        q2 = mapper.get_queue(2)

        assert list(q0) == [0, 0, 0]
        assert list(q1) == [1, 1]
        assert list(q2) == [2]

    def test_stairstep_up(self):
        mapper = HistoricalMapper(3)
        mapper.push(0, 0)

        mapper.push(0, 0)
        mapper.push(1, 1)

        mapper.push(0, 0)
        mapper.push(1, 1)
        mapper.push(2, 2)

        q0 = mapper.get_queue(0)
        q1 = mapper.get_queue(1)
        q2 = mapper.get_queue(2)

        assert list(q0) == [0, 0, 0]
        assert list(q1) == [1, 1]
        assert list(q2) == [2]
























        def test_case_nominal_usage_list(self):

            # Set up the switch
            data_pipe = switch()

            # Assign a value through a function
            case(data_pipe, 1, lambda x: "One")

            # Assign values directly
            case(data_pipe, 2, "Two")
            case(data_pipe, 3, "Three")
            case(data_pipe, 4, "Four")
            case(data_pipe, 5, "Five")
            case(data_pipe, 6, "Six")
            case(data_pipe, 7, "Seven")
            case(data_pipe, 8, "Eight")
            case(data_pipe, 9, "Nine")
            case(data_pipe, 10, "Ten")

            # Assign a default value
            case(data_pipe, default=lambda x: "Unknown")

            # Run the switch
            out = list(map(data_pipe, self.data_list))
            self.assertListEqual(out, self.expected_data_list)

        def test_case_nominal_usage_dict(self):

            # Set up the switch
            data_pipe = switch()

            # Assign a value through a function
            case(data_pipe, 1, lambda x: "One")

            # Assign values directly
            case(data_pipe, 2, "Two")
            case(data_pipe, 3, "Three")
            case(data_pipe, 4, "Four")
            case(data_pipe, 5, "Five")
            case(data_pipe, 6, "Six")
            case(data_pipe, 7, "Seven")
            case(data_pipe, 8, "Eight")
            case(data_pipe, 9, "Nine")
            case(data_pipe, 10, "Ten")

            # Assign a default value
            case(data_pipe, default=lambda x: "Unknown")

            # Run the switch
            out = list(map(data_pipe, self.data_dict))
            self.assertListEqual(out, self.expected_data_list)

        def test_case_nominal_usage_dict_with_default(self):

            # Set up the switch
            data_pipe = switch()

            # Assign a value through a function
            case(data_pipe, 1, lambda x: "One")

            # Assign values directly
            case(data_pipe, 2, "Two")
            case(data_pipe, 3, "Three")
            case(data_pipe, 4, "Four")
            case(data_pipe, 5, "Five")




