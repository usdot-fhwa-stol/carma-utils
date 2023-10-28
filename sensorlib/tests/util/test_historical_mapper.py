# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from collections import deque

from util.HistoricalMapper import HistoricalMapper


class TestHistoricalMapper(unittest.TestCase):

    # ------------------------------------------------------------------------------
    # Unit tests
    # ------------------------------------------------------------------------------

    def test_push(self):

        mapper = HistoricalMapper(3)

        # Build 0 queue
        assert mapper._HistoricalMapper__trailing_dictionary == {}
        mapper.push(0, 0)
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == [0]

        mapper.push(0, 0)
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == [0, 0]

        # Queues should not interfere with each other
        mapper.push(1, 1)
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == [0, 0]
        assert list(mapper._HistoricalMapper__trailing_dictionary[1]) == [1]

        # Queue length should be limited
        mapper.push(10, 0)
        mapper.push(10, 1)
        mapper.push(10, 2)
        assert list(mapper._HistoricalMapper__trailing_dictionary[10]) == [2, 1, 0]
        mapper.push(10, 3)
        mapper.push(10, 4)
        mapper.push(10, 5)
        assert list(mapper._HistoricalMapper__trailing_dictionary[10]) == [5, 4, 3]

        # Queue order should be LIFO
        mapper.push(2, 0)
        mapper.push(2, 1)
        mapper.push(2, 2)
        assert list(mapper._HistoricalMapper__trailing_dictionary[2]) == [2, 1, 0]

    def test_get(self):

        mapper = HistoricalMapper(3)

        mapper._HistoricalMapper__trailing_dictionary = {
            0: deque([2, 1, 0], maxlen=3),
            1: deque([2, 1], maxlen=3),
            2: deque([3], maxlen=3)
        }

        # Assert get accesses queues by key and index correctly
        assert mapper.get(0, 0) == 2
        assert mapper.get(0, 1) == 1
        assert mapper.get(0, 2) == 0

        assert mapper.get(1, 0) == 2
        assert mapper.get(1, 1) == 1

        assert mapper.get(2, 0) == 3

        # Default index should be 0
        assert mapper.get(0) == 2
        assert mapper.get(1) == 2
        assert mapper.get(2) == 3

        # Error conditions
        assert mapper.get(100) is None
        assert mapper.get(100, 0) is None
        assert mapper.get(100, 10) is None
        assert mapper.get(1, 1000) is None
        assert mapper.get(1, -1) is None

    def test_pop(self):

        mapper = HistoricalMapper(3)

        mapper._HistoricalMapper__trailing_dictionary = {
            0: deque([2, 1, 0], maxlen=3),
            1: deque([2, 1], maxlen=3),
            2: deque([3], maxlen=3)
        }

        # Nominal
        v = mapper.pop(0)
        assert v == 2
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == [1, 0]

        v = mapper.pop(0)
        assert v == 1
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == [0]

        v = mapper.pop(0)
        assert v == 0
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == []

        v = mapper.pop(0)
        assert v == None
        assert list(mapper._HistoricalMapper__trailing_dictionary[0]) == []

        # Error conditions
        v = mapper.pop(1000)
        assert v == None

    def test_get_keys(self):

        mapper = HistoricalMapper(3)
        mapper.push(0, 0)
        mapper.push(2, 3)
        mapper.push(3, 3)
        assert mapper.get_keys() == [0, 2, 3]

    def test_get_queue(self):

        mapper = HistoricalMapper(3)

        mapper._HistoricalMapper__trailing_dictionary = {
            0: deque([2, 1, 0], maxlen=3),
            1: deque([2, 1], maxlen=3),
            2: deque([3], maxlen=3)
        }

        assert list(mapper.get_queue(0)) == [2, 1, 0]
        assert list(mapper.get_queue(100)) == []

    def test_remove_queue(self):

        mapper = HistoricalMapper(3)

        mapper._HistoricalMapper__trailing_dictionary = {
            0: deque([2, 1, 0], maxlen=3),
            1: deque([2, 1], maxlen=3),
            2: deque([3], maxlen=3)
        }

        # Nominal operation
        assert list(mapper.get_queue(0)) == [2, 1, 0]
        mapper.remove_queue(0)
        assert list(mapper.get_queue(0)) == []

        # Error conditions - don't throw exception
        mapper.remove_queue(100)

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

        assert len(mapper.get_queue(0)) == 3
        assert len(mapper.get_queue(1)) == 3
        assert len(mapper.get_queue(2)) == 3

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

        mapper.push(0, "0-0")
        mapper.push(1, "1-0")
        mapper.push(2, "2-0")

        mapper.push(0, "0-1")
        mapper.push(1, "1-1")

        mapper.push(0, "0-2")

        q0 = mapper.get_queue(0)
        q1 = mapper.get_queue(1)
        q2 = mapper.get_queue(2)

        assert list(q0) == ["0-2", "0-1", "0-0"]
        assert list(q1) == ["1-1", "1-0"]
        assert list(q2) == ["2-0"]

    def test_stairstep_up(self):

        mapper = HistoricalMapper(3)

        mapper.push(0, "0-0")

        mapper.push(0, "0-1")
        mapper.push(1, "1-1")

        mapper.push(0, "0-2")
        mapper.push(1, "1-2")
        mapper.push(2, "2-2")

        q0 = mapper.get_queue(0)
        q1 = mapper.get_queue(1)
        q2 = mapper.get_queue(2)

        assert list(q0) == ["0-2", "0-1", "0-0"]
        assert list(q1) == ["1-2", "1-1"]
        assert list(q2) == ["2-2"]
