from collections import deque


class HistoricalMapper:

    def __init__(self, length):
        self.__queue_length = length
        self.__trailing_dictionary = dict()

    # ------------------------------------------------------------------------------
    # Queue operations
    # ------------------------------------------------------------------------------

    def push(self, key, value):
        q = self.__get_queue(key)
        q.appendleft(value)

    def get(self, key, index=0):
        q = self.get_queue(key)
        if 0 <= index < len(q):
            return q[index]
        else:
            return None

    def pop(self, key):
        q = self.__get_queue(key)
        if len(q) > 0:
            return q.popleft()
        else:
            return None

    # ------------------------------------------------------------------------------
    # Queue management operations
    # ------------------------------------------------------------------------------

    def get_keys(self):
        return list(self.__trailing_dictionary.keys())

    def get_queue(self, key):
        return self.__get_queue(key)

    def remove_queue(self, key):
        if key in self.__trailing_dictionary:
            del self.__trailing_dictionary[key]

    # ------------------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------------------

    def __get_queue(self, key):
        if key in self.__trailing_dictionary:
            return self.__trailing_dictionary.get(key)
        else:
            return self.__build_new_queue(key)

    def __build_new_queue(self, key):
        """
        Builds a new queue associated with key, and overwrites any existing.
        :param key:
        :return:
        """
        q = deque(maxlen=self.__queue_length)
        self.__trailing_dictionary[key] = q
        return q
