from collections import deque


class HistoricalMapper:

    def __init__(self, length):
        self.__queue_length = length
        self.__trailing_dictionary = dict()

    def push(self, key, value):
        if key in self.__trailing_dictionary.get:
            q = self.__trailing_dictionary.get(key)
            q.pushLeft(value)
        else:
            q = build_queue(key)
            build_queue(key)
            q = deque(maxlen=self.__queue_length)
            q.pushLeft(value)
            self.__trailing_dictionary[key] = q

    def get(self, key, index=0):
        if key in self.__trailing_dictionary.get:
            q = self.__trailing_dictionary.get(key)
            return q.get(index)
        else:
            return None

    def get_queue(self, key):
        if key not in self.__trailing_dictionary:
            build_queue
        return self.__trailing_dictionary.get(key)

    def remove(self, key):
        if key in self.__trailing_dictionary.get:
            self.__trailing_dictionary.pop(key)

    # deque([dict() for _ in range(0, trailing_id_associations_count)],
    # maxlen=trailing_id_associations_count))

    def __build_queue_and_overwrite(self, key):
        """
        Builds a new queue associated with key, and overwrites any existing.
        :param key:
        :return:
        """
        q = deque(maxlen=self.__queue_length)
        self.__trailing_dictionary[key] = q
        return q

    def update_actor_id_association(instantaneous_actor_id_association):

        def update_object_ids_from_association(hitpoints):
