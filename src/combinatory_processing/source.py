class CircularQueue:
    def __init__(self, capacity):
        self.buffer = [None] * capacity
        self.end = capacity - 1
        self.head_pointer = 0
        self.tail_pointer = 0

    def query(self):
        if self.head_pointer == self.tail_pointer:
            return None
        result = self.buffer[self.head_pointer]
        self._advance_head()
        return result
    
    def offer(self, item):
        self.buffer[self.tail_pointer] = item
        if self.tail_pointer == self.end:
            self.tail_pointer = 0
        else:
            self.tail_pointer += 1
        if self.tail_pointer == self.head_pointer:
            self._advance_head()

    def has_data(self):
        return self.head_pointer != self.tail_pointer
                    
    def _advance_head(self):
        if self.head_pointer == self.end:
            self.head_pointer = 0
        else:
            self.head_pointer += 1


class DataSource(object):
    def poll(self):
        raise RuntimeError('impossible state!')


class DataSourceTopic(DataSource):
    def __init__(self, topic_name, msg_type, buffer_size):
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.buffer = CircularQueue(buffer_size)

    def enqueue(self, msg):
        self.buffer.offer(msg)

    def poll(self):
        return self.buffer.query()


class DataSourcePollable(DataSource):
    def __init__(self, poll_func):
        self.poll_func = poll_func

    def poll(self):
        return self.poll_func()


class DataSourceConjoining(DataSourcePollable):
    def __init__(self, sources, buffer_size):
        super(DataSourceConjoining, self).__init__(self.retrieve_data)
        self.sources = [pipeline.source for pipeline in sources]
        self.count = len(sources)
        self.data = [CircularQueue(buffer_size) for pipeline in sources]

    def retrieve_data(self):
        has_all_data = True
        for i in range(self.count):
            datum = self.sources[i].poll()
            if datum is not None:
                self.data[i].offer(datum)
            elif not self.data[i].has_data():
                has_all_data = False
        return tuple(queue.query() for queue in self.data) if has_all_data else None