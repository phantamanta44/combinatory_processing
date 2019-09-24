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
    def __init__(self):
        self.last_poll_time = -1
        self.last_data = None

    def pull_data(self, poll_time):
        raise RuntimeError('impossible state!')

    def poll(self, poll_time):
        if poll_time > self.last_poll_time:
            self.last_poll_time = poll_time
            self.last_data = self.pull_data(poll_time)
        return self.last_data


class DataSourceTopic(DataSource):
    def __init__(self, topic_name, msg_type, buffer_size):
        super(DataSourceTopic, self).__init__()
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.buffer = CircularQueue(buffer_size)

    def enqueue(self, msg):
        self.buffer.offer(msg)

    def pull_data(self, poll_time):
        return self.buffer.query()


class DataSourcePollable(DataSource):
    def __init__(self, poll_func):
        super(DataSourcePollable, self).__init__()
        self.poll_func = poll_func

    def pull_data(self, poll_time):
        return self.poll_func()


class DataSourceMapping(DataSource):
    def __init__(self, upstream, mapping_func):
        super(DataSourceMapping, self).__init__()
        self.source = upstream.source
        self.mapping_func = mapping_func

    def pull_data(self, poll_time):
        unmapped = self.source.poll(poll_time)
        return None if unmapped is None else self.mapping_func(unmapped)


class DataSourceConjoining(DataSource):
    def __init__(self, upstream, joining_func, buffer_size):
        super(DataSourceConjoining, self).__init__()
        self.sources = [pipeline.source for pipeline in upstream]
        self.joining_func = joining_func
        self.count = len(upstream)
        self.data = [CircularQueue(buffer_size) for pipeline in upstream]

    def pull_data(self, poll_time):
        has_all_data = True
        for i in range(self.count):
            datum = self.sources[i].poll(poll_time)
            if datum is not None:
                self.data[i].offer(datum)
            elif not self.data[i].has_data():
                has_all_data = False
        return self.joining_func(*tuple(queue.query() for queue in self.data)) if has_all_data else None
