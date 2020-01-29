import rospy
from .source import *


class PipelineSegment:
    def __init__(self, source, source_hierarchy=[]):
        self.source = source
        self.source_hierarchy = source_hierarchy + [source]

    def persist(self):
        return PipelineSegment(DataSourcePersisting(self), self.source_hierarchy)

    def default(self, default_getter):
        return PipelineSegment(DataSourceDefaulting(self, default_getter), self.source_hierarchy)

    def default_const(self, default_value):
        return self.default(lambda: default_value)

    def map(self, mapping_func):
        return PipelineSegment(DataSourceMapping(self, mapping_func), self.source_hierarchy)

    def join(self, others, joining_func, buffer_size=4):
        pipelines = [self]
        try:
            for other in others:
                pipelines.append(other)
        except TypeError:
            pipelines.append(others)
        return PipelineSegment(DataSourceConjoining(pipelines, joining_func, buffer_size), self.source_hierarchy)

    def sink(self, topic, msg_type):
        return Pipeline(self, topic, msg_type)


class Pipeline:
    def __init__(self, pipeline, sink_topic, sink_type):
        self.source = pipeline.source
        self.source_set = set(pipeline.source_hierarchy)
        self.dest = rospy.Publisher(sink_topic, sink_type, queue_size=8)

    def consume(self, msg):
        if msg is not None:
            self.dest.publish(msg)

def source_topic(topic, msg_type, buffer_size=4):
    return PipelineSegment(DataSourceTopic(topic, msg_type, buffer_size))


def source_poll(poll_func):
    return PipelineSegment(DataSourcePollable(poll_func))
