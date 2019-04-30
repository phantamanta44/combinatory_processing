import rospy
from .source import *


class PipelineSegment:
    def __init__(self, source):
        self.source = source

    def map(self, mapping_func):
        return PipelineSegment(DataSourceMapping(self, mapping_func))

    def join(self, others, joining_func, buffer_size=4):
        pipelines = [self]
        try:
            for other in others:
                pipelines.append(other)
        except TypeError:
            pipelines.append(others)
        return PipelineSegment(DataSourceConjoining(pipelines, joining_func, buffer_size))

    def sink(self, topic, msg_type):
        return Pipeline(self, topic, msg_type)


class Pipeline:
    def __init__(self, pipeline, sink_topic, sink_type):
        self.source = pipeline.source
        self.dest = rospy.Publisher(sink_topic, sink_type, queue_size=8)

    def consume(self, msg):
        if msg is not None:
            self.dest.publish(msg)

def source_topic(topic, msg_type, buffer_size=4):
    return PipelineSegment(DataSourceTopic(topic, msg_type, buffer_size))


def source_poll(poll_func):
    return PipelineSegment(DataSourcePollable(poll_func))
