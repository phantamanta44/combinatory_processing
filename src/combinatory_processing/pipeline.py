import rospy
from .composable import *
from .source import *


class PipelineSegment:
    def __init__(self, source, consumer=Composable()):
        self.source = source
        self.consumer = consumer

    def map(self, mapping_func):
        return PipelineSegment(self.source, UnaryMappingComposable(mapping_func, self.consumer))

    def join(self, others, joining_func, buffer_size=4):
        pipelines = [self]
        try:
            for other in others:
                pipelines.append(other)
        except TypeError:
            pipelines.append(others)
        return PipelineSegment(DataSourceConjoining(pipelines, buffer_size), MultiMappingComposable(joining_func))

    def sink(self, topic, msg_type):
        return Pipeline(self, topic, msg_type)


class Pipeline:
    def __init__(self, pipeline, sink_topic, sink_type):
        self.source = pipeline.source
        self.consumer = pipeline.consumer
        self.dest = rospy.Publisher(sink_topic, sink_type, queue_size=8)

    def consume(self, msg):
        result = self.consumer.process(msg)
        if result is not None:
            self.dest.publish(result)

def source_topic(topic, msg_type, buffer_size=4):
    return PipelineSegment(DataSourceTopic(topic, msg_type, buffer_size))


def source_poll(poll_func):
    return PipelineSegment(DataSourcePollable(poll_func))
