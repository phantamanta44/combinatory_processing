import rospy
from .source import DataSourceTopic


class Subscription:
    def __init__(self, msg_type):
        self.msg_type = msg_type
        self.pipelines = []

    def accept(self, msg):
        for pipeline in self.pipelines:
            pipeline.source.enqueue(msg)


def spin(name, pipelines, anon=False, freq=30):
    rospy.init_node(name, anonymous=anon)

    subs = {}
    for pipeline in pipelines:
        source = pipeline.source
        if isinstance(source, DataSourceTopic):
            if source.topic_name not in subs:
                sub = Subscription(source.msg_type)
                rospy.Subscriber(source.topic_name, source.msg_type, callback=sub.accept)
                subs[source.topic_name] = sub
            elif subs[source.topic_name].msg_type != source.msg_type:
                raise RuntimeError('Topic type mismatch for %s: %s != %s' % (
                    source.topic_name, subs[source.topic_name].msg_type, source.msg_type))
            subs[source.topic_name].pipelines.append(pipeline)

    poll_time = 0
    sleeper = rospy.Rate(freq)
    while not rospy.is_shutdown():
        for pipeline in pipelines:
            pipeline.consume(pipeline.source.poll(poll_time))
        poll_time += 1
        sleeper.sleep()
