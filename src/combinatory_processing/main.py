import rospy
from .source import DataSourceTopic


class Subscription:
    def __init__(self, msg_type):
        self.msg_type = msg_type
        self.sources = []

    def accept(self, msg):
        for source in self.sources:
            source.enqueue(msg)


def spin(name, pipelines, anon=False, freq=30):
    rospy.init_node(name, anonymous=anon)

    subs = {}
    seen_sources = set()
    for pipeline in pipelines:
        for source in pipeline.source_set:
            if isinstance(source, DataSourceTopic) and source not in seen_sources:
                print(str(type(source)) + ' is a DataSourceTopic: ' + str(isinstance(source, DataSourceTopic)))
                seen_sources.add(source)
                if source.topic_name not in subs:
                    sub = Subscription(source.msg_type)
                    rospy.Subscriber(source.topic_name, source.msg_type, callback=sub.accept)
                    subs[source.topic_name] = sub
                elif subs[source.topic_name].msg_type != source.msg_type:
                    raise RuntimeError('Topic type mismatch for %s: %s != %s' % (
                        source.topic_name, subs[source.topic_name].msg_type, source.msg_type))
                subs[source.topic_name].sources.append(source)

    poll_time = 0
    sleeper = rospy.Rate(freq)
    while not rospy.is_shutdown():
        for pipeline in pipelines:
            pipeline.consume(pipeline.source.poll(poll_time))
        poll_time += 1
        sleeper.sleep()
