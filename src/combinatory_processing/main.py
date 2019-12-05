import rospy
from .source import DataSourceTopic


class Subscription:
    def __init__(self, msg_type):
        self.msg_type = msg_type
        self.sources = []

    def accept(self, msg):
        for source in self.sources:
            source.enqueue(msg)


class CombinatoryProcessingInstance:
    def __init__(self, pipelines):
        self.pipelines = pipelines
        subs = dict()
        seen_sources = set()
        for pipeline in pipelines:
            for source in pipeline.source_set:
                if isinstance(source, DataSourceTopic) and source not in seen_sources:
                    seen_sources.add(source)
                    if source.topic_name not in subs:
                        sub = Subscription(source.msg_type)
                        rospy.Subscriber(source.topic_name,
                                        source.msg_type, callback=sub.accept)
                        subs[source.topic_name] = sub
                    elif subs[source.topic_name].msg_type != source.msg_type:
                        raise RuntimeError('Topic type mismatch for %s: %s != %s' % (
                            source.topic_name, subs[source.topic_name].msg_type, source.msg_type))
                    subs[source.topic_name].sources.append(source)
        self.poll_time = 0

    def spin_once(self):
        for pipeline in self.pipelines:
            pipeline.consume(pipeline.source.poll(self.poll_time))
        self.poll_time += 1


cmb_proc_instance = None


def init(name, pipelines, anon=False):
    global cmb_proc_instance
    if cmb_proc_instance is None:
        rospy.init_node(name, anonymous=anon)
        cmb_proc_instance = CombinatoryProcessingInstance(pipelines)


def spin(name, pipelines, anon=False, freq=30):
    init(name, pipelines, anon)
    sleeper = rospy.Rate(freq)
    while not rospy.is_shutdown():
        spin_once()
        sleeper.sleep()


def spin_once():
    cmb_proc_instance.spin_once()
