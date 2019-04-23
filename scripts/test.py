#!/usr/bin/env python

import combinatory_processing as ros
from std_msgs.msg import Int64
from time import time

if __name__ == '__main__':
    ros.spin('systime_millis', [
        ros.source_poll(time) \
            .map(lambda x: int(x * 1000)) \
            .map(Int64) \
            .sink('systime_millis', Int64)
    ])
