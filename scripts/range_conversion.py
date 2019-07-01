#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range


class RangeConversion:
    def __init__(self):
        rospy.init_node('range_conversion', anonymous=True)

        self.default_measure = 0.37
        self.range_min = self.default_measure - 0.04
        self.range_max = self.default_measure + 0.04

        self.topics = ["/geko/range/wheelbackleft",
                       "/geko/range/wheelbackright",
                       "/geko/range/wheelfrontleft",
                       "/geko/range/wheelfrontright"]

        self.topics_and_subs = [(topic, rospy.Publisher(topic + "_modified", Range, queue_size=1)) for topic in self.topics]

        for topic, pub in self.topics_and_subs:
            rospy.Subscriber(topic, Range, self.converter, (pub, ))

        rospy.spin()

    def converter(self, data, args):
        pub = args[0]
        if data.range > self.range_max:
            data.range = self.default_measure
        elif data.range > self.range_min:
            data.range = data.max_range + 0.1

        pub.publish(data)

if __name__ == '__main__':
    RangeConversion()