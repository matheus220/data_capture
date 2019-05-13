#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String

def interface():
    pub = rospy.Publisher('/change_mode', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.0033333)

    def publish_msg(msg, sleep_time):
        rospy.sleep(sleep_time)
        json_msg = json.dumps(msg)
        ros_msg = String(json_msg)
        if 'pause' in msg:
            rospy.loginfo("Switching to %s mode with %d waypoints and %d seconds pause between each cycle.",
                      msg['mode'].upper(), len(msg['waypoints']), msg['pause'])
        elif 'waypoints' in msg:
            rospy.loginfo("Switching to %s mode with %d waypoints.",
                          msg['mode'].upper(), len(msg['waypoints']))
        else:
            rospy.loginfo("Switching to %s mode.",
                          msg['mode'].upper())

        pub.publish(ros_msg)

    # while not rospy.is_shutdown():

    msg = {"mode": "manual",
           "stamp": rospy.Time.now().to_sec()
           }

    publish_msg(msg, 10.0)

    msg = {"mode":"patrol",
           "pause":10,
           "waypoints":[{"id": 0, "x":-0.5, "y":-0.5, "theta":0},
                        {"id": 1, "x":-0.5, "y":0.5, "theta":0},
                        {"id": 2, "x":0.5, "y":0.5, "theta":0},
                        {"id": 3, "x":0.5, "y":-0.5, "theta":0}],
           "stamp": rospy.Time.now().to_sec()
           }

    publish_msg(msg, 100.0)

    msg = {"mode": "patrol",
           "pause": 5,
           "waypoints": [{"id": 0, "x": -1.8, "y": 0, "theta": 1.57},
                         {"id": 1, "x": 0, "y": -1.8, "theta": 0},
                         {"id": 2, "x": 1.8, "y": 0, "theta": 1.57},
                         {"id": 3, "x": 0, "y": 1.8, "theta": 0},
                         {"id": 4, "x": -1.6, "y": 0.55, "theta": 0},
                         {"id": 5, "x": -1.6, "y": -0.55, "theta": 1.57}],
           "stamp": rospy.Time.now().to_sec()
           }

    publish_msg(msg, 70.0)

    msg = {"mode": "verification",
           "waypoints": [{"id": 0, "x": -0.7, "y": -2.0, "theta": 0}],
           "stamp": rospy.Time.now().to_sec()
           }

    publish_msg(msg, 100.0)


    #    rate.sleep()




if __name__ == '__main__':
    try:
        interface()
    except rospy.ROSInterruptException:
        pass