#!/usr/bin/env python

import rospy
import json
import datetime
from std_msgs.msg import String, Float64

def interface():
    pub = rospy.Publisher('/change_mode', String, queue_size=1)
    pub_control = rospy.Publisher('/change_control', String, queue_size=1)
    pub2 = rospy.Publisher('/geko/pan_position_controller/command', Float64, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)

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

        pub2.publish(Float64(-1.57))
        rospy.sleep(0.5)
        pub.publish(ros_msg)

    def publish_control_msg(msg, sleep_time):
        rospy.sleep(sleep_time)
        json_msg = json.dumps(msg)
        ros_msg = String(json_msg)
        rospy.loginfo("Switching to %s control.",
                      msg['mode'].upper())

        rospy.sleep(0.5)
        pub_control.publish(ros_msg)

    # while not rospy.is_shutdown():

    cycle1 = {"min_duration": 300,
              "waypoints": [{"id": 0, "x": 3.55, "y": -0.46, "theta": 0.0},
                            {"id": 1, "x": 4.35, "y": -0.46, "theta": 0.0},
                            {"id": 2, "x": 5.10, "y": -0.46, "theta": 0.0},
                            {"id": 3, "x": 5.9, "y": -0.46, "theta": 0.0},
                            {"id": 4, "x": 6.75, "y": -0.46, "theta": 0.0},
                            {"id": 5, "x": 7.55, "y": -0.46, "theta": 0.0},
                            {"id": 6, "x": 7.55, "y": -0.37, "theta": -3.14},
                            {"id": 7, "x": 6.75, "y": -0.37, "theta": -3.14},
                            {"id": 8, "x": 5.90, "y": -0.37, "theta": -3.14},
                            {"id": 9, "x": 5.10, "y": -0.37, "theta": -3.14},
                            {"id": 10, "x": 4.35, "y": -0.37, "theta": -3.14},
                            {"id": 11, "x": 3.55, "y": -0.37, "theta": -3.14},
                            {"id": 12, "x": 3.55, "y": 3.13, "theta": 0.0},
                            {"id": 13, "x": 4.35, "y": 3.13, "theta": 0.0},
                            {"id": 14, "x": 5.10, "y": 3.13, "theta": 0.0},
                            {"id": 15, "x": 5.9, "y": 3.13, "theta": 0.0},
                            {"id": 16, "x": 6.75, "y": 3.13, "theta": 0.0},
                            {"id": 17, "x": 7.55, "y": 3.13, "theta": 0.0},
                            {"id": 18, "x": 7.55, "y": 3.25, "theta": -3.14},
                            {"id": 19, "x": 6.75, "y": 3.25, "theta": -3.14},
                            {"id": 20, "x": 5.90, "y": 3.25, "theta": -3.14},
                            {"id": 21, "x": 5.10, "y": 3.25, "theta": -3.14},
                            {"id": 22, "x": 4.35, "y": 3.25, "theta": -3.14},
                            {"id": 23, "x": 3.55, "y": 3.25, "theta": -3.14}]
              }
    cycle2 = {"min_duration": 200,
              "waypoints": [{"id": 0, "x": 3.55, "y": -0.46, "theta": 0.0},
                            {"id": 1, "x": 4.35, "y": -0.46, "theta": 0.0},
                            {"id": 2, "x": 5.10, "y": -0.46, "theta": 0.0},
                            {"id": 3, "x": 5.9, "y": -0.46, "theta": 0.0},
                            {"id": 4, "x": 6.75, "y": -0.46, "theta": 0.0},
                            {"id": 5, "x": 7.55, "y": -0.46, "theta": 0.0},
                            {"id": 6, "x": 7.55, "y": 3.25, "theta": -3.14},
                            {"id": 7, "x": 6.75, "y": 3.25, "theta": -3.14},
                            {"id": 8, "x": 5.90, "y": 3.25, "theta": -3.14},
                            {"id": 9, "x": 5.10, "y": 3.25, "theta": -3.14},
                            {"id": 10, "x": 4.35, "y": 3.25, "theta": -3.14},
                            {"id": 11, "x": 3.55, "y": 3.25, "theta": -3.14}]
              }

    msg = {"mode": "patrol",
           "default_mode": True,
           "stamp": rospy.Time.now().to_sec(),
           "cycles": [cycle1, cycle2]
           }

    publish_msg(msg, 5.0)

    # msg = {"mode": "assistance",
    #        "stamp": str(datetime.datetime.now())
    #        }
    #
    # publish_msg(msg, 30.0)
    #
    # msg = {"mode": "assisted_teleop"}
    #
    # publish_control_msg(msg, 3.0)
    #
    # msg = {"mode": "semi_autonomous",
    #        "waypoint": {"x": 3.55, "y": -0.46, "theta": 0.0}}
    #
    # publish_control_msg(msg, 20.0)
    #
    # msg = {"mode": "recharge"}
    #
    # publish_control_msg(msg, 30.0)
    #
    # msg = {"mode": "exit"}
    #
    # publish_control_msg(msg, 30.0)

if __name__ == '__main__':
    try:
        interface()
    except rospy.ROSInterruptException:
        pass