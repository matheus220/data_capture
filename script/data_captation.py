#!/usr/bin/env python

from data_capture.srv import *
import rospy
import time

def save_data(req):
    time.sleep(5)
    return RequestSaveDataResponse(['succeeded'])

def save_data_server():
    rospy.init_node('save_data_server')
    s = rospy.Service('request_save_data', RequestSaveData, save_data)
    rospy.spin()

if __name__ == "__main__":
    save_data_server()