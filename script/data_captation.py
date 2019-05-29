#!/usr/bin/env python

from data_capture.srv import *
import rospy
from sensor_msgs.msg import Image
from mongodb_bridge import MongoBridge


class Sensor(object):

    ACTIVE_SENSORS = []

    def __init__(self, sensor_name, type_name):
        Sensor.ACTIVE_SENSORS.append(self)
        self._data = None
        self._name = sensor_name.lower()
        self._type = type_name.lower()

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, data):
        self._data = data

    def get_measure(self):
        return self._data


class CameraRos(Sensor):
    def __init__(self,sensor_name, image_topic = "/usb_cam/image_raw"):
        super(CameraRos, self).__init__(sensor_name, "camera")
        rospy.Subscriber(image_topic, Image, self.camera_callback)

    def camera_callback(self, msg):
        self.data = {'image': msg}

class DataAcquisition:
    def __init__(self):
        rospy.init_node('data_acquisition')
        self.mongo_bridge = MongoBridge()
        CameraRos('camera1')
        rospy.Service('make_data_acquisition', RequestSaveData, self.make_acquisition)
        rospy.spin()

    def make_acquisition(self, req):
        msgs = []
        for sensor in Sensor.ACTIVE_SENSORS:
            msg = {'class_name': sensor.type,
                   'item_name': sensor.name,
                   'data': sensor.data}
            msgs.append(msg)

        self.mongo_bridge.add_data(msgs)
        self.mongo_bridge.send_data_and_wait()
        return RequestSaveDataResponse(['succeeded'])

if __name__ == "__main__":
    DataAcquisition()