#!/usr/bin/env python
# -*- coding: utf-8 -*-

from data_capture.srv import *
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from mongodb_bridge import MongoBridge
from tf.transformations import euler_from_quaternion

class Sensor(object):

    ACTIVE_SENSORS = []

    def __init__(self, sensor_name, type_name):
        Sensor.ACTIVE_SENSORS.append(self)
        self._data = {}
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


class Robot(Sensor):
    def __init__(self, sensor_name):
        super(Robot, self).__init__(sensor_name, "robot")
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/battery_level", Float32, self.battery_callback)

        self.pose = None
        self.battery_level = None

    def pose_callback(self, msg):
        pose = msg.pose.pose
        orientation_z = euler_from_quaternion([pose.orientation.x,
                                              pose.orientation.y,
                                              pose.orientation.z,
                                              pose.orientation.w])[2]
        self.pose = [pose.position.x, pose.position.y, orientation_z]

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def get_measure(self):
        self.data = {'pose': self.pose, 'battery_level': self.battery_level}
        return self.data


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

        Robot('geko')
        CameraRos('camera1', image_topic="/camera/image_raw")
        CameraRos('camera2', image_topic="/camera2/image2_raw")
        CameraRos('camera3', image_topic="/camera3/image3_raw")
        CameraRos('camera4', image_topic="/camera4/image4_raw")

        rospy.Service('make_data_acquisition', RequestSaveData, self.make_acquisition)
        rospy.spin()

    def make_acquisition(self, req):
        msgs = []
        for sensor in Sensor.ACTIVE_SENSORS:
            if sensor.get_measure():
                msg = {'class_name': sensor.type,
                       'item_name': sensor.name,
                       'data': sensor.get_measure()}
                msgs.append(msg)

        self.mongo_bridge.add_data(msgs)
        self.mongo_bridge.send_data_and_wait()

        return RequestSaveDataResponse(['succeeded'])

if __name__ == "__main__":
    DataAcquisition()