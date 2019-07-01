#!/usr/bin/env python

"""
    battery_simulator.py - Version 1.0 2013-03-18
    
    Publish a simulated battery level (0-100) with a configurable runtime.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from std_msgs.msg import Float32
from data_capture.srv import *
import thread

class BatterySimulator():
    def __init__(self):
        rospy.init_node("battery_simulator")
        
        # The rate at which to publish the battery level
        self.rate = rospy.get_param("~rate", 1)
        
        # Convert to a ROS rate
        r = rospy.Rate(self.rate)
        
        # The battery runtime in seconds
        self.battery_runtime = rospy.get_param("~battery_runtime", 30) # seconds
        
        # The intial battery level - 100 is considered full charge
        self.initial_battery_level = rospy.get_param("~initial_battery_level", 100)
        
        # Error battery level for diagnostics
        self.error_battery_level = rospy.get_param("~error_battery_level", 20)
        
        # Warn battery level for diagnostics
        self.warn_battery_level = rospy.get_param("~warn_battery_level", 50)      
        # Initialize the current level variable to the startup level
        self.current_battery_level = self.initial_battery_level
        
        # Initialize the new level variable to the startup level
        self.new_battery_level = self.initial_battery_level
        
        # The step sized used to decrease the battery level on each publishing loop
        self.battery_step = float(self.initial_battery_level) / self.rate / self.battery_runtime

        # Reserve a thread lock
        self.mutex = thread.allocate_lock()
        
        # Create the battery level publisher
        battery_level_pub = rospy.Publisher("battery_level", Float32, queue_size=5)
        
        # A service to maually set the battery level
        rospy.Service('~set_battery_level', SetBatteryLevel, self.SetBatteryLevelHandler)

        rospy.loginfo("Publishing simulated battery level with a runtime of " + str(self.battery_runtime) + " seconds...")

        # Start the publishing loop
        while not rospy.is_shutdown():
            
            battery_level_pub.publish(self.current_battery_level)
            
            self.current_battery_level = max(0, self.current_battery_level - self.battery_step)
                        
            r.sleep()
            
    def SetBatteryLevelHandler(self, req):
        self.mutex.acquire()
        self.current_battery_level = req.value
        self.mutex.release()
        return SetBatteryLevelResponse()

if __name__ == '__main__':
    BatterySimulator()
