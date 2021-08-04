#!/usr/bin/env python

import rospy
from .bridge import *

def digitaltwin_bridge_node():
    rospy.init_node('digitaltwin_bridge_node', anonymous=False)
    # Instantiate object
    mqtt_ros_bridge = MqttRosBridge()
    # spin the node
    rospy.spin()

__all__ = ['digitaltwin_bridge_node']

