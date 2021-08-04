#!/usr/bin/env python

import rospy
from digitaltwin_bridge.app import digitaltwin_bridge_node

try:
    digitaltwin_bridge_node()
except rospy.ROSInterruptException:
    pass