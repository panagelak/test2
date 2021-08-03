#!/usr/bin/env python

import math
import random
import rospy
import rospkg
import tf
import sys

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose


def spawn_model(postfix, model_name, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w):
    """ Spawns a model.
        Args: None
        Returns: None
    """
    initial_pose = Pose()

    initial_pose.position.x = pos_x
    initial_pose.position.y = pos_y
    initial_pose.position.z = pos_z
    initial_pose.orientation.x = ori_x
    initial_pose.orientation.y = ori_y
    initial_pose.orientation.z = ori_z
    initial_pose.orientation.w = ori_w

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    # Spawn the new model #
    model_path = rospkg.RosPack().get_path('perception_gazebo')+'/models/'
    model_xml = ''

    with open(model_path + model_name + '/model.sdf', 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(model_name + postfix, model_xml,
                     '', initial_pose, 'world')


def spawn_fences_start():
    ###########################
    ######SPAWN FENCES#########
    ###########################
    
    spawn_model(postfix="", model_name="george", pos_x=0.0,   pos_y=-0.8,
                pos_z=0, ori_x=0, ori_y=0, ori_z=1.0, ori_w=0.0)


if __name__ == '__main__':

    rospy.init_node('spawn_models_node')
    x=5
    y=3
    z=[2,3]
    rospy.loginfo("This node spawns the models in the simulation on startup")

    rospy.loginfo("Spawning Models")
    spawn_fences_start()
    rospy.loginfo("Models Spawned")
