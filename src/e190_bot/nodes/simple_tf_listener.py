#!/usr/bin/env python
import rospy
import rospkg
import numpy as np

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()

class simple_tf_listener:

    def __init__(self):

        rospy.init_node('simple_tf_listener', anonymous=True)

        self.listener = tf.TransformListener()

        self.goal_pose_init()

        self.rate = rospy.Rate(20)


        while not rospy.is_shutdown():
            self.tf_lis();
            self.rate.sleep();

    def goal_pose_init(self):

        self.goalPose=PoseStamped()# goal pose, in odom frame
        self.goalPoseLocal = PoseStamped()# goal pose, in local base_link frame

        self.goalPose.header.frame_id = "odom"
        self.goalPoseLocal.header.frame_id = "base_link"

        self.goalPose.pose.position.x = 2.0
        self.goalPose.pose.position.y = 2.0
        self.goalPose.pose.position.z = .0

        self.goalPose.pose.orientation.x = .0
        self.goalPose.pose.orientation.y = .0
        self.goalPose.pose.orientation.z = .1
        self.goalPose.pose.orientation.w = .0


    def tf_lis(self):
        #Shall we put "odom" first or "base_link" first? Experiment it!
        #The example may be wrong!
        self.listener.waitForTransform("base_link","odom",rospy.Time(0),rospy.Duration(2.0))
        (trans,rot) = self.listener.lookupTransform("base_link", "odom", rospy.Time(0))

        self.goalPoseLocal = self.listener.transformPose("base_link",self.goalPose)
        print([self.goalPoseLocal.pose.position.x,self.goalPoseLocal.pose.position.y])


if __name__ == '__main__':
    try:
        tf = simple_tf_listener()

    except (rospy.ROSInterruptException):
        pass
