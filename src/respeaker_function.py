#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------
#Title: ReSpeakerの処理をもとめたpythonスクリプト
#Author: Ishiyama Yuki
#Data: 2019/12/17
#Memo: 
#-----------------------------------------------

#Python関係
import time
import sys
#ROS関係
import rospy
from std_msgs.msg import String, Float64, Int32

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_function import *

class MimiControl():
    def __init__(self):
        #Subscriber
        self.sub_angle = rospy.Subscriber('/sound_direction', Int32, self.angleCB)
        self.angle = Int32()
        self.flag = False

    def angleCB(self, receive_msg):
        self.angle = receive_msg
        self.flag = True

    def faceOperator(self):
        try:
            while not rospy.is_shutdown() and self.flag == False:
                print 'wait for sound..'
                rospy.sleep(1.0)
            self.flag = False
            angleRotation(self.angle)
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass

if __name__ == '__main__':
    rospy.init_node('respeaker_function')
    mi = MimiControl()
    mi.faceOperator()



