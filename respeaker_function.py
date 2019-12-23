#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------
#Title: ReSpeakerの処理をもとめたpythonスクリプト
#Author: Ishiyama Yuki
#Data: 2019/12/17
#Memo: 
#-----------------------------------------------

#Python関係
#import yaml import load
import sys
#ROS関係
import rospy
#import rosparam
from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import PoseStamped
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from gcp_texttospeech.srv import TTS
#from mimi_common_pkg.msg import common_function

class MimiControl():
    def __init__(self):
        self.pub_cmd_vel_mux = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        #Subscriber
        self.sub_location = rospy.Subscriber('/sound_localization', PoseStamped, self.locationCB)
        self.sub_angle = rospy.Subscriber('/sound_direction', Int32, self.angleCB)
        self.location = PoseStamped()
        self.angle = Int32()
        self.twist_value = Twist()
        self.flg = False
        #Publisされている中身
        # self.pub_doa = rospy.Publisher("sound_localization", PoseStamped, queue_size=1, latch    =True) 
        # self.pub_doa_raw = rospy.Publisher("sound_direction", Int32, queue_size=1, latch=True)

    def locationCB(self, receive_msg):
        if self.flg == False:
            self.location = receive_msg.pose.orientation.z

    def angleCB(self, receive_msg):
        distance = 2.0
        if self.location < distance:
            self.angle = receive_msg
            self.flg = True

    def faceOperator(self):
        try:
            while not rospy.is_shutdown() and self.flg == False:
                print 'wait for instructions'
                rospy.sleep(1.0)
            if self.angle >= 0:
                self.twist_value.angular.z = 1.0
            elif self.angle < 0
                self.twist_value.angular.z = -1.0
            count = abs(self.angle)
            for i in range(count):
                self.pub_cmd_vel_mux.publish(self.twist_value)
                rospy.sleep(0.004)
        except rospy.ROSInterruptException:
            rospy.loginfo('**Interrupted**')
            pass

if __name__ == '__main__':
    rospy.init_node('respeaker_function')
    mi = MimiControl()
    mi.faceOperator()



