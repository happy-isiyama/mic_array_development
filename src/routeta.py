#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------
#Title: 
#Author: Ishiyama Yuki
#Dara: 2019/12/23
#Memo:
#------------------------------------------------

#python関係
import time
import sys
import rospy
from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import Twist


class Rotate():
    def __init__(self):
        self.pub_cmd_vel_mux = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        self.twist_value = Twist()

    def rotate_mimi(self):
        angle =  input () 
        target_time = abs(angle*0.027)
        if angle  >= 0:
            self.twist_value.angular.z = 1.0
        elif angle < 0:
            self.twist_value.angular.z = -1.0
        rate = rospy.Rate(250)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.pub_cmd_vel_mux.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('routeta')
    ro = Rotate()
    ro.rotate_mimi()

