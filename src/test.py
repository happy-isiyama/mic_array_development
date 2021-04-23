#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------
#Author: Ishiyama Yuki
#Date 2020/2/5
#Memo
#-------------------------------------------------

import rospy
import sys
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String, Float64, Int32

class Mimicontrol():
    def __init__(self):
       self.sub_audio = rospy.Subscriber('/audio', AudioData, self.audioCB)
       self.audio = AudioData()
        # self.pub_audio = rospy.Publisher("audio", AudioData, queue_size=10)
        # self.pub_speech_audio = rospy.Publisher("speech_audio", AudioData, queue_size=10)       
    def andioCB(self, receive_msg):
        self.audio = receive_msg

    def 
