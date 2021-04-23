#!/usr/bin/env python                                                                            
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
#Title: SPR
#Author: Ishiyama Yuki
#Data: 2020/2.20 
#Memo
#-------------------------------------------------------------------
import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *
sys.path.insert(0, '/home/athome/catakin_ws/src/mimi_voice_control/src')
from voice_common_pkg.srv import WhatDidYouSay
for spr_speak.srv import SprInformation 


class WatingAndturn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_RIDDLE'])

    def execute(self, userdata):
        rospy.loginfo('start speech and person recoginition')
        speak('Start riddle game')
        rospy.sleep(100)
        #群衆の人数を数える。できれば男女も見分ける。
        speak('Five people play games with me. There are three men among them')
        return 'to_RIDDLE'

class RiddleGame(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_BMB'],
                            input_keys=['question_times_in'],
                            output_keys=['question_times_out'])
        rospy.wait_for_service('/spr_speak')
        self.question_srv = rospy.Service('/spr_speak',SprInformation)
        self.human =10

    def execute(self, userdata):
        angleRotation(180)
        result = question_srv(self.human)
        speak(result.ans_str)
        question_times_out = question_times_in + 1
        if question_times_out => 5:
            question_times_out = 0
            return 'game_end'
        else:
            return 'game_continues'

class BlindManBluffGame(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['noise', 'person_speech'],
                            input_keys=['question_times_in'],
                            output_keys=['question_times_out'])
        rospy.wait_for_service('/spr_speak')
        self.question_srv = rospy.Service('/spr_speak',SprInformation)


    def execute(self, userdata):
        rospy.loginfo('Start Blind Man bluff Game')
        result = question_srv(self.human)
        angleRotation(result.resoeaker_angle)
        speak(result.ans_str)
        question_times_out = question_times_in + 1
        if question_times_out >= 5:
            return 'game_end'
        else:
            return 'game_continues'

class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('exit')
        location_list = searchLocationName('exit')
        navigationAC(location_list)
        return 'to_finish'


def main():
    sm_top = smach.StateMachine(outcomes=['FINISH'])
    sm_top.userdata.sm_times = 0
    with sm_top:
        #### ExitRoom
        smach.StateMachine.add('WAT', WatingAndturn(),
                transitions={
                    'to_RIDDLE':'RIDDL'})

        smach.StateMachine.add('RIDDL', RiddleGame(),
                transitions={
                    'game_continues':'RIDDL',
                    'game_end':'BLINDMANBLUFF'},
                remapping={
                    'question_times_in':'sm_times',
                    'question_times_out':'sm_times'})


        smach.StateMachine.add('BLINDMANBLUFF', BlindManBluffGame(),
                transitions={
                    'game_continues':'BLINDMANBLUFF',
                    'game_end':'to_exit'},
                remapping={
                    'question_times_in':'sm_times',
                    'question_times_out':'sm_times')}


        smach.StateMachine.add('EXIT':ExitRoom(),
                transitions={
                    'to_finish':'FINISH'})


if __name__ == '__main__':
    rospy.init_node('sm_spr')
    main()
        
