#!/usr/bin/env python                                                                            
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
#Title: SPR
#Author: Ishiyama Yuki
#Data: 2020/2.20 
#Memo
#-------------------------------------------------------------------
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

#sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
#from common_action_client import *
#from common_function import *

sys.path.insert(0, '/home/athome/catakin_ws/src/mimi_voice_control/src')


class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_spr'])

    def execute(self, userdata):
        rospy.loginfo('Enter The Room')
       # speak('start pick and place')
       # enterTheRoomAC(0.8)
        return 'to_spr'

class VoiceDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['noise', 'person_speech'],
                            input_keys=['response_times_in'],
                            output_keys=['response_times_out'])

    def execute(self, userdata):
        rospy.loginfo('voicedetection')
        #音を披露まで待機
        #return 'noise'
        return 'person_speech'
        return 'noise'

class Response(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['continue', 'conpleted'],
                            input_keys=['response_times_in'],
                            output_keys=['response_times_out'])

    def execute(self, userdata):
        rospy.loginfo('Response')
        #if sm_taime <= 3 return 'continue'
        #else 
        return 'conpleted'
        return 'continue'

class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_finish'])

    def execute(self, userdata):
        rospy.loginfo('exit room')
        #Navigation
        return 'to_finish'


def main():
    sm_top = smach.StateMachine(outcomes=['FINISH'])
    with sm_top:
        #### ExitRoom
        smach.StateMachine.add('ENTER', EnterRoom(),
                            transitions={'to_spr':'SPR'})

        sm_spr = smach.StateMachine(outcomes=['to_exit'])
        sm_spr.userdata.sm_times = 0

        with sm_spr:
            smach.StateMachine.add('VOICEDETECTION', VoiceDetection(),
                    transitions={'person_speech':'RESPONSE',
                                 'noise':'VOICEDETECTION'},
                    remapping={'response_times_in':'sm_times',
                               'response_times_out':'sm_times'})

            smach.StateMachine.add('RESPONSE',Response(),
                    transitions={'continue':'VOICEDETECTION',
                                 'conpleted':'to_exit'},
                    remapping={'response_times_in':'sm_times',
                               'response_times_out':'sm_times'})

        smach.StateMachine.add('SPR', sm_spr,
                transitions={'to_exit':'EXIT'})

        smach.StateMachine.add('EXIT', ExitRoom(),
                transitions={'to_finish':'FINISH'})

    outcome = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_spr')
    main()








        
