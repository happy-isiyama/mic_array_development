#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------
#Title:前後進をスムーズにするコードのお試し
#Author: Ishiyama Yuki
#Data: 2020/3/29
#----------------------------------

import sys
import time

import rospy
import rosparam
from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class BaseCarrier():
    def __init__(self):
        # Publisher
        self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop',
                                         Twist, queue_size = 1)
        # Value
#        self.twist_value = Twist())

key_mapping = {[ 0, 1], [ 0, -1]}
twist_pub = None
g_target_twist = None 
g_last_twist = None
g_last_send_time = None
g_vel_scales = [0.1, 0.1] # default to very slow 
g_vel_ramps = [1, 1] # units: meters per second^2self.twist_value = Twist()

    # BEGIN RAMP
    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate):
        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        if error < step: # we can get there within this timestep. we're done.
            return v_target
        else:
            return v_prev + sign * step  # take a step towards the target
    # END RAMP


    def ramped_twist(self, prev, target, t_prev, t_now, ramps):
        tw = Twist()
        tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
                             t_now, ramps[0])
        tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
                               t_now, ramps[1])
        return tw


    def send_twist(self):
        global g_last_twist_send_time, g_target_twist, g_last_twist,\
        g_vel_scales, g_vel_ramps, g_twist_pub
        t_now = rospy.Time.now()
        g_last_twist = ramped_twist(g_last_twist, g_target_twist,
                                    g_last_twist_send_time, t_now, g_vel_ramps)
        g_last_twist_send_time = t_now
        g_twist_pub.publish(g_last_twist)

    def keys(self, distance):
        global g_target_twist, g_last_twist, g_vel_scales
        if distance >0:
            vels = key_mapping[0]
        if distance < 0:
            vels = key_mapping[1]
        g_target_twist.angular.z = vels[0] * g_vel_scales[0]
        g_target_twist.linear.x  = vels[1] * g_vel_scales[1]

    def fetch_param(self, name, default):
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
            return default





    def translateDist(self, distance):
        g_last_twist_send_time = rospy.Time.now()
        g_target_twist = Twist() # initializes to zero
        g_last_twist = Twist()
        keys(distance)
        g_vel_scales[0] = fetch_param('~angular_scale', 0.1)
        g_vel_scales[1] = fetch_param('~linear_scale', 0.1)
        g_vel_ramps[0] = fetch_param('~angular_accel', 1.0)
        g_vel_ramps[1] = fetch_param('~linear_accel', 1.0)
        
        rate = rospy.Rate(20)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.pub_twist.publish(send_twist)
            rate.sleep()


if __init__ == '__main__':
    rospy.init_node('keys_to_twist')
    tr = translateDist()
    tr.translateDist(1)

    # 指定した距離を並進移動
 #   def translateDist(self, distance):
 #       target_time = abs(distance / 0.15)
 #       if distance >0:
 #           self.twist_value.linear.x = 0.23
 #       elif distance < 0:
 #           self.twist_value.linear.x = -0.23
 #       rate = rospy.Rate(500)
 #       start_time = time.time()
 #       end_time = time.time()
 #       while end_time - start_time <= target_time:
 #           self.pub_twist.publish(self.twist_value)
 #           end_time = time.time()
 #           rate.sleep()
 #       self.twist_value.linear.x = 0.0
 #       self.pub_twist.publish(self.twist_value)
