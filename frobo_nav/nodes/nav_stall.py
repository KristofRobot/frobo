#!/usr/bin/env python

""" nav_stall.py - Version 0.1 2014-12-22

    Monitor motor currents and in case of motor stall:
	cancel all current navigation goals
	execute recovery behaviour 

    Based on nav_test.py by Patrick Goebel

    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ros_arduino_msgs.msg import AnalogFloat

class NavStall():
    def __init__(self):
        rospy.init_node('nav_stall', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the move_base action server
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server()
        
        rospy.loginfo("Connected to move base server")

        #initiate bucket counter variables
        self.stallCounterBucket = 0
        self.STOP_MAX_STALL_BUCKET_COUNT = 10
        self.MAX_STALL_BUCKET_COUNT = 3
        self.hasGivenUp = 0

        self.current_topic = rospy.get_param("~current_topic")
        self.stall_current = rospy.get_param("~stall_current", 0.1)
        self.recovery_speed = rospy.get_param("~recovery_speed", 0.1)
        self.recovery_time = rospy.get_param("~recovery_time", 2)

        rospy.loginfo("Time: " + str(self.recovery_time))

        # Wait for motor current topics to become available
        rospy.loginfo("Waiting for motor current topic to become available...")
        rospy.wait_for_message(self.current_topic, AnalogFloat)

        #subscribe to motor current topics
        rospy.Subscriber(self.current_topic, AnalogFloat, self.detect_stall)
        
        rospy.loginfo("Stall detection started on " + self.current_topic)
            

    def shutdown(self):
        self.cancelAndStop()

    '''
    Implement leaky bucket for stall detection
    '''
    def detect_stall(self, msg):
        now = rospy.Time.now()
        if msg.header.stamp.secs < (now.secs-1):
            #skip messages that are older than 1 sec (stale)
            return
        '''
        if (self.hasGivenUp):
            self.givenUp()
            return

        #if we've been stalled for too long time, just give up
        if (self.stallCounterBucket > self.STOP_MAX_STALL_BUCKET_COUNT):
            self.hasGivenUp = 1
            self.givenUp()
            return
        '''     

        if (msg.value > self.stall_current):
            self.stallCounterBucket+=2;
            rospy.loginfo("Potential stall condition detected at current: " + str(msg.value) + " (stall current: " + str(self.stall_current) + ") - incremented Stall Counter to " + str(self.stallCounterBucket))
        else:
            if (self.stallCounterBucket > 0):
                self.stallCounterBucket-=1; #decrement bucket
                rospy.loginfo("Decremented Stall Counter to " + str(self.stallCounterBucket))

        if (self.stallCounterBucket > self.MAX_STALL_BUCKET_COUNT):
            rospy.logwarn("Stall conditition detected! Trying to recover...")
            self.cancelAndStop()
            self.recover()
            rospy.logwarn("Stall recovery completed.")

    def recover(self):
        rospy.loginfo("Initiating recovery... Speed: " + str(self.recovery_speed) + " Time: " + str(self.recovery_time))
        cmd_vel = Twist()
        #move back for one second at low speed
        cmd_vel.linear.x = -self.recovery_speed
        
        #need to repeat this multiple times, as base_controller will timeout after 0.5 sec
        for x in range(0, self.recovery_time*4):
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.25)
        #stop
        rospy.loginfo("Stopping the robot after recovery move...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        
    def cancelAndStop(self):
        rospy.loginfo("Canceling all goals...")
        self.move_base.cancel_all_goals()
        rospy.sleep(1)
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    '''
    def givenUp(self):
        rospy.logwarn("Stall detection has given up - all movement stopped")
        self.cancelAndStop()
    '''

if __name__ == '__main__':
    try:
        NavStall()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation stall detection finished.")
