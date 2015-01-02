#!/usr/bin/env python

"""
  Based on email conversation with Charel Van Hoof 
  and voice_nav.py - Version 1.1 2013-12-20
  
  Allows controlling a mobile base using simple speech commands.
  
  Based on the voice_cmd_vel.py script by Michael Ferguson in
  the pocketsphinx ROS package.
  
  See http://www.ros.org/wiki/pocketsphinx
"""

import rospy
import actionlib
import datetime
from std_msgs.msg import String
from math import copysign
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sound_play.libsoundplay import SoundClient

class VoiceNav:
    def __init__(self, script_path):
        rospy.init_node('voice_nav_move')
        
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)

        # A flag to determine whether or not voice control is paused
        self.paused = False

        self.passive = True
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)
 
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort'],
                                    'living room': ['living', 'room'],
                                    'help': ['help', 'help me'],
                                    'hall': ['hole', 'hal'],
                                    'kitchen': ['kitchen'],
                                    'thank you': ['thank you', 'thanks'],
                                    'frobo': ['frobo'],
                                    'time': ['time', 'what is the time', 'tell me the time'],
                                    'date': ['date', 'what is the date', 'tell me the date']}
        
        rospy.loginfo("Ready to receive voice commands")

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()
        
        self.locations['hall'] = Pose(Point(-0.486, -0.689, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
        self.locations['living room'] = Pose(Point(1.623, 7.880, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))
        self.locations['kitchen'] = Pose(Point(-1.577, 6.626, 0.000), Quaternion(0.000, 0.000, 1.000, 0.000))

    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
        #dont react upon anything if speech recognition is paused
        if self.paused:
           return
	
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
        

        if self.passive:       
        # state passive listening untill attention is requested
            rospy.loginfo("Passive listening: " + str(command) + " ( " + str(msg.data) + " ) ")
            if command == 'stop': 
                # Stop the robot!  Publish a Twist message consisting of all zeros.         
                self.move_base.cancel_all_goals()
                self.msg = Twist()
                self.cmd_vel_pub.publish(self.msg)
            if command == 'frobo':
                #rospy.sleep(1)
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
                self.passive = False    
		rospy.loginfo("Activated active listening")
            return           
        else:
            # Active listening, fetch real command    
            rospy.loginfo("Active listening: " + str(command) + " ( " + str(msg.data) + " ) ")
            #return to passive mode
            self.passive = True
            #Speak the recognized words in the selected voice
            self.soundhandle.say(msg.data, self.voice)  
        
            # The list of if-then statements should be fairly
            # self-explanatory
            if command == 'stop': 
                # Stop the robot!  Publish a Twist message consisting of all zeros.         
                self.move_base.cancel_all_goals()
                self.msg = Twist()
                self.cmd_vel_pub.publish(self.msg)
                rospy.loginfo("Stopped")
            
            elif command == 'time':
                self.paused=True
                now = datetime.datetime.now()
                self.soundhandle.say("the time is " + str(now.hour) + " hours and " + str(now.minute) + " minutes", self.voice)
                self.paused=False

            elif command == 'date':
                self.paused=True
                now = datetime.datetime.now()
                self.soundhandle.say("the date is " + now.strftime("%B") + str(now.day), self.voice)
                self.paused=False

            elif command == 'thank you':
                self.paused=True
                self.soundhandle.say("you are welcome", self.voice)
                self.paused=False
            
            elif command in self.locations:
                self.paused=True
                rospy.loginfo("OK, going to " + command)
                self.soundhandle.say("OK, going to " + command, self.voice)
                rospy.sleep(4)
                self.paused=False

                # Set up the next goal location
                self.goal = MoveBaseGoal()

                self.goal.target_pose.pose = self.locations[command] 
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()

                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)
       
                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
         
                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")

                else:
                   state = self.move_base.get_state()
                   if state == GoalStatus.SUCCEEDED:
                       rospy.loginfo("Goal succeeded!")
                   else:
                       rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))


            elif command == 'help':
                self.paused=True
                self.soundhandle.say("Hello my name is Frobo. I support the following commands: help, stop, date, time, kitchen, hall, living room", self.voice)
                rospy.sleep(32)
                self.paused=False

 
            elif command == 'frobo':
        	self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
                self.passive = False
                    
            else:
                return


    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceNav(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")

