#!/usr/bin/env python
import rospy
import copy
import cv2
import numpy as np
import time
import sys
import rospkg
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from duckietown_msgs.srv import  SetValue
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class StateSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped,queue_size=1)
        
        # Subscriptions
        self.sub_prediction_car_cmd = rospy.Subscriber("~prediction_car_cmd", Twist2DStamped, self.cbPrediction, queue_size=1)
        self.sub_joy_car_cmd = rospy.Subscriber("~joy_car_cmd", Twist2DStamped, self.cbJoyCarCmd, queue_size=1)
        self.sub_switch = rospy.Subscriber("~joystick_override", BoolStamped, self.cbSwitch, queue_size=1)
        self.switch = "JoyStick"  
    
    def cbPrediction(self, prediction_msg):
        if self.switch == "Prediction":
            self.pub_car_cmd.publish(prediction_msg)

    def cbJoyCarCmd(self, joy_car_cmd_msg):
        if self.switch == "JoyStick":
            self.pub_car_cmd.publish(joy_car_cmd_msg)

    def cbSwitch(self, switch_msg):
        #print switch_msg.data
        if (switch_msg.data == True) :
            if (self.switch == "JoyStick") :
                self.switch = "Prediction"
                rospy.loginfo('[%s] Prediction Control' %self.node_name)
            elif(self.switch == "Prediction"):
                self.switch = "JoyStick"
                rospy.loginfo('[%s] Joystick Control' %(self.node_name))


    def onShutdown(self):
        rospy.loginfo('[%s] Closing Control Node.' %(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('state_switch_node',anonymous=False)
    state_switch_node = StateSwitchNode()
    rospy.on_shutdown(state_switch_node.onShutdown)
    rospy.spin()
