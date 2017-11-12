#! /usr/bin/env python2
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import Image, CompressedImage, Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Float32
import cv2
import numpy as np
import rospy
import threading
import time
import math
import sys
from cv_bridge import CvBridge, CvBridgeError
from mvnc import mvncapi as mvnc
class NcsCaffePredictionNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.bridge = CvBridge()
		self.model_name = rospy.get_param('~caffe_model')
		self.omega_weight = rospy.get_param('~omega_weight')
		rospy.loginfo('[%s] caffe model name = %s' %(self.node_name, self.model_name))
		self.model_Base_Dir = '../models/' + self.model_name + '/'

		#setup device and parameter
		self.count = 0
		self.gain_step = 0
		self.initial()
		
		self.sub_image = rospy.Subscriber("~compressed", CompressedImage, self.cbImage, queue_size=1)
		self.sub_gain_step = rospy.Subscriber("~gain_step", Float32, self.cbGainStep, queue_size=1)

		self.pub_carcmd = rospy.Publisher("~carcmd", Twist2DStamped, queue_size=1)
		
		

	def initial(self):

		self.device_work = False
		mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)
		self.deviceCheck()
		shape_txt = ""
		shape = []
		with open(self.model_Base_Dir+'input_shape.prototxt', 'r') as file:
			shape_txt = file.read().replace('\n', ' ')
		for s in shape_txt.split():
			if s.isdigit():
				shape.append(int(s))
		
		self.dim = (shape[2], shape[3])


	def deviceCheck(self):
		#check device is plugged in
		self.devices = mvnc.EnumerateDevices()
		if len(self.devices) == 0:
			self.device_work = False
			rospy.loginfo('[%s] NCS device not found' %(self.node_name))
			
		else:
			self.device_work = True
			rospy.loginfo('[%s] NCS device found' %(self.node_name))
			self.initialDevice()

	def initialDevice(self):
		# set the blob, label and graph
		self.device = mvnc.Device(self.devices[0])

		labels_file=self.model_Base_Dir + 'label.txt'
		self.labels = np.loadtxt(labels_file,str,delimiter='\t')
		
		self.device.OpenDevice()

		network_blob=self.model_Base_Dir + 'graph'

		#Load blob
		with open(network_blob, mode='rb') as f:
			blob = f.read()

		self.graph = self.device.AllocateGraph(blob)

	def cbGainStep(self, msg):
		self.gain_step = msg.data

	def cbImage(self, msg):
		#receive img from camera
		self.count += 1
		if self.device_work == True and self.count == 4:
			# Load the image
			self.count = 0
			np_arr = np.fromstring(msg.data, np.uint8)
			img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)	
			img = cv2.resize(img, self.dim)
			img = img.astype(np.float32)
			
			img_m = np.zeros((self.dim[0], self.dim[1], 1), np.float32)
			img_m[:] = (128.0)
			img = cv2.subtract(img, img_m)	
			img = img * 0.0078125
			
			# Send the image to the NCS
			self.graph.LoadTensor(img.astype(np.float16), 'user object')

			output, userobj = self.graph.GetResult()

			order = output.argsort()[::-1][:4]
			#print('\n------- predictions --------')
			#for i in range(0, 3):
				#print ('prediction ' + str(i) + ' (probability ' + str(output[order[i]]*100) + '%) is ' + self.labels[order[i]] + '  label index is: ' + str(order[i]) )

			self.tf_pred2cmd(output, order, msg.header)

	def tf_pred2cmd(self, output, order, header):
		carcmd_msg = Twist2DStamped()
		carcmd_msg.header = header
		carcmd_msg.omega = 0
		carcmd_msg.v = 0.6 + self.gain_step

		for i in range(0, 3):
			if(order[i]==0): #L
				carcmd_msg.omega += self.tf_probs2omega(output[order[i]]) * self.omega_weight[0][0]
			elif order[i]==2:#R
				carcmd_msg.omega += self.tf_probs2omega(output[order[i]]) * self.omega_weight[0][2]
			else:
				carcmd_msg.omega += self.tf_probs2omega(output[order[i]]) * self.omega_weight[0][1]
		
		#print 'omega = ', carcmd_msg.omega
		self.pub_carcmd.publish(carcmd_msg)

				
	def tf_probs2omega(self, prob):
		prob_left = 0.2
		prob_right = 0.8
		o = 1/(1+math.exp(-prob)) 
		o = o * 9

		if(prob <= prob_left):
		    o = prob/1.2
		elif(prob >= prob_right):
		    o = 1.8+prob/(3-1.8)
		else:
		    o = 1.2+prob/(1.8-1.2)

		return o

	def onShutdown(self):
		#cloe divice when shutdown
		if(self.device_work==True):
			self.device_work=False
			rospy.sleep(0.5)
			self.graph.DeallocateGraph()
			self.device.CloseDevice()

if __name__ == '__main__':
	rospy.init_node('ncs_caffe_prediction',anonymous=False)
	ncs_caffe_prediction_node = NcsCaffePredictionNode()
	rospy.on_shutdown(ncs_caffe_prediction_node.onShutdown)
	rospy.spin()
