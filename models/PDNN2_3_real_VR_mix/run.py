#! /usr/bin/env python3

from mvnc import mvncapi as mvnc
import sys
import numpy as np
import cv2
import time
import csv
import os

dim=(101,101)
EXAMPLES_BASE_DIR='../../'

# ***************************************************************
# get labels
# ***************************************************************
labels_file='lab_list.txt'
labels=np.loadtxt(labels_file,str,delimiter='\t')

# ***************************************************************
# configure the NCS
# ***************************************************************
mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)

# ***************************************************************
# Get a list of ALL the sticks that are plugged in
# ***************************************************************
devices = mvnc.EnumerateDevices()
if len(devices) == 0:
	print('No devices found')
	quit()

# ***************************************************************
# Pick the first stick to run the network
# ***************************************************************
device = mvnc.Device(devices[0])

# ***************************************************************
# Open the NCS
# ***************************************************************
device.OpenDevice()

network_blob='graph'

#Load blob
with open(network_blob, mode='rb') as f:
	blob = f.read()

graph = device.AllocateGraph(blob)

# ***************************************************************
# Load the image
# ***************************************************************
#ilsvrc_mean = np.load(EXAMPLES_BASE_DIR+'data/ilsvrc12/ilsvrc_2012_mean.npy').mean(1).mean(1) #loading the mean file
#img = cv2.imread(EXAMPLES_BASE_DIR+'data/images/road_right.jpg')
img = cv2.imread('/home/tony/Desktop/data/test/frame0346.jpg')

img = cv2.resize(img, dim)
img = img.astype(np.float32)
img_m = np.zeros((dim[0], dim[1] ,3), np.float32)
img_m[:] = (128.0, 128.0, 128.0)
			
img = cv2.subtract(img, img_m)	
img = img * 0.0078125

# ***************************************************************
# Send the image to the NCS
# ***************************************************************
graph.LoadTensor(img.astype(np.float16), 'user object')

# ***************************************************************
# Get the result from the NCS
# ***************************************************************
output, userobj = graph.GetResult()

# ***************************************************************
# Print the results of the inference form the NCS
# ***************************************************************
order = output.argsort()[::-1][:4]
print('\n------- predictions --------')
for i in range(0,3):
	print ('prediction ' + str(i) + ' (probability ' + str(output[order[i]]*100) + '%) is ' + labels[order[i]] + '  label index is: ' + str(order[i]) )


# ***************************************************************
# Clean up the graph and the device
# ***************************************************************
graph.DeallocateGraph()
device.CloseDevice()
    



