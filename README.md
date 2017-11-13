# Deep Lane Following 
## Introduction
This is the ROS pkg about deep lane following with Neural Compute Stick Caffe. The caffe model is based on one paper about drone navigation to make adjustments to our need. 
 * Reference paper : [XXXX]()
 * System : Raspberry Pi 3
 * [NCS Caffe Support](https://github.com/movidius/ncsdk/blob/master/docs/Caffe.md) - more information about caffe on NCS

## Prerequisites
### Installing
Install [NCS SDK](https://developer.movidius.com/start) for your Pi 3.</br> 
Because we just focus on using caffe, you can skip install tensorflow in ncsdk.conf. It can be installed much faster.

```
$ vim ncsdk.conf
```
```
MAKE_PROCS=1
SETUPDIR=/opt/movidius
VERBOSE=yes
SYSTEM_INSTALL=yes
INTEL_CAFFE=no
CAFFE_USE_CUDA=no
INSTALL_TENSORFLOW=yes  # change yes to no
INSTALL_TOOLKIT=yes
```

### Testing 

[examples video](https://www.youtube.com/watch?v=fESFVNcQVVA) - GoogleNet in NCS caffe, the example starts from 2:00 in the video.
 
### Trobleshooting
If you get error suck like this when running examples, it is an error caused by matplotlib and X11 forwarding.
```
Gdk.Cursor.new(Gdk.CursorType.FLEUR), TypeError: constructor returned NULL
```
two ways to slove it
1. add "-X" in command line when you ssh to your Pi3
```
ssh -X hostname
```
2. reinstall matplotlib
```
sudo pip uninstall matplotlib
sudo apt-get install python3-matplotlib
```

## How to run the deep lane following
### Hardware

### Environment

### Software
clone this repo to your duckietown's catkin_ws/src/, and then catkin_make.
After finishing all, run this line.
```
roslaunch deep_lane_following deep_lane_following.launch veh:=your_duckiebot caffe_model:=trailnet
```
