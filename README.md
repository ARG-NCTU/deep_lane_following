# Deep Lane Following 
## Introduction
This is the ROS pkg about deep lane following with Neural Compute Stick Caffe. The caffe model is based on one paper about drone navigation to make adjustments to our need. 
 * Reference paper : [a machine learning approach to visual perception of forested trails for mobile robots](http://rpg.ifi.uzh.ch/docs/RAL16_Giusti.pdf)
 * System : Raspberry Pi 3
 * NCS Caffe : [NCS Caffe Support](https://github.com/movidius/ncsdk/blob/master/docs/Caffe.md)

## Prerequisites
### Installing
1. Install [NCS SDK](https://developer.movidius.com/start) for your Pi 3.</br> 
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
2. Don't run $ make examples. It cause some problems when install opencv and tersorflow examples(because we didn't install tensorflow).</br>
We only need to make the single model we want to test in examples/caffe.

### Testing 

[example video](https://www.youtube.com/watch?v=fESFVNcQVVA) - GoogleNet in NCS caffe, the example starts from 2:00 in the video.

 
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
* Pi camera with angle down 30 degrees

### Environment
* Yellow and blue line for each 5cm width
* Turn angle of line segments does not exceed 30 degrees. If you want to make a big turn, please divide into several lines to turn.

### Software
* clone this repo to your duckietown's catkin_ws/src/, and then catkin_make.
After finishing all, run this line.
```
source environment.sh 
source set_ros_master.sh your_duckiebot_name
roslaunch deep_lane_following deep_lane_following.launch veh:=your_duckiebot_name caffe_model:=trailnet
```
Press "start" on joystick to change "Joystick Control" or "Prediction Control" </br >
Press "Y" or "A" on joystick to change "increase gain" or "decrease gain"

* It your car turn too slow, you can modify omega_weight
```
vim (your catkin_ws folder)/src/deep_lane_following/config/baseline/deep_lane_following/ncs_caffe_prediction_node/default.yaml
```
```
# weight of omega
omega_weight: [ [-1.9,0.0,1.9] ]  # first and third parameters in this line
```


## How to use NCS with your own caffemodel
This part is the instruction about how to use your own caffemodel in your own code. You can create a folder in exampls/caffe. </br>
We use Trailnet for example
### 1. prepare the file ".caffemodel", ".prototxt", "run.py" and "Makefile"
Our caffemodel and prototxt can be downloaded [here](https://drive.google.com/file/d/1t20Ew4e6JfV_cbh8qry3eujEE1M4_5FR/view?usp=sharing)
```
mkdir (your NCS examples folder path)/caffe/TrailNet
mv (your file folder)/trailnet.caffemodel (your NCS examples folder path)/caffe/TrailNet/
mv (your file folder)/deploy.prototxt (your NCS examples folder path)/caffe/TrailNet/
cp (your NCS examples folder path)/caffe/GoogLeNet/Makefile (your NCS examples folder path)/caffe/TrailNet/Makefile
cp (your NCS examples folder path)/caffe/GoogLeNet/run.py (your NCS examples folder path)/caffe/TrailNet/run.py
```
### 2. modify Makefile
```
vim (your NCS examples folder path)/caffe/TrailNet/Makefile
```
modify line 11-15, 28-61, 125-128 in original Makefile, please see example_Makefile

### 3. compile. If it succeeds, it will produce 'graph' and this is what we need to do prediction.
```
make compile
```
