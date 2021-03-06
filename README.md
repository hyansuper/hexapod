# hexapod
ROS project for [Antdroid](https://antdroid.grigri.cloud/), controlled by RPi 3b and 2 Adafruit servo controller boards.<br/>
The author of Antdroid used a Raspberry Pi together with an Arduino to control the hexpod.<br/>
I use a Raspberry Pi 3b to communicate through I2C with 2 servo controller boards, without Arduino board. So I wrote my own ROS project. I implemented 3 walking gaits mentioned in [this article](https://hexyrobot.wordpress.com/2015/11/20/common-walking-gaits-for-hexapods/)

[Videos](https://www.bilibili.com/video/av50957479/)

## Warning:
* Servos should be put at the right angle in installation, otherwise the legs may conflict each other
* Upon the first test, it's highly recommended to support the body so that it dosen't apply weight to the legs.

## Software installation:
Flash the [Ubiquity image with ROS kinetic pre-installed(2019-02-19)](https://downloads.ubiquityrobotics.com/pi.html) to RPi SD card

On PC with Ubuntu:

    cd catkin_ws/src
    git clone https://github.com/phil333/face_detection.git
    git clone https://github.com/hyansuper/hexapod.git
    mv hexapod/CMakeLists.txt.for_PC hexapod/CMakeLists.txt
    cd ..
    catkin_make

On RPi:

    sudo apt-get install libi2c-dev -y
    cd catkin_ws/src
    git clone https://github.com/hyansuper/hexapod.git
    cd ..
    catkin_make
    
Then, [setup env variables on both PC and RPi for multiple machine communication in the same local network](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

## Configure the servos:
... I'll illustrate later ...<br/>
read the src/servo_driver.cpp and yaml/servo_config.yaml file and see if you can figure it out yourself, basically it's using some "rosservice call ..." commands

## Startup:
On PC:

    roscore &
    roslaunch hexapod pc_maximum.launch    
    
On RPi:

    roslaunch hexapod pi_minimum.launch
    
On any terminal:

    (stand up:)
    rostopic pub --once /rise/goal [tab][tab]
        ...
        z: 0.13
    
    (test:)
    rosrun hexapod test.py
    
    (walk:)
    rostopic pub -r 3 /vel_cmd [tab][tab]
        linear:
        x: 0.04
        ...
        
    (change gait:)
    rosservice call /set_gait "name: 'wave/tripod/ripple'"
    
Face tracking test:<br/>

On RPi:

    roslaunch hexapod camerav1_640x480.launch
    
On PC:

    roslaunch hexapod track_face.launch
    rosrun rqt_reconfigure rqt_reconfigure
    (under 'raspicam_node' tab, check 'hflip')
    (under 'face_tracking' tab, set imageInput: /raspicam_node/image, publish: Publish_Data(2))
    
    
