# hexapod
ROS project for [antdroid](https://antdroid.grigri.cloud/), controlled by RPi 3b and 2 Adafruit servo controller boards.

Flash the [Ubiquity image with ROS kinetic pre-installed(2019-02-19)](https://downloads.ubiquityrobotics.com/pi.html) to RPi SD card

On PC with Ubuntu:

    cd catkin_ws/src
    git clone https://github.com/phil333/face_detection.git
    git clone https://github.com/hyansuper/hexapod.git
    mv hexapod/CMakeLists.txt.for_PC hexapod/CMakeLists.txt
    cd ..
    catkin_make

On RPi:

    cd catkin_ws/src
    git clone https://github.com/hyansuper/hexapod.git
    cd ..
    catkin_make
    
[Setup env variables on both PC and RPi for multiple machine communication in the same local network](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)

then, on PC:

    roslaunch hexapod pc_maximum.launch
    
On RPi:

    roslaunch hexapod pi_minimum.launch
