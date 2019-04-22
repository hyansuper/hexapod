#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import math
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion



def rotate(rate,pub):
    p=Pose()
    a=0
    while a<.2 and not rospy.is_shutdown():
        a+=0.1/20
        q=quaternion_from_euler(0,0,a)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()
    a=.2
    while a>-.2 and not rospy.is_shutdown():
        a-=.1/20
        q=quaternion_from_euler(0,0,a)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()
    a=-.2
    while a<0 and not rospy.is_shutdown():
        a+=.1/20
        q=quaternion_from_euler(0,0,a)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()

def stretch(rate,pub):
    p=Pose()
    a=0
    while a<math.pi*2 and not rospy.is_shutdown():
        a+=0.025
        p.position.x=0.05*math.cos(a)
        p.position.y=0.05*math.sin(a)
        q = quaternion_from_euler(0,0,0)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()

def up_down(rate,pub):
    p=Pose()
    h=0
    while h<0.05 and not rospy.is_shutdown():
        h+=0.002
        p.position.z=h
        q = quaternion_from_euler(0,0,0)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()
    h=0.05
    while h>-0.05 and not rospy.is_shutdown():
        h-=0.002
        p.position.z=h
        q = quaternion_from_euler(0,0,0)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()
    h=-0.05
    while h<0 and not rospy.is_shutdown():
        h+=0.002
        p.position.z=h
        q = quaternion_from_euler(0,0,0)
        p.orientation.x=q[0]
        p.orientation.y=q[1]
        p.orientation.z=q[2]
        p.orientation.w=q[3]
        pub.publish(p)
        rate.sleep()

def talker():
    pub = rospy.Publisher('pose_cmd', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(40)
    p=Pose()
    a=0.0        

    while not rospy.is_shutdown(): 
        for i in range(1):
            up_down(rate,pub)

        for i in range(1):
            rotate(rate,pub)

        for i in range(1):
            stretch(rate,pub)
        
        a=0
        while a<math.pi*4 and not rospy.is_shutdown():
            a+=0.05
            q = quaternion_from_euler(1.5*math.cos(a), 1.5*math.sin(a),0)
            p.orientation.x=q[0]
            p.orientation.y=q[1]
            p.orientation.z=q[2]
            p.orientation.w=q[3]
            pub.publish(p)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass