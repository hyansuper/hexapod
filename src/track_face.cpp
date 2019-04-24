#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

int too_close_area;
double lean_back;
int faceid;
ros::Publisher vel_pub, pose_pub;

void track(std::vector<int>::const_iterator& it, int index){
	// ROS_INFO("tracking face: %d", id);
	int center_x = it[index+2]+it[index+4]/2;
	int center_y = it[index+3]+it[index+5]/2;
	int area = it[index+4]*it[index+5];

	int off_x = center_x-it[index+3]/2;
	int off_y = center_y-it[index+5]/2;

	tf2::Quaternion q_tf;
	geometry_msgs::Pose p;
	q_tf.setRPY(0, off_y*0.01, off_x*0.01);

	if(area > too_close_area){

	}

	q_tf.normalize();
	p.orientation = tf2::toMsg(q_tf);
	pose_pub.publish(p);
}

void sub_callback(const std_msgs::Int32MultiArray::ConstPtr& myMsg)
{
	std::vector<int>::const_iterator it = myMsg->data.begin();
	// ROS_INFO("### Face_Data: fps[%d] numFaces[%d] ImgX[%d] ImgY[%d] ###########\n", it[0], it[1], it[2], it[3]);
	if(it[1]==0) return;

	int counter = 4;	
	for (int i = 0; i < it[1]; i++) {
		if(it[counter]==faceid){
			track(it,counter);
			return;
		}
		// ROS_INFO("Face: %d # detected for [%d] # X[%d] Y[%d]\n", it[counter], it[counter+1], it[counter+2], it[counter+3]);
		counter += 6;
	}

	faceid=it[4];
	track(it,4);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "track_face");    
    ros::NodeHandle nh, private_nh("~");

    private_nh.param("too_close_area",too_close_area,40000);    
    private_nh.param("lean_back",lean_back,0.05);    

    ros::Subscriber sub = nh.subscribe("faceCoord", 10, sub_callback);
    pose_pub = nh.advertise<geometry_msgs::Pose>("pose_cmd", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmd", 10);

    ros::spin();
    return 0;
}
