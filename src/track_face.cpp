#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

int too_close_area;
double lean_back;
int faceid;
ros::Publisher vel_pub, pose_pub;
int center_x, center_y, off_x, off_y, area;
bool tracking = false;
double rotate_factor;
int tracking_threshold_x, tracking_threshold_y;

void track(std::vector<int>::const_iterator& it, int index){
	// ROS_INFO("tracking face: %d", id);
	center_x = it[index+2]+it[index+4]/2;
	center_y = it[index+3]+it[index+5]/2;
	area = it[index+4]*it[index+5];

	off_x = center_x-it[2]/2;
	off_y = center_y-it[3]/2;

	if(abs(off_x) > tracking_threshold_x || abs(off_y) > tracking_threshold_y || area > too_close_area){
		tracking = true;
	}
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
    private_nh.param("rotate_factor",rotate_factor,0.0001);
    private_nh.param("tracking_threshold_x",tracking_threshold_x,100);
    private_nh.param("tracking_threshold_y",tracking_threshold_y,100);

    ros::Subscriber sub = nh.subscribe("faceCoord", 10, sub_callback);
    pose_pub = nh.advertise<geometry_msgs::Pose>("pose_cmd", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmd", 10);

    ros::Rate r(10);
    tf::TransformListener listener;

    while(ros::ok()){
	    if(tracking) {
	    	tf::StampedTransform transform;
		    try{
		    	listener.waitForTransform("base_center", "base_link", ros::Time(0), ros::Duration(10.0));		    	
	    		listener.lookupTransform("base_center", "base_link", ros::Time(0), transform);
		    }
		    catch (tf::TransformException &ex) {
		    	ROS_ERROR("%s",ex.what());
		    }
		    tf::Quaternion q;
		    q.setRPY(0, off_y*rotate_factor, off_x*rotate_factor);
		    q = transform.getRotation()*q;	    

			geometry_msgs::Pose pose_msg;
			tf::quaternionTFToMsg(q, pose_msg.orientation);			
			if(area > too_close_area){
				// lean back a little bit
		    }
		    pose_pub.publish(pose_msg);
		    tracking = false;
		}
    	ros::spinOnce();
    	r.sleep();
    }
    return 0;
}
