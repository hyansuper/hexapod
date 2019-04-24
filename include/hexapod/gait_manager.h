#ifndef __HEXAPOD_GAIT_MANAGER__
#define __HEXAPOD_GAIT_MANAGER__

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>
#include "hexapod/gait.h"
#include "hexapod/leg.h"
#include "hexapod/leg_ik.h"

namespace hexapod{

class GaitManager {

    public:
    std::map<std::string,Gait*> gaits;
    Gait* gait;
    std::vector<Leg> legs;
    LegIk ik;
    ros::Publisher ik_js_pub;
    tf::TransformBroadcaster br;
    tf::StampedTransform pose_tf,center_tf;
    GaitManager (ros::NodeHandle& nh,
                urdf::Model& model,
                const double leg_tip_default_y):
                
                ik_js_pub(nh.advertise<sensor_msgs::JointState>("ik_joint_states",1))

    {

        //init legs:
        legs.push_back(Leg("leftFrontLeg"));
        legs.push_back(Leg("leftMiddleLeg"));
        legs.push_back(Leg("leftRareLeg"));  
        legs.push_back(Leg("rightFrontLeg"));        
        legs.push_back(Leg("rightMiddleLeg"));              
        legs.push_back(Leg("rightRareLeg"));

        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
            urdf::Pose pose = model.getJoint("base_to_" + it->l0)->parent_to_joint_origin_transform;
            double r,p,y;
            pose.rotation.getRPY(r,p,y);
            it->tip_default.setX(pose.position.x+leg_tip_default_y*sin(-y));
            it->tip_default.setY(pose.position.y+leg_tip_default_y*cos(-y));
            it->tip_current = it->tip_default;
            it->l0_to_center_tf.setOrigin(tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
            it->l0_to_center_tf.setRotation(tf::Quaternion(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w));
            it->l0_to_center_tf = it->l0_to_center_tf.inverse();
        }

        // init ik:
        ik.init(model,legs);

        pose_tf.frame_id_="base_center";
        pose_tf.child_frame_id_="base_link";
        pose_tf.setIdentity();
        center_tf.frame_id_="world";
        center_tf.child_frame_id_="base_center";
        center_tf.setIdentity();
    }

    void ik_and_pub(ros::Time t=ros::Time::now()){
        ik.do_ik(center_tf,pose_tf,legs);
        pose_tf.stamp_=t;
        center_tf.stamp_=t;

        br.sendTransform(center_tf);
        br.sendTransform(pose_tf);
        ik_js_pub.publish(ik.js);
    }

    void pub(ros::Time t=ros::Time::now()){
        pose_tf.stamp_=t;
        center_tf.stamp_=t;

        br.sendTransform(center_tf);
        br.sendTransform(pose_tf);
    }

    ~GaitManager(){
        for(std::map<std::string,Gait*>::iterator it=gaits.begin();it!=gaits.end();it++){
            delete it->second;
        }
    }
};

}

#endif