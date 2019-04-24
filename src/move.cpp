#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>
#include "hexapod/RiseAction.h"
#include "hexapod/GetGait.h"
#include "hexapod/SetGait.h"
#include "hexapod/gait.h"
#include "hexapod/gait_manager.h"
#include "hexapod/default_gaits.h"

using namespace hexapod;
using hexapod::SetGait;
using hexapod::GetGait;

ros::Time vel_cmd_time;
bool new_pose_cmd = false;
tf::Transform vel_delta;
double pose_translation_limit,pose_rotation_limit,vel_linear_limit,vel_angular_limit;
double update_rate;
double delta_z,z_upper_limit,z_lower_limit;


//Normalize to [-180,180)
double normalize_angle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

void preempt_cb(actionlib::SimpleActionServer<hexapod::RiseAction>& as)
{
    as.setPreempted();
}

void goal_cb(actionlib::SimpleActionServer<hexapod::RiseAction>& as,GaitManager& gm)
{
    double goal_z=as.acceptNewGoal()->z;
    if(goal_z>z_upper_limit || goal_z<z_lower_limit){
        as.setAborted();
        return;
    }
    ros::Rate r(update_rate);
    gm.pose_tf.setIdentity();
    gm.ik_and_pub();
    ros::Duration(1).sleep();
    double center_z = gm.center_tf.getOrigin().z();
    while(center_z != goal_z) {
        if(as.isPreemptRequested() || !ros::ok()){
            as.setPreempted();
            return;
        }
        center_z += std::min(delta_z,std::max(-delta_z,goal_z-center_z));
        gm.center_tf.getOrigin().setZ(center_z);
        gm.ik_and_pub();
        r.sleep();
    }
    as.setSucceeded();
}

void pose_cmd_sub_cb(const geometry_msgs::PoseConstPtr& msg,GaitManager& gm)
{
    tf::Vector3 v(msg->position.x,msg->position.y,msg->position.z);
    double len = v.length();
    if(len > pose_translation_limit) {
        v*=pose_translation_limit/len;
    }
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation,q);  
    double angle = q.getAngle();
    angle=normalize_angle(angle);
    angle=std::max(-pose_rotation_limit,std::min(pose_rotation_limit,angle));
    q.setRotation(q.getAxis(),angle);
    gm.pose_tf.setOrigin(v);
    gm.pose_tf.setRotation(q);
    new_pose_cmd = true;
}

void vel_cmd_sub_cb(const geometry_msgs::TwistConstPtr& msg)
{
    tf::vector3MsgToTF(msg->linear,vel_delta.getOrigin());
    vel_delta.getOrigin().setZ(0.0);
    double len = vel_delta.getOrigin().length();
    if(len > vel_linear_limit) {
        vel_delta.getOrigin()*=vel_linear_limit/len;
    }
    vel_delta.getOrigin()/=update_rate;
    double a=std::max(-vel_angular_limit,std::min(vel_angular_limit,msg->angular.z));
    vel_delta.getBasis().setRPY(0,0,a/update_rate);
    vel_cmd_time=ros::Time::now();
}

bool set_gait_cb(SetGait::Request &req,SetGait::Response &res,GaitManager& gm)
{
    if(gm.gait->state!=Gait::STANDING){
        res.success=false;
        std::stringstream ss;
        ss<<"cann't set new gait when hexpod is moving";
        res.extra_info=ss.str();
    }else if(gm.gaits.find(req.name)==gm.gaits.end()){
        res.success=false;
        std::stringstream ss;
        ss<<"unknown gait named "<<req.name;
        res.extra_info=ss.str();
    }else{
        res.success=true;
        gm.gait=gm.gaits[req.name];
    }
    return true;
}

bool get_gait_cb(GetGait::Request &req,GetGait::Response &res,GaitManager& gm)
{    
    res.name=gm.gait->getName();
    return true;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh,private_nh("~");
    
    double vel_cmd_duration,gait_period,leg_tip_trajectory_height,leg_tip_default_y;
    private_nh.param("update_rate",update_rate,10.0);
    private_nh.param("vel_cmd_duration",vel_cmd_duration,1.0);
    private_nh.param("pose_rotation_limit",pose_rotation_limit,M_PI/6);
    private_nh.param("pose_translation_limit",pose_translation_limit,0.05);
    private_nh.param("vel_linear_limit",vel_linear_limit,0.05);
    private_nh.param("vel_angular_limit",vel_angular_limit,M_PI/6);
    private_nh.param("gait_period",gait_period,2.0);
    private_nh.param("leg_tip_trajectory_height",leg_tip_trajectory_height,0.02);
    private_nh.param("leg_tip_default_y",leg_tip_default_y,0.05);

    private_nh.param("delta_z",delta_z,0.005);
    private_nh.param("z_upper_limit",z_upper_limit,0.10);
    private_nh.param("z_lower_limit",z_lower_limit,0.0);

    ros::Rate r(update_rate);
    ros::Duration vel_cmd_dur(vel_cmd_duration);

    urdf::Model* model = new urdf::Model();
    if(!model->initParam("robot_description")) {
        ROS_ERROR("could not find robot model, legs uninitialized.");
        ros::shutdown();
        return -1;
    }
    GaitManager gm(nh,*model,leg_tip_default_y);
    gm.gaits.insert(std::pair<std::string,Gait*>(std::string("tripod"),new TripodGait(gait_period*update_rate,leg_tip_trajectory_height)));
    gm.gaits.insert(std::pair<std::string,Gait*>(std::string("wave"),new WaveGait(gait_period*update_rate,leg_tip_trajectory_height)));
    gm.gaits.insert(std::pair<std::string,Gait*>(std::string("ripple"),new RippleGait(gait_period*update_rate,leg_tip_trajectory_height)));

    gm.gait = gm.gaits["ripple"];
    delete model;

    ros::Subscriber pose_cmd_sub = nh.subscribe<geometry_msgs::Pose>("pose_cmd", 1, boost::bind(pose_cmd_sub_cb,_1,boost::ref(gm)));
    ros::Subscriber vel_cmd_sub = nh.subscribe<geometry_msgs::Twist>("vel_cmd", 1, vel_cmd_sub_cb);  

    ros::ServiceServer set_gait_srv = nh.advertiseService<SetGait::Request,SetGait::Response>("set_gait", boost::bind(set_gait_cb,_1,_2,boost::ref(gm)));
    ros::ServiceServer get_gait_srv = nh.advertiseService<GetGait::Request,GetGait::Response>("get_gait", boost::bind(get_gait_cb,_1,_2,boost::ref(gm)));

    actionlib::SimpleActionServer<hexapod::RiseAction> as(nh,"rise",false);
    as.registerGoalCallback(boost::bind(goal_cb,boost::ref(as),boost::ref(gm)));
    as.registerPreemptCallback(boost::bind(preempt_cb,boost::ref(as)));
    as.start();

    while(ros::ok()) {
        ros::Time now = ros::Time::now(); 
        bool expired = (now - vel_cmd_time) > vel_cmd_dur;
        if(expired && gm.gait->state==Gait::STANDING && !new_pose_cmd) {
           gm.pub(now);
        }else{

            if(expired){
                gm.gait->update(gm.legs,gm.center_tf);
                
            }else{
                gm.gait->update(gm.legs,gm.center_tf,vel_delta);
                gm.center_tf*=vel_delta;
            }

            new_pose_cmd = false;
            gm.ik_and_pub(now);
        }
        ros::spinOnce();
        r.sleep();        
    }
    return 0;
}