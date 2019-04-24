
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "hexapod/ServoInfo.h"
#include "hexapod/ConfigServo.h"
#include "hexapod/SetServo.h"
#include "hexapod/RelaxServo.h"
#include "hexapod/GetServo.h"
#include "hexapod/SaveConfig.h"
#include "yaml/yaml_convert.h"
#include "i2cpwm/i2cpwm_manager.h"
#include "i2cpwm/servo.h"

using hexapod::ServoInfo;
using hexapod::SetServo;
using hexapod::GetServo;
using hexapod::RelaxServo;
using hexapod::ConfigServo;
using hexapod::SaveConfig;
using i2cpwm::Servo;
using i2cpwm::I2cpwmManager;

//global var
std::map<std::string,Servo> servos;
I2cpwmManager i2cpwm_manager;
ros::Publisher set_servo_pub;

// this function is not properly written, coz exceptions are not handled and it always returns true
bool load_config(std::string file)
{
    YAML::Node config=YAML::LoadFile(file);

    // init i2cpwm boards
    i2cpwm_manager.set_number_of_boards(config["number_of_boards"].as<int>());
    i2cpwm_manager.init_boards();    
    i2cpwm_manager.set_pwm_freq(config["pwm_freq"].as<int>());
    
    // load servo info
    YAML::Node infos=config["servos"];
    for(YAML::const_iterator it=infos.begin();it!=infos.end();++it){        
        Servo s(&i2cpwm_manager,it->second.as<ServoInfo>());
        servos.insert(std::pair<std::string,Servo>(it->first.as<std::string>(),s));
    }
    return true;
}

void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(unsigned int i=0;i<msg->name.size();i++)
    {
	    if(servos.find(msg->name[i].c_str())==servos.end())
            continue;
        // if(!
        servos[msg->name[i].c_str()].set_angle(msg->position[i]);//){
            // ROS_ERROR("Joint position of %s out of bound",msg->name[i].c_str());
        // }
    }
}

bool config_servo_cb(ConfigServo::Request &req, ConfigServo::Response &res)
{
    if(servos.find(req.name)==servos.end()){
        res.success=false;
        std::stringstream ss;
        ss<<"no servo found with the name "<<req.name;
        res.extra_info=ss.str();
    }else{
        servos[req.name].set_info(req.info);
        res.success=true;
    }
    return true;
}

bool relax_servo_cb(RelaxServo::Request &req,RelaxServo::Response &res)
{
    if(req.name=="ALL" || req.name=="all"){
        i2cpwm_manager.relax_all();
        res.success=true;
    }else if(servos.find(req.name)==servos.end()){
        res.success=false;
        std::stringstream ss;
        ss<<"no servo found with the name "<<req.name;
        res.extra_info=ss.str();
    }else{
        servos[req.name].relax();
        res.success=true;
    }
    return true;
}

bool get_servo_cb(GetServo::Request &req,GetServo::Response &res)
{
    if(servos.find(req.name)==servos.end()){
        return false;
    }else{
        res.info=servos[req.name].get_info();
        res.angle=servos[req.name].get_angle();
    }
    return true;
}

/** E.g. rosservice call /i2cpwm_controller/set_servo [joint_name] [position]
    Instead of directly calling the Servo::set_angle method, 
    this service pubishes a JointState msg to topic [ns]/test_joint_states
    which is recieved by joint_state_publisher so that robot_state_publisher and rviz will update.
**/
bool set_servo_cb(SetServo::Request &req,SetServo::Response &res)
{
    if(servos.find(req.name)==servos.end()){
        res.success=false;
        std::stringstream ss;
        ss<<"no servo found with the name "<<req.name;
        res.extra_info=ss.str();
    }else{
        if(servos[req.name].set_angle_check(req.angle)){
            // servos[req.name].set_angle(req.angle);
            sensor_msgs::JointState jointstate;
            jointstate.header.stamp=ros::Time::now();
            jointstate.name.push_back(req.name);
            jointstate.position.push_back(req.angle);
            set_servo_pub.publish(jointstate);
            res.success=true;
        }else{
            res.success=false;
            ServoInfo info=servos[req.name].get_info();
            std::stringstream ss;
            ss<<"angle out of range ["<<info.angle_min<<", "<<info.angle_max<<"]";
            res.extra_info=ss.str();
        }
    }
    return true;
}

/** E.g. rosservice call /i2cpwm_controller/set_servo_raw [joint_name] [position]
    Directly set servo angle so that joint_state_publisher is not notified,
    robot_state_publisher or rviz will not update.
    This service should be used when there's no joint_state_publisher node running.
**/
bool set_servo_raw_cb(SetServo::Request &req,SetServo::Response &res)
{
    if(servos.find(req.name)==servos.end()){
        res.success=false;
        std::stringstream ss;
        ss<<"no servo found with the name "<<req.name;
        res.extra_info=ss.str();
    }else{
        if(servos[req.name].set_angle_check(req.angle)){
            servos[req.name].set_angle(req.angle);
            res.success=true;
        }else{
            res.success=false;
            ServoInfo info=servos[req.name].get_info();
            std::stringstream ss;
            ss<<"angle out of range ["<<info.angle_min<<", "<<info.angle_max<<"]";
            res.extra_info=ss.str();
        }
    }
    return true;
}

/**
    E.g. rosservice call /i2cpwm_controller/save_config ~/ros_ws/src/hexapod/yaml/servo_config.yaml
    Note: if relative path is used, file will be saved under ~/.ros by default.
    In most cases you will want to use absolute path like the given example.
**/
bool save_config_cb(SaveConfig::Request &req,SaveConfig::Response &res)
{
    std::ofstream fout(req.path.c_str());
    if(fout.is_open()){
        YAML::Node config;
        config["number_of_boards"]=i2cpwm_manager.get_number_of_boards();
        config["pwm_freq"]=i2cpwm_manager.get_pwm_freq();
        for(std::map<std::string,Servo>::iterator it=servos.begin();it!=servos.end();++it){
            config["servos"][it->first]=it->second.get_info();
        }    
        fout << config;
        fout.close();
        res.success=true;
    }else{
        res.success=false;
        std::stringstream ss;
        ss<<"cannot open file "<<req.path;
        res.extra_info=ss.str();
    }
    return true;
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"servo_deriver");  
    ros::NodeHandle private_nh("~"),nh;

    //open i2c bus    
    std::string i2c_dev_file;
    private_nh.param<std::string>("i2c_dev_file", i2c_dev_file,"/dev/i2c-1");
    if(i2cpwm_manager.open_i2c(i2c_dev_file.c_str())){
        ROS_INFO("I2C bus opened on %s",i2c_dev_file.c_str());        
    }else{
        ROS_ERROR("Failed to open I2C bus %s",i2c_dev_file.c_str());
        ros::shutdown();
        return -1;
    }       

    // load config file
    if(private_nh.hasParam("config_file")){
        std::string file;
        private_nh.getParam("config_file",file);
        if(!load_config(file)){
            ROS_ERROR("Error with file %s",file.c_str());
            i2cpwm_manager.close_i2c();
            ros::shutdown();
            return -1;
        }
    }else{
        ROS_ERROR("Parameter 'config_file' is not set");
        i2cpwm_manager.close_i2c();
        ros::shutdown();
        return -1;
    }

    // subscribtion and services
    ros::Subscriber joint_states_sub=nh.subscribe("joint_states",100,joint_states_cb);
    ros::ServiceServer config_servo_srv=private_nh.advertiseService("config_servo",config_servo_cb);
    ros::ServiceServer save_config_srv=private_nh.advertiseService("save_config",save_config_cb);    
    ros::ServiceServer get_servo_srv=private_nh.advertiseService("get_servo",get_servo_cb);
    ros::ServiceServer set_servo_raw_srv=private_nh.advertiseService("set_servo_raw",set_servo_raw_cb);

    set_servo_pub=nh.advertise<sensor_msgs::JointState>("test_joint_states",1);
    ros::ServiceServer set_servo_srv=private_nh.advertiseService("set_servo",set_servo_cb);
    ros::ServiceServer relax_servo_srv=private_nh.advertiseService("relax_servo",relax_servo_cb);

    ros::spin();
    //the following will not be called when CTRL-C is pressed, I don't know how to close the resource properly, though it won't cause any trouble.
    i2cpwm_manager.close_i2c();
	return 0;
}