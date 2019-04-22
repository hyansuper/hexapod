#ifndef __HEXAPOD_GAIT__
#define __HEXAPOD_GAIT__

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

namespace hexapod{

double safe_acos(double a){
    if(a>1)a=1;
    else if(a<-1)a=1;
    return acos(a);
}

class Trajectory{
    public:
    double v_axis;
    int phase;
    std::vector<double>h,v;
    Trajectory(const Trajectory &t):phase(t.phase),v_axis(t.v_axis),h(t.h),v(t.v){}
    Trajectory(double _v_axis,int _phase):phase(_phase),v_axis(_v_axis){
        double delta=M_PI/_phase;
        for(int i=0;i<_phase;i++) {
            double theta=i*delta;
            h.push_back(-cos(theta));
            v.push_back(v_axis*sin(theta));
        }
        h.push_back(1);
        v.push_back(0);
    }
};

class Leg{
    public:
    enum State {
        SUPPORT,TRANSFER
    };
    Leg::State state;
    int traj_index;// index of progress in transfer trajectory
    tf::Vector3 tip_current;// relative to world frame
    tf::Vector3 tip_default;// relative to body center, only x ans y components are used.
    tf::Vector3 traj_ori,traj_h;
    tf::Transform l0_to_center_tf;
    std::string name,j01,j12,j23,j34,l0;    
    Leg(std::string n,double x=0,double y=0):
                        traj_index(0),
                        tip_default(x,y,0),
                        state(SUPPORT),
                        name(n),
                        l0(name+"_L0"),
                        j01(name+"_J01"),j12(name+"_J12"),j23(name+"_J23"),j34(name+"_J34"){}
    
};

class LegIk {
    double link1,link2,link3,
       link2_sqr,link3_sqr,link2_mul_2,
       link2_sqr_add_link3_sqr,link2_sqr_sub_link3_sqr,link2_mul_link3_mul_2;
    
    public:
    sensor_msgs::JointState js;
    
    void init(const urdf::Model& model,std::vector<Leg>& legs) {
        Leg& leg = legs[0];
        link1 = model.getJoint(leg.j12)->parent_to_joint_origin_transform.position.y;
        link2 = model.getJoint(leg.j23)->parent_to_joint_origin_transform.position.y;
        link3 = model.getJoint(leg.j34)->parent_to_joint_origin_transform.position.y;
        link2_sqr=pow(link2,2);
        link3_sqr=pow(link3,2);
        link2_mul_link3_mul_2=2.0*link2*link3;
        link2_mul_2=2.0*link2;
        link2_sqr_add_link3_sqr=link2_sqr+link3_sqr;
        link2_sqr_sub_link3_sqr=link2_sqr-link3_sqr;     

        for(std::vector<hexapod::Leg>::iterator it=legs.begin();it!=legs.end();++it){
            js.name.push_back(it->j01);
            js.name.push_back(it->j12);
            js.name.push_back(it->j23);
            int i=0;
            while(i++ < 3)
                js.position.push_back(0.0);
        }
    }

    void do_ik(const tf::Transform& center_tf,const tf::Transform& pose_tf,std::vector<Leg>& legs) {
        js.position.clear();
        tf::Transform center_tf_inv = center_tf.inverse();
        tf::Transform pose_tf_inv = pose_tf.inverse();
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++) {
            tf::Vector3 v = it->l0_to_center_tf(pose_tf_inv(center_tf_inv(it->tip_current)));
            double t1 = pow(v.x(),2)+pow(v.y(),2);
            double t3 = sqrt(t1)-link1;
            double t2 = pow(t3,2)+pow(v.z(),2);
            js.position.push_back(-atan2(v.x(),v.y()));
            js.position.push_back(acos((link2_sqr_sub_link3_sqr+t2)/link2_mul_2/sqrt(t2))-atan2(-v.z(),t3));
            js.position.push_back(-M_PI+acos((link2_sqr_add_link3_sqr-t2)/link2_mul_link3_mul_2));
        }
    }
};

class Gait {
    protected:
    const std::string name;
    public:    
    enum State {
        STANDING,// all 6 legs on the ground
        WALKING,// the base is moving, some legs all in the air
        STOPPING// base is not moving, but some legs are still returing to its default position
    };    
    Gait::State state;
    Gait(std::string n):name(n),state(STANDING){}
    std::string getName(){return name;}
    virtual void update(std::vector<Leg>& legs,const tf::Transform& base)=0;
    virtual void update(std::vector<Leg>& legs,const tf::Transform& base,const tf::Transform& d_vel)=0;
};

class GaitCommon: public Gait {

    protected:
    int index;
    Trajectory traj;
    const int phase;

    GaitCommon(std::string n,int _phase,double height,int traj_phase):Gait(n),index(0),phase(_phase),traj(height,traj_phase){}

    tf::Transform base_planed(const tf::Transform& base,const tf::Transform& vel_delta){
        double r,p,y;
        vel_delta.getBasis().getRPY(r,p,y);
        int t_pl = phase/4*3;
        tf::Vector3 v_pl = vel_delta.getOrigin()*t_pl;
        tf::Vector3 axis(0,0,1);
        if(y!=0 && !v_pl.isZero()){
            double half_a_pl=y*t_pl/2;
            v_pl=v_pl.normalize().rotate(axis,half_a_pl)*(vel_delta.getOrigin().length()/y*sin(half_a_pl));
        }
        return base*tf::Transform(tf::Quaternion(axis,y*t_pl),v_pl);
    }

    void plan_foothold(Leg& leg,const tf::Transform& base_pl){
        tf::Vector3 tip_pl = base_pl*leg.tip_default;
        tip_pl.setZ(0);
        leg.traj_h = (tip_pl - leg.tip_current)/2;
        leg.traj_ori = (tip_pl + leg.tip_current)/2;
    }

    void update_leg(Leg& leg){
        if(leg.state==Leg::TRANSFER){ 
            leg.traj_index++;    
            leg.tip_current = leg.traj_ori + leg.traj_h*traj.h[leg.traj_index];  
            leg.tip_current.setZ(traj.v[leg.traj_index]); 
            if(leg.traj_index==traj.phase){    
                leg.traj_index=0;
                leg.state=Leg::SUPPORT;
            }  
        }
    }
};

class WaveGait: public GaitCommon{
    public:
        //phase must be multiple of 6
    WaveGait(int _phase,double height):GaitCommon("wave",_phase,height,_phase/6){}
    void update(std::vector<Leg>& legs,const tf::Transform& base){
        if(state==WALKING){
            state=STOPPING;
        }
        if(index%(traj.phase)==0){
            bool legs_reset = true;
            for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
                tf::Vector3 v=base*it->tip_default;
                v.setZ(0);
                if((it->tip_current - v).length2() > 0.01*0.01){
                    legs_reset = false;
                    break;
                }
            }
            if(legs_reset){
                index=0;
                state=STANDING;
                return;
            }else{
                plan_foothold(legs[index/traj.phase],base);
                legs[index/traj.phase].state=Leg::TRANSFER;
            }

        }  
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
            update_leg(*it);
        }
        index++;
        if(index==phase){
            index=0;
        }  
    }

    void update(std::vector<Leg>& legs,const tf::Transform& base,const tf::Transform& d_vel){
        if(state!=WALKING)
            state=WALKING;     
        if(index%traj.phase == 0){
            std::vector<Leg>::iterator it=legs.begin()+ (index/traj.phase);
            tf::Transform base_pl = base_planed(base,d_vel);
            plan_foothold(*it,base_pl);
            it->state=Leg::TRANSFER;
        }
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){         
            update_leg(*it);
        }    
        index++;
        if(index==phase){
            index=0;
        }
    }
};
class RippleGait: public GaitCommon{
    public:
        //phase must be multiple of 6
    RippleGait(int _phase,double height):GaitCommon("ripple",_phase,height,_phase/3){}
    void update(std::vector<Leg>& legs,const tf::Transform& base){
        if(state==WALKING){
            state=STOPPING;
        }
        if(index%(traj.phase/2)==0){
            int i=index/(traj.phase/2);
            switch(i){
                case 1:i=5;break;
                case 3:break;
                case 5:i=4;break;
                default:i/=2;
            }
            std::vector<Leg>::iterator it=legs.begin()+ i;
            tf::Vector3 v=base*it->tip_default;
            v.setZ(0);
            if((it->tip_current - v).length2() > 0.01*0.01){                
                plan_foothold(*it,base);
                it->state=Leg::TRANSFER;    
            }
            bool legs_reset = true;
            for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
                tf::Vector3 v=base*it->tip_default;
                v.setZ(0);
                if((it->tip_current - v).length2() > 0.01*0.01){
                    legs_reset = false;
                    break;
                }
            }
            if(legs_reset){
                index=0;
                state=STANDING;
            }
        }  
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
            update_leg(*it);
        }
        index++;
        if(index==phase){
            index=0;
        }
    }
    void update(std::vector<Leg>& legs,const tf::Transform& base,const tf::Transform& d_vel){
        if(state!=WALKING)
            state=WALKING;     
        if(index%(traj.phase/2)==0){
            int i=index/(traj.phase/2);
            switch(i){
                case 1:i=5;break;
                case 3:break;
                case 5:i=4;break;
                default:i/=2;
            }
            std::vector<Leg>::iterator it=legs.begin()+ i;
            tf::Transform base_pl = base_planed(base,d_vel);            
            plan_foothold(*it,base_pl);
            it->state=Leg::TRANSFER;
        }
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){         
            update_leg(*it);
        }    
        index++;
        if(index==phase){
            index=0;
        }  
    }
};
class TripodGait: public GaitCommon{
    
    public:
        //phase must be multiple of 2
    TripodGait(int _phase,double height):GaitCommon("tripod",_phase,height,_phase/2){}
    
    void update(std::vector<Leg>& legs,const tf::Transform& base){
        if(state==WALKING){
            state=STOPPING;
        }
        if(index==0 || index==phase/2){
            bool legs_reset = true;
            for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
                tf::Vector3 v=base*it->tip_default;
                v.setZ(0);
                if((it->tip_current - v).length2() > 0.01*0.01){
                    legs_reset = false;
                    break;
                }
            }
            if(legs_reset){
                index=0;
                state=STANDING;
                return;
            }else{
                std::vector<Leg>::iterator it=legs.begin()+ (index==0 ? 0 : 1);
                int i=0;
                while(i<3){
                    plan_foothold(*it,base);
                    it->state=Leg::TRANSFER;
                    it+=2;
                    i++;
                }
            }

        }  
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){
            update_leg(*it);
        }
        index++;
        if(index==phase){
            index=0;
        }  
    }

    void update(std::vector<Leg>& legs,const tf::Transform& base,const tf::Transform& d_vel){
        if(state!=WALKING)
            state=WALKING;     
        if(index==0 || index==phase/2){
            std::vector<Leg>::iterator it=legs.begin()+ (index==0 ? 0 : 1);
            tf::Transform base_pl = base_planed(base,d_vel);
            int i=0;
            while(i<3){
                plan_foothold(*it,base_pl);
                it->state=Leg::TRANSFER;
                it+=2;
                i++;
            }
        }
        for(std::vector<Leg>::iterator it=legs.begin();it!=legs.end();it++){         
            update_leg(*it);
        }    
        index++;
        if(index==phase){
            index=0;
        }  
    }
};

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