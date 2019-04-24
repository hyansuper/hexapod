#ifndef __HEXAPOD_LEG_IK__
#define __HEXAPOD_LEG_IK__

#include <math.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include "hexapod/leg.h"

namespace hexapod{	

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

}

#endif