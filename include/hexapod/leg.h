#ifndef __HEXAPOD_LEG__
#define __HEXAPOD_LEG__

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace hexapod{

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

}

#endif