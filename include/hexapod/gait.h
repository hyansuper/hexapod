#ifndef __HEXAPOD_GAIT__
#define __HEXAPOD_GAIT__

#include <tf/tf.h>
#include <vector>
#include "hexapod/leg.h"

namespace hexapod{

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


}

#endif