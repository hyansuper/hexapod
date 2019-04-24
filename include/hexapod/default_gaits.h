#ifndef __HEXAPOD_DEFAULT_GAITS__
#define __HEXAPOD_DEFUALT_GAITS__

#include <vector>
#include <tf/tf.h>
#include "hexapod/gait.h"
#include "hexapod/trajectory.h"
#include "hexapod/leg.h"

namespace hexapod{

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

}

#endif