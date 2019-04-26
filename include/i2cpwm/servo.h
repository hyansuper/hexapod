#ifndef __HEXAPOD_SERVO__
#define __HEXAPOD_SERVO__

#include "hexapod/ServoInfo.h"
#include "i2cpwm/i2cpwm_manager.h"

using hexapod::ServoInfo;
using i2cpwm::I2cpwmManager;

namespace i2cpwm{

class Servo{
    I2cpwmManager *manager;
    ServoInfo info;
    double k;
    double angle;
    //bool relaxed;
    
    void calc_k(){
        k=(info.angle_max_pwm-info.angle_min_pwm)/(info.angle_max-info.angle_min);
    }
    
public:
    Servo(){}
    Servo(I2cpwmManager* m,ServoInfo i):manager(m),info(i){
        calc_k();
    }

    bool set_angle_check(double a){
        return (a>=info.angle_min && a<=info.angle_max);
    }

    void set_angle(double a){
        if(angle!=a && set_angle_check(a)){
            angle=a;
            manager->set_active_board(info.board);
            manager->set_pwm_interval(info.channel,0,k*(angle-info.angle_min)+info.angle_min_pwm);
            //relaxed = false;
        }
    }

    void relax(){
        manager->set_active_board(info.board);
        manager->set_pwm_interval(info.channel, 0, 0);
        //relaxed = true;
    }

    void set_info(ServoInfo i){
        info=i;
        calc_k();
        if(manager!=NULL){
            manager->set_active_board(info.board);
            manager->set_pwm_interval(info.channel,0,k*(angle-info.angle_min)+info.angle_min_pwm);
        }
    }
    
    double get_angle(){return angle;}
    //bool is_relaxed(){return relaxed;}
    ServoInfo get_info(){return info;}
};

}

#endif
