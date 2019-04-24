#ifndef __HEXAPOD_TRAJECTORY__
#define __HEXAPOD_TRAJECTORY__

namespace hexapod{

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

}

#endif
