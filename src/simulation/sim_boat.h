#pragma once
#include <stdint.h>

class SimBoat{
public:
    struct model_parameter{
        /* model: du/dt = ka * in - ka/kv *u + kc*/
        double u_ka;
        double u_kv;
        double u_kc;

        double T;
        double alpha;
        double K;
    };

    struct model_state{
        double u;
        double v;
        double r;
        double fai;
        double groundspeed;
        double gama;
        double latitude;
        double longitude;
    };

    SimBoat(){singleton_ = this;};
    virtual ~SimBoat()=default;

    /* Do not allow copies */
    SimBoat(const SimBoat &other) = delete;
    SimBoat &operator=(const SimBoat&) = delete;

    static SimBoat *get_singleton();

    void init(void);
    void reset(void);
    void update(double throttle,double steering,double ts);

    const model_state& get_sim_state()const{return state_;}

private:
    static SimBoat* singleton_;
    model_parameter param_;
    model_state     state_;
    uint64_t last_update_time_ = 0;
private:
    double ds = 0;
};

namespace AP{
    SimBoat *sim_boat();
};