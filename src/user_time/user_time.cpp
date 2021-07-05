#include <time.h>
#include "user_time.h"

namespace user_time{

uint64_t get_micros()
{
    uint64_t t = 0;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	t = 1.0e6*(ts.tv_sec + (ts.tv_nsec*1.0e-9));
    return (t & 0xFFFFFFFF);
}


 uint64_t get_millis()
    {
        uint64_t t = 0;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t = 1.0e3*(ts.tv_sec + (ts.tv_nsec*1.0e-9));
    
        return (t & 0xFFFFFFFF);
    }

double NowInSeconds()
{
    return get_millis()*0.001;
}

}
