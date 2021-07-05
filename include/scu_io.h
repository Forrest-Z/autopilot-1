#ifndef __SCU_IO_H
#define __SCU_IO_H


#include "usv_include.h"

#define        constrain_value(amt,low,high)    ((amt) < (low) ? (low):((amt) >(high) ? (high) :(amt)))
#define        MAX(A,B)                         ((A) > (B) ?(A) :(B))
#define        MIN(A,B)                         ((A) < (B) ?(A) :(B))

#define        FLT_PI                            (3.141592653589793f)
#define        FLT_2PI                           (float)((3.141592653589793f) * 2)

#define        DEG_TO_RAD                        (FLT_PI / 180.0f)
#define        RAD_TO_DEG                        (180.0f / FLT_PI)
#define        deg2rad(x)                       ((3.141592653589793f)*(x)/180.0)
#define        rad2deg(x)                       (180.0f*(x)/(3.141592653589793f))
/* 船控的控制分配状态 */
enum actutor_mixer_type
{
    MIXER_DOUBLE_JET = 0,
    MIXER_DIFF_SPEED = 1,
    
    MIXER_UNKNOW
};

/*船控的模式状态 */
enum scu_ctl_mode_type
{
    CTL_MUAL = 0,     // 直接电传操纵
    CTL_ATT,          // 定速定向
    CTL_POS,          // 位置控制
    CTL_TAKE_OFF,     // 自动离开港口
    CTL_LAND,         // 自动进入港口
    CTL_RETURN_HOME,  // 自动返航
    CTL_CIRCLE,       // 自动绕圈
    CTL_AUTO_PILOT,   // 自动航线
    CTL_FOLLOW_ME,    // 自动跟随

    CTL_MODE_UNKNOW
};

struct scu_io_send_msg_t
{
    uint8 _ems;          /* 急停：1，复位：0 */
    uint8 _armed;        /* 默认：0，启动：1，停止：2*/
    uint8 _sw_group;     /* 继电器/开关控制 */
    uint8 _mixer_class;  /* 0: 联动，1：差速 */
    uint8 _ctl_mode;     /* 控制模式 */

    int16 _forward;      /* 向*/
    int16 _yaw;
    int16 _lateral;
};

/* Translation value into [0,2PI] */
inline float wrap_2PI(float radian)
{
    if (radian < 0)
    {
    	radian += FLT_2PI;
    }

    if(radian > FLT_2PI)
    {
    	radian-= FLT_2PI;
    }

    return radian;
}

/* Translation value into [-PI.+PI] */
inline float wrap_PI(float radian)
{
    float res = wrap_2PI(radian);
	if (res > FLT_PI)
	{
		res -= FLT_2PI;
	}
	return res;
}

#endif