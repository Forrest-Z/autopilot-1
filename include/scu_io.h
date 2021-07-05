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
/* ���صĿ��Ʒ���״̬ */
enum actutor_mixer_type
{
    MIXER_DOUBLE_JET = 0,
    MIXER_DIFF_SPEED = 1,
    
    MIXER_UNKNOW
};

/*���ص�ģʽ״̬ */
enum scu_ctl_mode_type
{
    CTL_MUAL = 0,     // ֱ�ӵ紫����
    CTL_ATT,          // ���ٶ���
    CTL_POS,          // λ�ÿ���
    CTL_TAKE_OFF,     // �Զ��뿪�ۿ�
    CTL_LAND,         // �Զ�����ۿ�
    CTL_RETURN_HOME,  // �Զ�����
    CTL_CIRCLE,       // �Զ���Ȧ
    CTL_AUTO_PILOT,   // �Զ�����
    CTL_FOLLOW_ME,    // �Զ�����

    CTL_MODE_UNKNOW
};

struct scu_io_send_msg_t
{
    uint8 _ems;          /* ��ͣ��1����λ��0 */
    uint8 _armed;        /* Ĭ�ϣ�0��������1��ֹͣ��2*/
    uint8 _sw_group;     /* �̵���/���ؿ��� */
    uint8 _mixer_class;  /* 0: ������1������ */
    uint8 _ctl_mode;     /* ����ģʽ */

    int16 _forward;      /* ��*/
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