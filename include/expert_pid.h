/*
* expert_pid.h --专家PID
* 四方继保(  武汉)  软件有限公司
*
* 历史记录：
*
* V1.00，2017/10/12 15:09:31，shaoyuping
*/

#ifndef _EXPERT_PID_H
#define _EXPERT_PID_H


typedef struct{
    float SetPoint  ;           //设定值
    float Actual    ;           //实际值
    float M1        ;           //误差界限1 
    float M2        ;           //误差界限2
    float ep        ;           //误差界限3
    float k1        ;           //增益放大系数  k1>1
    float k2        ;           //增益抑制系数  0<k2<1
    float kp        ;           //比例系数
    float ki        ;           //积分系数
    float kd        ;           //微分系数
    float em        ;           //前一峰值
    float err       ;           //当前误差
    float errLast   ;           //前一拍误差
    float d_err     ;           //当前误差变化量
    float d_errLast ;           //前一排误差变化量    
    
}Spd_ExPID;











#endif