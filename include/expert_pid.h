/*
* expert_pid.h --ר��PID
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2017/10/12 15:09:31��shaoyuping
*/

#ifndef _EXPERT_PID_H
#define _EXPERT_PID_H


typedef struct{
    float SetPoint  ;           //�趨ֵ
    float Actual    ;           //ʵ��ֵ
    float M1        ;           //������1 
    float M2        ;           //������2
    float ep        ;           //������3
    float k1        ;           //����Ŵ�ϵ��  k1>1
    float k2        ;           //��������ϵ��  0<k2<1
    float kp        ;           //����ϵ��
    float ki        ;           //����ϵ��
    float kd        ;           //΢��ϵ��
    float em        ;           //ǰһ��ֵ
    float err       ;           //��ǰ���
    float errLast   ;           //ǰһ�����
    float d_err     ;           //��ǰ���仯��
    float d_errLast ;           //ǰһ�����仯��    
    
}Spd_ExPID;











#endif