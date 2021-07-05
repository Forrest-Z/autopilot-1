#pragma once

//#define MCU_V1				//大机箱版本
//#define MCU_V2					// metal 金属主控 
#define MCU_V3				// plastic 1号船


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <math.h>

#ifndef WINNT
#include <unistd.h>
#include <termios.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <sys/time.h>
#include "threadexp.h" 
#else
#include "../../win_prj/win_prj/threadexp_win.h"
#include "../../win_prj/win_prj/win_drive.h"
#include "../../win_prj/win_prj/getopt.h"
#endif


#include "watch_dog.h"
#include "USV_const.h"
#include "can_api.h"
#include "lan_api.h"
#include "uart_api.h"
#include "monitor.h"
#include "read_usv_cfg.h"
#include "read_usv_state.h"
#include "input_monitor_data.h"
#include "Algorithm_path.h"
#include "bd.h"
#include "bd_comm.h"
//#include "RingQueLib.h"
#include "TaskQue.h"
#include "SysMsgPub.h"
#include "SysLog.h"
#include "MsgQue.h"
#include "AisMsgQue.h"
#include "BradioUdpThread.h"
#include "ins_recorder.h"



#include "GlobalFunction.h"
#include "can_deal_IDU.h"
#include "can_deal_IOP.h"
#include "can_deal_IHC.h"

#include "sailWaypoint.h"

#include "uart_deal_ins.h"								//惯导接收模块
#include "uart_irtk.h"									//iRTK收发模块
#include "uart_deal_BD.h"
#include "uart_deal_DRadio.h"
#include "lan_deal_BRadio.h"
#include "can_deal_main.h"
#include "Component.h"

#include "control_assistance.h"					//辅助驾驶
#include "pid_control.h"
#include "comm_main.h"
#include "control_jetCtrlFunc.h"
#include "control_main.h"
#include "control_cmd.h"
#include "control_BD.h"
#include "control_operation.h"
#include "control_auto.h"						//自动巡航模式
#include "control_stanby.h"						//镇定模式
#include "control_manual.h"						//手动模式
#include "control_semiAuto.h"					//半自动模式
#include "ctrl_semiAuto_speedConstant.h"		//定速巡航
#include "ctrl_semiAuto_headingConstant.h"		//定向巡航
#include "ctrl_semiAuto_berth.h"				//泊岸模式
#include "control_signalSt.h"


#include "ApfMethod.h"
#include "zmqGetObstacles.h"
#include "samplingComm.h"
//#define debug_print									//调试打印
#include "warnProcess.h"
#include "lan_deal_CtrlCenter.h"

#include "docking_main.h"				//usv-船坞通信
#include "docking_pid_loop.h"
#include "docking_visual_guid.h"
#include "docking_sumlink_tuning.h"
#include "docking_usv_control.h"

#include "auto_return.h"
#include "adrc.h"
extern int poweron_init;

float wrap_180_cd(float angle);
float wrap_360_cd(float angle);
float wrap_180(float angle, float unit_mod);
float wrap_360(float angle, float unit_mod);

//#define        deg2rad(x)                       ((3.141592653589793f)*(x)/180.0)
//#define        rad2deg(x)                       (180.0f*(x)/(3.141592653589793f))

#define        constrain_value(amt,low,high)    ((amt) < (low) ? (low):((amt) >(high) ? (high) :(amt)))
#define        MAX(A,B)                         ((A) > (B) ?(A) :(B))
#define        MIN(A,B)                         ((A) < (B) ?(A) :(B))

#define        FLT_PI                            (3.141592653589793f)
#define        FLT_2PI                           (float)((3.141592653589793f) * 2)

#define        DEG_TO_RAD                        (FLT_PI / 180.0f)
#define        RAD_TO_DEG                        (180.0f / FLT_PI)
#define        deg2rad(x)                       ((3.141592653589793f)*(x)/180.0)
#define        rad2deg(x)                       (180.0f*(x)/(3.141592653589793f))

#define        FLT_EPSILON                      (0.000001f)
#define        kn2ms(x)                         (x*0.5144f)
#define        ms2kn(x)                         (x*1.9438f)
#define        sq(x)                            ((x)*(x))
#define        is_zero(x)                       (((x) < FLT_EPSILON) && ((x) > (-1.0*FLT_EPSILON)))
#define        is_equal(v_1,v_2)                (FABS(v_1 - v_2) <= FLT_EPSILON)
#define        is_positive(x)                   ((x) >= FLT_EPSILON)
#define        is_negative(x)                   ((x) <= (-1.0 * FLT_EPSILON))




// #endif /* USV_INCLUDE_H */
