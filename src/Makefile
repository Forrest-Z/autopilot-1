LINK_ZMQ = -L ../../cross_compile/lib -lzmq
INC_NANOMSG= -I ../3rdLibs/nanomsg/include/
INC_PROTOBUF= -I ../3rdLibs/protobuf/include
INC_MODBUS= -I ../3rdLibs/modbus/include 
LINK_PROTOBUF= -L ../3rdLibs/protobuf/lib -lprotobuf
LINK_NANOMSG = -L ../3rdLibs/nanomsg/lib -lnanomsg 
LINK_MODBUS = -L ../3rdLibs/modbus/lib -lmodbus  
LINKOPTION =-o 
FLAG= -Wall -O2  -c ${INC_NANOMSG} ${INC_PROTOBUF} ${INC_MODBUS}
OBJS=main.o algorithm.o can_ctrl.o can_api.o uart_ctrl.o uart_api.o lan_api.o \
	lan_ctrl.o threadexp.o sysinfo.o gpio_ctrl.o monitor.o USV_const.o input_monitor_data.o \
	read_usv_cfg.o Algorithm_path.o calculate_path_point.o read_usv_state.o bd_comm.o MsgQue.o \
	SysLog.o AisMsgQue.o BradioUdpThread.o ins_recorder.o\
	can_deal_IDU.o can_deal_IHC.o can_deal_IOP.o can_deal_main.o \
	comm_main.o adrc.o \
	control_auto.o control_cmd.o control_jetCtrlFunc.o control_main.o control_manual.o \
	control_operation.o control_semiAuto.o control_signalSt.o control_stanby.o \
	ctrl_semiAuto_berth.o ctrl_semiAuto_headingConstant.o ctrl_semiAuto_speedConstant.o \
	GlobalFunction.o uart_deal_DRadio.o sailWaypoint.o uart_deal_ins.o pid_control.o taskQue.o \
	lan_deal_Bradio.o watch_dog.o uart_deal_BD.o control_BD.o SysMsgPub.o control_assistance.o \
	ApfMethod.o zmqGetObstacles.o samplingComm.o nanoObstacleSender.o \
	error_code.pb.o geometry.pb.o header.pb.o obstacle_radar.pb.o gnss_un237.pb.o warnProcess.o \
	lan_deal_CtrlCenter.o docking_main.o docking_usv_control.o docking_visual_guid.o docking_pid_loop.o \
 	docking_sumlink_tuning.o auto_return.o uart_irtk.o
OUTPUT = ../../../2.download-package/run_prog/S99USV_ARM
SHAREDLIB =-lm -lpthread -lsocketcan -lrt
$(OUTPUT): $(OBJS) 
	${CXX} $(LINKOPTION) $(OUTPUT) $(OBJS) $(SHAREDLIB) $(LINK_ZMQ) $(LINK_NANOMSG) ${LINK_PROTOBUF} ${LINK_MODBUS}

threadexp.o:threadexp.cpp 
	${CXX} ${FLAG} threadexp.cpp
main.o:main.cpp 
	${CXX} ${FLAG} main.cpp
uart_ctrl.o:uart_ctrl.cpp 
	${CXX} ${FLAG} uart_ctrl.cpp
uart_api.o:uart_api.cpp 
	${CXX} ${FLAG} uart_api.cpp
lan_api.o:lan_api.cpp 
	${CXX} ${FLAG} lan_api.cpp
lan_ctrl.o:lan_ctrl.cpp 
	${CXX} ${FLAG} lan_ctrl.cpp
can_api.o:can_api.cpp 
	${CXX} ${FLAG} can_api.cpp
can_ctrl.o:can_ctrl.cpp 
	${CXX} ${FLAG} can_ctrl.cpp
algorithm.o:algorithm.cpp 
	${CXX} ${FLAG} algorithm.cpp
sysinfo.o:sysinfo.cpp 
	${CXX} ${FLAG} sysinfo.cpp
gpio_ctrl.o:gpio_ctrl.cpp 
	${CXX} ${FLAG} gpio_ctrl.cpp
monitor.o:monitor.cpp 
	${CXX} ${FLAG} monitor.cpp
USV_const.o:USV_const.cpp 
	${CXX} ${FLAG} USV_const.cpp
input_monitor_data.o:input_monitor_data.cpp 
	${CXX} ${FLAG} input_monitor_data.cpp	
read_usv_cfg.o:read_usv_cfg.cpp 
	${CXX} ${FLAG} read_usv_cfg.cpp		
Algorithm_path.o:Algorithm_path.cpp
	${CXX} ${FLAG} Algorithm_path.cpp
calculate_path_point.o:calculate_path_point.cpp
	${CXX} ${FLAG} calculate_path_point.cpp
read_usv_state.o:read_usv_state.cpp
	${CXX} ${FLAG} read_usv_state.cpp
bd_comm.o:bd_comm.cpp
	${CXX} ${FLAG} bd_comm.cpp	
MsgQue.o:MsgQue.cpp
	${CXX} ${FLAG} MsgQue.cpp	
SysLog.o:SysLog.cpp
	${CXX} ${FLAG} SysLog.cpp
AisMsgQue.o:AisMsgQue.cpp
	${CXX} ${FLAG} AisMsgQue.cpp
BradioUdpThread.o:BradioUdpThread.cpp
	${CXX} ${FLAG} BradioUdpThread.cpp
ins_recorder.o:ins_recorder.cpp
	${CXX} ${FLAG} ins_recorder.cpp
can_deal_IDU.o:can_deal_IDU.cpp
	${CXX} ${FLAG} can_deal_IDU.cpp
can_deal_IHC.o:can_deal_IHC.cpp
	${CXX} ${FLAG} can_deal_IHC.cpp
can_deal_IOP.o:can_deal_IOP.cpp
	${CXX} ${FLAG} can_deal_IOP.cpp
can_deal_main.o:can_deal_main.cpp
	${CXX} ${FLAG} can_deal_main.cpp
comm_main.o:comm_main.cpp
	${CXX} ${FLAG} comm_main.cpp
adrc.o:adrc.cpp
	${CXX} ${FLAG} adrc.cpp
control_auto.o:control_auto.cpp
	${CXX} ${FLAG} control_auto.cpp
control_cmd.o:control_cmd.cpp
	${CXX} ${FLAG} control_cmd.cpp
control_jetCtrlFunc.o:control_jetCtrlFunc.cpp
	${CXX} ${FLAG} control_jetCtrlFunc.cpp
control_main.o:control_main.cpp
	${CXX} ${FLAG} control_main.cpp
control_manual.o:control_manual.cpp
	${CXX} ${FLAG} control_manual.cpp
control_operation.o:control_operation.cpp
	${CXX} ${FLAG} control_operation.cpp
control_semiAuto.o:control_semiAuto.cpp
	${CXX} ${FLAG} control_semiAuto.cpp
control_signalSt.o:control_signalSt.cpp
	${CXX} ${FLAG} control_signalSt.cpp
control_stanby.o:control_stanby.cpp
	${CXX} ${FLAG} control_stanby.cpp
ctrl_semiAuto_berth.o:ctrl_semiAuto_berth.cpp
	${CXX} ${FLAG} ctrl_semiAuto_berth.cpp
ctrl_semiAuto_headingConstant.o:ctrl_semiAuto_headingConstant.cpp
	${CXX} ${FLAG} ctrl_semiAuto_headingConstant.cpp
ctrl_semiAuto_speedConstant.o:ctrl_semiAuto_speedConstant.cpp
	${CXX} ${FLAG} ctrl_semiAuto_speedConstant.cpp
GlobalFunction.o:GlobalFunction.cpp
	${CXX} ${FLAG} GlobalFunction.cpp
uart_deal_DRadio.o:uart_deal_DRadio.cpp
	${CXX} ${FLAG} uart_deal_DRadio.cpp
uart_deal_ins.o:uart_deal_ins.cpp
	${CXX} ${FLAG} uart_deal_ins.cpp
sailWaypoint.o:sailWaypoint.cpp
	${CXX} ${FLAG} sailWaypoint.cpp
pid_control.o:pid_control.cpp
	${CXX} ${FLAG} pid_control.cpp
taskQue.o:taskQue.cpp
	${CXX} ${FLAG} taskQue.cpp
lan_deal_Bradio.o:lan_deal_Bradio.cpp
	${CXX} ${FLAG} lan_deal_Bradio.cpp	
watch_dog.o:watch_dog.cpp
	${CXX} ${FLAG} watch_dog.cpp
uart_deal_BD.o:uart_deal_BD.cpp
	${CXX} ${FLAG} uart_deal_BD.cpp
control_BD.o:control_BD.cpp
	${CXX} ${FLAG} control_BD.cpp
SysMsgPub.o:SysMsgPub.cpp
	${CXX} ${FLAG} SysMsgPub.cpp 
control_assistance.o:control_assistance.cpp
	${CXX} ${FLAG} control_assistance.cpp
ApfMethod.o:ApfMethod.cpp
	${CXX} ${FLAG} ApfMethod.cpp
zmqGetObstacles.o:zmqGetObstacles.cpp
	${CXX} ${FLAG} zmqGetObstacles.cpp
samplingComm.o:samplingComm.cpp
	${CXX} ${FLAG} samplingComm.cpp
nanoObstacleSender.o:nanoObstacleSender.cpp
	${CXX} ${FLAG} nanoObstacleSender.cpp
error_code.pb.o:./proto/obstacle/error_code.pb.cc
	${CXX} ${FLAG} ./proto/obstacle/error_code.pb.cc
geometry.pb.o:./proto/obstacle/geometry.pb.cc
	${CXX} ${FLAG} ./proto/obstacle/geometry.pb.cc
header.pb.o:./proto/obstacle/header.pb.cc
	${CXX} ${FLAG} ./proto/obstacle/header.pb.cc
obstacle_radar.pb.o:./proto/obstacle/obstacle_radar.pb.cc
	${CXX} ${FLAG} ./proto/obstacle/obstacle_radar.pb.cc
gnss_un237.pb.o:./proto/obstacle/gnss_un237.pb.cc
	${CXX} ${FLAG} ./proto/obstacle/gnss_un237.pb.cc
warnProcess.o: warnProcess.cpp
	${CXX} ${FLAG} warnProcess.cpp
lan_deal_CtrlCenter.o: lan_deal_CtrlCenter.cpp
	${CXX} ${FLAG} lan_deal_CtrlCenter.cpp
docking_main.o: docking_main.cpp
	${CXX} ${FLAG} docking_main.cpp
docking_usv_control.o:docking_usv_control.cpp
	${CXX} ${FLAG} docking_usv_control.cpp
docking_visual_guid.o:docking_visual_guid.cpp
	${CXX} ${FLAG} docking_visual_guid.cpp
docking_pid_loop.o:docking_pid_loop.cpp
	${CXX} ${FLAG} docking_pid_loop.cpp
docking_sumlink_tuning.o:docking_sumlink_tuning.cpp
	${CXX} ${FLAG} docking_sumlink_tuning.cpp
auto_return.o:auto_return.cpp
	${CXX} ${FLAG} auto_return.cpp
uart_irtk.o:uart_irtk.cpp
	${CXX} ${FLAG} uart_irtk.cpp
.PHONY:clean
clean:
	rm *.o -rf
