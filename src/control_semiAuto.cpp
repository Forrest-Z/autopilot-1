/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_semiAuto.h"
/******************************  Local Variable  *****************************/
SEMI_MODE_TRIG semi_modeTrig;
/******************************  Extern Variable  ****************************/
/******************************  Local Function   ****************************/
void getSemiModeTrig(void);
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/
void semiAuto_operation( void )
{
	dspControlOFF();
	getSemiModeTrig();	
	
	SemiAutoBerth();			
	SemiAutoSpeedConstant();	
	SemiAutoHeadingConstant();
}

void getSemiModeTrig(void)
{
	static uint8 b1_old_speedConstant   =0;
	static uint8 b1_old_headingConstant =0;
	static uint8 b1_old_setReturn       =0;
	static uint8 b1_old_autoReturn      =0;
	static uint8 b1_old_dock           =0;


	if(b1_old_speedConstant==COMMAND_OFF && command_signal.func_mode_cmd.b1_speedConstant == COMMAND_ON)
		semi_modeTrig.b1_trig_speedConstant = 1;
	else
		semi_modeTrig.b1_trig_speedConstant = 0;

	if(b1_old_headingConstant==COMMAND_OFF && command_signal.func_mode_cmd.b1_headingConstant == COMMAND_ON)
		semi_modeTrig.b1_trig_headingConstant = 1;
	else
		semi_modeTrig.b1_trig_headingConstant = 0;


	if (b1_old_dock == COMMAND_OFF && command_signal.func_mode_cmd.b1_dock_cmd == COMMAND_ON)
		semi_modeTrig.b1_trig_berth = 1;
	else
		semi_modeTrig.b1_trig_berth = 0;


	b1_old_speedConstant   = command_signal.func_mode_cmd.b1_speedConstant	;
	b1_old_headingConstant = command_signal.func_mode_cmd.b1_headingConstant;
	b1_old_setReturn       = command_signal.func_mode_cmd.b1_setReturn		;
	b1_old_autoReturn      = command_signal.func_mode_cmd.b1_autoReturn		;
	b1_old_dock = command_signal.func_mode_cmd.b1_dock_cmd;
}

