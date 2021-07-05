
/**********************************  Include  ********************************/
#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/control_signalSt.h"

/******************************  Local Variable  *****************************/
STATE_SIGNAL state_signal;
int			 timetick;
/******************************  Extern Variable  ****************************/
void getSignal(void);

/******************************  Local Function   ****************************/
/******************************  Extern Function  ****************************/
/******************************    Code   ************************************/

void initStateSignal( void )
{
	memset((uint8 *)&state_signal,0,sizeof(STATE_SIGNAL));
	addTask(1,runStateSignal,(void*)0);

	timetick = 0;
	
	//insRecordInit();
}

void runStateSignal( void * )
{
	getSignal();


	//if(timetick <  18000)
	//{
	//	insRecordSave();
	//}
	//if(timetick == 18000)
	//{
	//	insRecordClose();
	//}

	//if (timetick <=  18000)
	//{
	//	timetick++;
	//}
	
	
}


void getSignal(void)
{
	getINS();
}

void getINS(void)
{
	uint8 hour,date,month,year;
	state_signal.time.u8_second = ins_msg.u8_second;
	state_signal.time.u8_minute = ins_msg.u8_minute;
	
	hour  = ins_msg.u8_hour + 8	;
	date  = ins_msg.u8_date		;
	month = ins_msg.u8_month	;
	year  = ins_msg.u8_year		;
	if(hour>24)
	{
		state_signal.time.u8_hour = hour-24;
		date++;
		if (month == 1||month == 3||month == 5||month == 7||month == 8||month == 10||month == 12) //31����
		{
			if(date>31)
			{
				date = 1;
				month++;
			}
		}
		else if(month == 4||month == 6||month == 9||month == 11)	//30����
		{
			if(date>30)
			{
				date = 1;
				month++;
			}
		}
		else if(month == 2)	
		{
			if(year%4 == 0)	
			{
				if(date>29)	
				{
					date = 1;
					month++;
				}
			}
			else			
			{
				if(date>28)
				{
					date = 1;
					month++;
				}
			}
		}
		else ;
		if (month>12)
		{
			month = 1;
			year ++;
		}
	}
	state_signal.time.u8_hour = hour	;
	state_signal.time.u8_date = date	;
	state_signal.time.u8_month= month	;
	state_signal.time.u8_year = year	;
	
}