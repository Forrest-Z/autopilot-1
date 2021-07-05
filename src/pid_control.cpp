#include "stdafx.h"
#include "../include/usv_include.h"
#include "../include/pid_control.h"



simu_Spd_PID r_PID;
simu_Spd_PID r_PID_R;	

simu_Spd_PID r_PID_LowSpeed_L;	//��תPID����
simu_Spd_PID r_PID_LowSpeed_R;	//��תPID����

simu_Spd_PID v_PID;


CPidModel *pPidAutoSpeed;

CPidModel *pPidHeading;

CPidModel *pPidRot;

/* Time funcitons */
uint32 micros()
{
    uint64 t = 0;

#ifdef WINNT
#define EPOCHFILETIME   (116444736000000000UL)
	FILETIME ft;
	///LARGE_INTEGER freq;
	LARGE_INTEGER time;
	int64_t tt = 0;
	GetSystemTimeAsFileTime(&ft);
	time.LowPart = ft.dwLowDateTime;
	time.HighPart = ft.dwHighDateTime;
	tt = (time.QuadPart - EPOCHFILETIME) / 10;
	t = tt;
	///QueryPerformanceFrequency(&freq); //��ȡʱ������
	///QueryPerformanceCounter(&time);	//��ȡ��ǰʱ��

#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	t = 1.0e6*(ts.tv_sec + (ts.tv_nsec*1.0e-9));
#endif

    return (t & 0xFFFFFFFF);
}

uint32 millis()
{
	uint64 t = 0;
#ifdef WINNT

	FILETIME ft;
	///LARGE_INTEGER freq;
	LARGE_INTEGER time;
	int64_t tt = 0;
	GetSystemTimeAsFileTime(&ft);
	time.LowPart = ft.dwLowDateTime;
	time.HighPart = ft.dwHighDateTime;
	tt = (time.QuadPart - EPOCHFILETIME) / 10;
	t = tt * 1.0e-3;
#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	t = 1.0e3*(ts.tv_sec + (ts.tv_nsec*1.0e-9));
#endif
    return (t & 0xFFFFFFFF);
}




void pid_contorl_init(void)
{
	r_PID_init();
	v_PID_init();

	pPidAutoSpeed = new CPidModel(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].P,	
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[5].D);
	pPidAutoSpeed->setPidOutLimit(OPEN_DEG_MAX, 0);		


	pPidHeading = new CPidModel(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].P,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[6].D);
	pPidHeading->setPidOutLimit(10.0, -10.0);			

	
	pPidRot = new CPidModel(monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].P,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].I,
		monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[7].D);
	pPidRot->setPidOutLimit(RUDDER_UP, RUDDER_DOWN);

}

float constrain(float num, float low, float high) //ֵԼ��
{
	return ((num) < (low) ? (low) : ((num) > (high) ? (high) : (num)));
}
void paramPIDinit(void)
{
	r_PID.LastError = 0;
	r_PID.PreError = 0;
	r_PID.Integral_error = 0;
	r_PID.pid_out = 0;

	v_PID.LastError = 0;
	v_PID.PreError = 0;
	v_PID.Integral_error = 0;
	v_PID.pid_out = 0;
}

void r_PID_init(void)
{
	//r_PID.Proportion	= 1		;
	//r_PID.Integral		= 0.0	;
	//r_PID.Derivative	= 0.01	;
	r_PID.Proportion = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].P	;
	r_PID.Integral   = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].I	;
	r_PID.Derivative = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[0].D	;


	r_PID_R.Proportion = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[2].P	;
	r_PID_R.Integral   = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[2].I	;
	r_PID_R.Derivative = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[2].D	;


	r_PID_LowSpeed_L.Proportion = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[3].P	;
	r_PID_LowSpeed_L.Integral   = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[3].I	;
	r_PID_LowSpeed_L.Derivative = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[3].D	;


	r_PID_LowSpeed_R.Proportion = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[4].P	;
	r_PID_LowSpeed_R.Integral   = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[4].I	;
	r_PID_LowSpeed_R.Derivative = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[4].D	;
}

void v_PID_init(void)
{
	//v_PID.Proportion    = 3000.0/40.0*0.001			;
	//v_PID.Integral		= 0.00						;
	//v_PID.Derivative	= 0.00001					;
	//v_PID.pid_out		= 0.0						;
	v_PID.Proportion = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].P	;
	v_PID.Integral   = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].I	;
	v_PID.Derivative = & monitor_all_inf.rec_monitor_all_set_param.monitor_set_pid_param[1].D	;
}

float simu_speed_PID(float speed_exp,float speed_real)
{
	static double rev_bak	= 0		;
	static double T			= 1   	;

	double Proportion_local ;
	v_PID.SetPoint	= speed_exp						;		
	v_PID.Actual	= speed_real					;
	v_PID.Error		= v_PID.SetPoint - v_PID.Actual	;

	v_PID.Ec			 = v_PID.Error - v_PID.LastError;
	v_PID.PreError		 = v_PID.LastError				;
	v_PID.LastError		 = v_PID.Error					;
	v_PID.Integral_error+= v_PID.Error					;
	if (v_PID.Integral_error >= 1000)
	{
		v_PID.Integral_error = 1000;
	}
	else if(v_PID.Integral_error <= -1000)
	{
		v_PID.Integral_error = -1000;
	}

	
	v_PID.pid_out +=  (*v_PID.Proportion)*v_PID.Error + (*v_PID.Integral) *v_PID.Integral_error + (*v_PID.Derivative) *v_PID.Ec;

	

	if(v_PID.pid_out > OPEN_DEG_MAX)
		v_PID.pid_out = OPEN_DEG_MAX;
	if(v_PID.pid_out < 0)
		v_PID.pid_out = 0	;
	
	return v_PID.pid_out;
}

#define HEADING_INTEGRAL_MAX  7200000.0
#define HEADING_INTERGAL_MIN -7200000.0

float simu_heading_PID(float heading_exp,float heading_real)
{
	

	r_PID.pid_out	=	0				;
	r_PID.SetPoint	=	heading_exp		;
	r_PID.Actual	=	heading_real	;
	
	r_PID.Error		=	r_PID.SetPoint - r_PID.Actual	;
	
	if(r_PID.Error>180.0)
		r_PID.Error = r_PID.Error - 360.0;
	else if(r_PID.Error < -180)
		r_PID.Error = r_PID.Error + 360.0;

	r_PID.Ec				=	r_PID.Error - r_PID.LastError	;
	r_PID.LastError			=	r_PID.Error	;
	r_PID.Integral_error   +=	r_PID.Error	;

	if (r_PID.Integral_error > HEADING_INTEGRAL_MAX)
	{
		r_PID.Integral_error = HEADING_INTEGRAL_MAX;
	}
	else if(r_PID.Integral_error < HEADING_INTERGAL_MIN)
	{
		r_PID.Integral_error = HEADING_INTERGAL_MIN;
	}
	else ;

	r_PID.pid_out = (*r_PID.Proportion) *r_PID.Error + (*r_PID.Integral) *r_PID.Integral_error + (*r_PID.Derivative) *r_PID.Ec	;


	if(r_PID.pid_out > RUDDER_UP)
		r_PID.pid_out = RUDDER_UP;
	else if(r_PID.pid_out < RUDDER_DOWN)
		r_PID.pid_out = RUDDER_DOWN;

	return r_PID.pid_out;
}

float simu_heading_TJ_PID( float heading_exp,float heading_real )
{
	float iretPidOut = 0;

	r_PID.pid_out	=	0				;
	r_PID.SetPoint	=	heading_exp		;
	r_PID.Actual	=	heading_real	;

	r_PID.Error		=	r_PID.SetPoint - r_PID.Actual	;

	if(r_PID.Error>180.0)
		r_PID.Error = r_PID.Error - 360.0;
	else if(r_PID.Error < -180)
		r_PID.Error = r_PID.Error + 360.0;

	r_PID.Ec				=	r_PID.Error - r_PID.LastError	;
	r_PID.LastError			=	r_PID.Error	;
	r_PID.Integral_error   +=	r_PID.Error	;

	if (r_PID.Integral_error > HEADING_INTEGRAL_MAX)
	{
		r_PID.Integral_error = HEADING_INTEGRAL_MAX;
	}
	else if(r_PID.Integral_error < HEADING_INTERGAL_MIN)
	{
		r_PID.Integral_error = HEADING_INTERGAL_MIN;
	}
	else ;

	if(r_PID.Error > 15.0 || r_PID.Error < -15.0)	//����ת��ʱ��������
		r_PID.Integral_error = 0.0;


	r_PID.pid_out = (*r_PID.Proportion) *r_PID.Error + (*r_PID.Integral) *r_PID.Integral_error + (*r_PID.Derivative) *r_PID.Ec	;
	r_PID_R.pid_out = (*r_PID_R.Proportion) *r_PID.Error + (*r_PID_R.Integral) *r_PID_R.Integral_error + (*r_PID_R.Derivative) *r_PID.Ec	;

	if(r_PID.Error < 0.0) //��ת��
		iretPidOut = r_PID.pid_out;
	else
		iretPidOut = r_PID_R.pid_out;


	if(iretPidOut > RUDDER_UP)
		iretPidOut = RUDDER_UP;
	else if(iretPidOut < RUDDER_DOWN)
		iretPidOut = RUDDER_DOWN;

	return iretPidOut;
}

float simu_heading_TJ_PID_LowSpeed( float heading_exp,float heading_real )
{
	float iretPidOut = 0;

	r_PID_LowSpeed_L.pid_out	=	0				;
	r_PID_LowSpeed_L.SetPoint	=	heading_exp		;
	r_PID_LowSpeed_L.Actual	=	heading_real	;

	r_PID_LowSpeed_L.Error		=	r_PID_LowSpeed_L.SetPoint - r_PID_LowSpeed_L.Actual	;

	if(r_PID_LowSpeed_L.Error>180.0)
		r_PID_LowSpeed_L.Error = r_PID_LowSpeed_L.Error - 360.0;
	else if(r_PID_LowSpeed_L.Error < -180)
		r_PID_LowSpeed_L.Error = r_PID_LowSpeed_L.Error + 360.0;

	r_PID_LowSpeed_L.Ec				=	r_PID_LowSpeed_L.Error - r_PID_LowSpeed_L.LastError	;
	r_PID_LowSpeed_L.LastError			=	r_PID_LowSpeed_L.Error	;
	r_PID_LowSpeed_L.Integral_error   +=	r_PID_LowSpeed_L.Error	;

	if (r_PID_LowSpeed_L.Integral_error > HEADING_INTEGRAL_MAX)
	{
		r_PID_LowSpeed_L.Integral_error = HEADING_INTEGRAL_MAX;
	}
	else if(r_PID_LowSpeed_L.Integral_error < HEADING_INTERGAL_MIN)
	{
		r_PID_LowSpeed_L.Integral_error = HEADING_INTERGAL_MIN;
	}
	else ;

	if(r_PID_LowSpeed_L.Error > 15.0 || r_PID_LowSpeed_L.Error < -15.0)	//����ת��ʱ��������
		r_PID_LowSpeed_L.Integral_error = 0.0;


	r_PID_LowSpeed_L.pid_out = (*r_PID_LowSpeed_L.Proportion) *r_PID_LowSpeed_L.Error + (*r_PID_LowSpeed_L.Integral) *r_PID_LowSpeed_L.Integral_error + (*r_PID_LowSpeed_L.Derivative) *r_PID_LowSpeed_L.Ec	;
	r_PID_LowSpeed_R.pid_out = (*r_PID_LowSpeed_R.Proportion) *r_PID_LowSpeed_L.Error + (*r_PID_LowSpeed_R.Integral) *r_PID_LowSpeed_L.Integral_error + (*r_PID_LowSpeed_R.Derivative) *r_PID_LowSpeed_L.Ec	;



	if(r_PID_LowSpeed_L.Error < 0.0) //��ת��
		iretPidOut = r_PID_LowSpeed_L.pid_out;
	else
		iretPidOut = r_PID_LowSpeed_R.pid_out;


	if(iretPidOut > RUDDER_UP)
		iretPidOut = RUDDER_UP;
	else if(iretPidOut < RUDDER_DOWN)
		iretPidOut = RUDDER_DOWN;

	return iretPidOut;
}



//CPidModel Class
CPidModel::CPidModel(float p, float i, float d)
{
	m_setValue = 0;					// ����ֵ
	m_measureValue = 0;				// ʵ��ֵ
	m_coff_P = p;					// ����ϵ��
	m_coff_I = i;					// ����ϵ��
	m_coff_D = d;					// ΢��ϵ��
	m_error = 0;					// ��ǰ���
	m_errorLast = 0;				// Error[-1]ǰһ�����
	m_errorPre = 0;					// Error[-2]ǰ�������
	m_ecD = 0;						//
	m_ecP = 0;						//
	m_ecI = 0;						//
	m_ecIMax = 7200000.0;			//
	m_ecIMin = -7200000.0;			//
	m_pidOut = 0;					// pid���ֵ
}

void CPidModel::setPidCoff(float p, float i, float d)
{
	m_coff_P = p;					// ����ϵ��
	m_coff_I = i;					// ����ϵ��
	m_coff_D = d;					// ΢��ϵ��
}


void CPidModel::resetPidParam(void)
{
	m_setValue = 0;				// ����ֵ
	m_measureValue = 0;			// ʵ��ֵ
	m_error = 0;				// ��ǰ���
	m_errorLast = 0;			// Error[-1]ǰһ�����
	m_errorPre = 0;				// Error[-2]ǰ�������
	m_ecD = 0;					// 
	m_ecP = 0;					//
	m_ecI = 0;					//
	//m_pidOut = 0;				// pid���ֵ
}

float CPidModel::pidRealize(float giveValue, float measureValue)
{
	static float m_last_i_coff = m_coff_I;
	static uint32 tlast = 0;
	uint32 tnow = millis();
	float dt = 0;

	if(tnow - tlast > 200 || tlast == 0)
	{
		dt = 0.05;
	}
	else
	{
		dt = (tnow - tlast) * 1.0e-3f;
	}

	if(m_last_i_coff != m_coff_I || (tnow - tlast) >= 200 || (tlast  == 0))
	{
		m_ecI = 0;
		m_errorLast = 0;
		m_setValue = 0;
		last_set = 0;
	}

	m_error = giveValue - measureValue;

	m_ecD = m_error - m_errorLast;
	m_ecP = m_error;
	m_ecI += m_error;


	m_ecI += m_error;
	m_errorPre = m_errorLast;
	m_errorLast = m_error;

	if (m_ecI > m_pidOutMax*0.67){
		m_ecI = m_pidOutMax*0.67;
	}
	else if (m_ecI < m_pidOutMin*0.67){
		m_ecI = m_pidOutMin*0.67;
	}
	else{}

	//m_pidOut = m_ecP * m_coff_P + m_ecI * m_coff_I + m_ecD * m_coff_D;
	//m_pidOut = m_ecP * m_coff_P + m_ecI * m_coff_I + m_ecD * m_coff_D;
	m_pidOut = sqrt_controller(m_ecP,m_coff_P,m_coff_D,dt) + m_ecI * m_coff_I;

	if (m_pidOut > m_pidOutMax){
		m_pidOut = m_pidOutMax;
	}
	else if (m_pidOut < m_pidOutMin){
		m_pidOut = m_pidOutMin;
	}
	else{

	}
	m_last_i_coff = m_coff_I;
	tlast = tnow;
	//printf("omega:p = %f,i = %f,d = %f,pidOut = %f\n",m_coff_P,m_coff_I,m_coff_D,m_pidOut);
	return m_pidOut;
}


float CPidModel::pidIncrease(float giveValue, float measureValue)
{
	static float m_last_i_coff = m_coff_I;
	static uint32 tlast = 0;
	uint32 tnow = millis();
	float dt = 0.05;
	const float acc = 0.5;


	if(m_last_i_coff != m_coff_I || (tnow - tlast) >= 200 || (tlast  == 0))
	{
		m_pidOut = 0;
		m_errorPre = 0;
		m_errorLast = 0;
		m_setValue = 0;
		last_set = 0;
	}

	m_measureValue = measureValue;
	m_setValue = giveValue;
	float change = acc * dt;
	if(m_setValue > (change + last_set)){
		m_setValue = (change + last_set);
	}
	
	if(m_setValue < (last_set - change)){
		m_setValue = (last_set - change);
	}
	
	last_set = m_setValue;

	m_error = m_setValue - m_measureValue;
	m_ecD = m_error - 2*m_errorLast + m_errorPre;
	m_ecP = m_error - m_errorLast;

	m_errorPre = m_errorLast;
	m_errorLast = m_error;
	
	m_pidOut += m_coff_P * m_ecP + m_coff_I * m_error + m_coff_D * m_ecD;

	if (m_pidOut > m_pidOutMax){
		m_pidOut = m_pidOutMax;
	}		
	else if (m_pidOut < m_pidOutMin){
		m_pidOut = m_pidOutMin;
	}
	else{

	}

	m_last_i_coff = m_coff_I;
	tlast = tnow;

	return m_pidOut;
}

void CPidModel::setPidOutLimit(float iMax, float iMin)
{
	m_pidOutMax = iMax;
	m_pidOutMin = iMin;
}

float CPidModel::pidHeadingCalc(float  setHeading, float measureHeading)
{
	static uint32 tlast = 0;
	static float  last_ki = m_coff_I;
	static float  last_set = 0;
	uint32 tnow = millis();

	float dt = 0;
	if(tnow - tlast > 200 || tlast == 0)
	{
		dt = 0.05;
	}
	else
	{
		dt = (tnow - tlast) * 1.0e-3f;
	}

	if(tnow  - tlast > 200 || last_ki != m_coff_I || tlast == 0)
	{
		m_ecI = 0;
		m_errorLast = measureHeading;
		m_errorPre = 0;
	}
	
	m_setValue     = setHeading;
	m_measureValue = measureHeading;
	m_error        = m_setValue - m_measureValue;

	// wrap_180(x)
	if (m_error > 180.0)
	{
		m_error = m_error - 360;
	}
	else if (m_error < -180.0)
	{
		m_error = m_error + 360;
	}		
	if (isnan(m_error)){
		m_error = 0;
	}

	// PID
	m_ecD = (m_error - m_errorLast);
	m_ecP = m_error;

	if(fabsf(m_error) < 2)
	{
		m_ecI += m_error;
	}
	else
	{
		m_ecI = 0;
	}
	
	m_errorPre = m_errorLast;
	m_errorLast = m_error;

	if (m_ecI > m_pidOutMax*0.67){
		m_ecI = m_pidOutMax*0.67;
	}
	else if (m_ecI < m_pidOutMin*0.67){
		m_ecI = m_pidOutMin*0.67;
	}
	else;

	//m_pidOut = m_ecP * m_coff_P+ m_ecI * m_coff_I + m_ecD * m_coff_D;
	m_pidOut = sqrt_controller(m_ecP,m_coff_P,m_coff_D,dt) + m_ecI * m_coff_I;

	if (m_pidOut > m_pidOutMax){
		m_pidOut = m_pidOutMax;
	}
	else if(m_pidOut < m_pidOutMin){
		m_pidOut = m_pidOutMin;
	}
	else{

	}
	tlast = tnow;
	last_ki = m_coff_I;

	//printf("heading:p = %f,i = %f,d = %f,pidOut = %f\n",m_coff_P,m_coff_I,m_coff_D,m_pidOut);

	return m_pidOut;
}


void CPidModel::setPidIntergLimit(float iMax, float iMin)
{
	m_ecIMax = iMax;
	m_ecIMin = iMin;
}

float CPidModel::getSetingValue(void)
{
	return m_setValue;
}

float CPidModel::getOutputValue(void)
{
	return m_pidOut;
}

void CPidModel::setOutputValue(float setValue)
{
	float tmpValue;
	if (setValue > m_pidOutMax){
		tmpValue = m_pidOutMax;
	}		
	else if (setValue < m_pidOutMin){
		tmpValue = m_pidOutMin;
	}		
	else{
		tmpValue = setValue;
	}

	m_pidOut = setValue;
		
}

