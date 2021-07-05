#ifndef PID_CONTROL_H
#define PID_CONTROL_H



//PID结构体定义
typedef struct 
{             
	float  SetPoint			;          		//设定值
	float  Actual			;				//实际值
	float  *Proportion		;      	 		// Proportion 比例系数
	float  *Integral			;          		// Integral   积分系数
	float  *Derivative		;          		// Derivative  微分系数
	float  Error			;				//当前误差
	float  LastError		;      			// Error[-1]  前一拍误差 ----
	float  PreError			;      			// Error[-2]  前两拍误差 ----
	float  Ec				;				//误差变化量
	float  Integral_error	;            	// -----
	float  pid_out			;				//pid 累加值 ---- 
} simu_Spd_PID;

extern simu_Spd_PID r_PID;
extern simu_Spd_PID v_PID;

extern void pid_contorl_init(void);

void r_PID_init(void);
void v_PID_init(void);
extern float simu_speed_PID(float speed_exp,float speed_real);
extern float simu_heading_PID(float heading_exp,float heading_real);
extern float simu_heading_TJ_PID(float heading_exp,float heading_real);
extern float simu_heading_TJ_PID_LowSpeed(float heading_exp,float heading_real);	//天极低速pid
extern void  paramPIDinit(void);


extern uint32 micros();
extern uint32 millis();


class CPidModel
{
public:
	CPidModel(float p,float i,float d);
	~CPidModel();

	void setPidCoff(float p, float i, float d);
	void setPidIntergLimit(float iMax, float iMin);
	void setPidOutLimit(float iMax, float iMin);
	void resetPidParam(void);

	float pidIncrease(float giveValue, float measureValue);		//增量式PID
	float pidRealize(float giveValue, float measureValue);		//位置式PID
	float pidHeadingCalc(float setHeading, float measureHeading);	//航向PID 输出航向偏差 位置式PID
	float getSetingValue(void);
	float getOutputValue(void);
	void setOutputValue(float setValue);
	void reset_speed_pid();
	void reset_heading_pid();

private:
	float m_setValue;		// 给定值
	float last_set;
	float m_measureValue;	// 实际值
	float m_coff_P;			// 比例系数
	float m_coff_I;			// 积分系数
	float m_coff_D;			// 微分系数
	float m_error;			// 当前误差
	float m_errorLast;		// Error[-1]前一拍误差
	float m_errorPre;		// Error[-2]前两拍误差
	float m_ecD;			// 误差微分量
	float m_ecP;			// 误差比例量
	float m_ecI;			// 误差积分量
	float m_ecIMax;			// 积分量上限
	float m_ecIMin;			// 积分量下限
	float m_pidOutMax;		//PID输出上限
	float m_pidOutMin;		//PID输出下限
	float m_pidOut;			//pid输出值
};

extern CPidModel *pPidAutoSpeed;
extern CPidModel *pPidHeading;
extern CPidModel *pPidRot;
#endif /*PID_CONTROL_H*/


