#ifndef PID_CONTROL_H
#define PID_CONTROL_H



//PID�ṹ�嶨��
typedef struct 
{             
	float  SetPoint			;          		//�趨ֵ
	float  Actual			;				//ʵ��ֵ
	float  *Proportion		;      	 		// Proportion ����ϵ��
	float  *Integral			;          		// Integral   ����ϵ��
	float  *Derivative		;          		// Derivative  ΢��ϵ��
	float  Error			;				//��ǰ���
	float  LastError		;      			// Error[-1]  ǰһ����� ----
	float  PreError			;      			// Error[-2]  ǰ������� ----
	float  Ec				;				//���仯��
	float  Integral_error	;            	// -----
	float  pid_out			;				//pid �ۼ�ֵ ---- 
} simu_Spd_PID;

extern simu_Spd_PID r_PID;
extern simu_Spd_PID v_PID;

extern void pid_contorl_init(void);

void r_PID_init(void);
void v_PID_init(void);
extern float simu_speed_PID(float speed_exp,float speed_real);
extern float simu_heading_PID(float heading_exp,float heading_real);
extern float simu_heading_TJ_PID(float heading_exp,float heading_real);
extern float simu_heading_TJ_PID_LowSpeed(float heading_exp,float heading_real);	//�켫����pid
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

	float pidIncrease(float giveValue, float measureValue);		//����ʽPID
	float pidRealize(float giveValue, float measureValue);		//λ��ʽPID
	float pidHeadingCalc(float setHeading, float measureHeading);	//����PID �������ƫ�� λ��ʽPID
	float getSetingValue(void);
	float getOutputValue(void);
	void setOutputValue(float setValue);
	void reset_speed_pid();
	void reset_heading_pid();

private:
	float m_setValue;		// ����ֵ
	float last_set;
	float m_measureValue;	// ʵ��ֵ
	float m_coff_P;			// ����ϵ��
	float m_coff_I;			// ����ϵ��
	float m_coff_D;			// ΢��ϵ��
	float m_error;			// ��ǰ���
	float m_errorLast;		// Error[-1]ǰһ�����
	float m_errorPre;		// Error[-2]ǰ�������
	float m_ecD;			// ���΢����
	float m_ecP;			// ��������
	float m_ecI;			// ��������
	float m_ecIMax;			// ����������
	float m_ecIMin;			// ����������
	float m_pidOutMax;		//PID�������
	float m_pidOutMin;		//PID�������
	float m_pidOut;			//pid���ֵ
};

extern CPidModel *pPidAutoSpeed;
extern CPidModel *pPidHeading;
extern CPidModel *pPidRot;
#endif /*PID_CONTROL_H*/


