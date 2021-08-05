#pragma once

#include <vector>
#include <string>
#include "usv_include.h"

#ifndef WINNT
const string AUTO_RETURN_CFG_FILE_NAME = "../cfg/AutoReturn.cfg";	//�����ļ�
#else
const string AUTO_RETURN_CFG_FILE_NAME = "../../cfg/AutoReturn.cfg";	//�����ļ�
#endif


class CAutoReturn
{
public:
	enum retProcess{
		RETURN_NONE = 0,
		RETURN_PATH,		
		RETURN_GPOINT,		
		RETURN_DOCK			
	};

	CAutoReturn(const string& cfgFileName);
	~CAutoReturn();

public:
	void JudgeNeedPowerOffAutoReturn(POSITION locPos, uint32 powerPercent);		// �����ж���Ҫ����
	void JudgeNeedPowerOffAutoReturn(uint32 powerPercent);						// ����ֱ���ж���Ҫ����
	bool IsLowPowerAutoReturnNeeded(void);											// �����Ƿ���Ҫ�͵����Զ�����
	bool IsForceAutoReturnNeeded(void);										// ǿ�Ʒ���
	bool IsAutoReturnNeeded(void);											// ���ַ���


	int8 ControlAutoReturnMain(void);										// ��ѭ�����������˴�ִ�к���


	void TurnOnAutoReturn(void);
	void TurnOffAutoReturn(void);


	void reset_AutoReturn(void);

	
	retProcess getReturnState(void);	//��ȡ�Զ�����״̬
	bool isAutoReturnRunning(void);		//�Զ�����״̬��ȡ
	bool isAutoReturnCfgValid(void);	//�Զ�����������Ч
	
	POSITION getDestPoint(void);		//��ȡ�Զ�����λ��

private:

	retProcess m_retProcess;

	bool read_AutoReturn_Setting(string cfgFileName);


	void AutoReturn_JudgeStart(void);

	void AutoReturn_SailPath(void);

	void AutoReturn_GDPoint(void);

	void AutoReturn_GDock(void);


	void DoRudderOpenDeg(double speed, double heading);


	void auto_return_plan(POSITION last_point, POSITION target_point);


	void auto_aovid_return_plan(POSITION last_point, POSITION target_point);


	float sqrt_controller(float error, float p, float second_ord_lim);
    
private:
	
	uint32  m_FuncEnable;						
	uint32	m_DockID;							

	AUTO_NAVI_ST auto_return_st;
	AUTO_NAVI_CFG auto_return_cfg;


	POSITION init_ret_point;					
	POSITION m_DockPos;							//����λ��
	POSITION m_ReturnPos;						//�Զ���������λ��
	vector <POSITION>	m_ReturnPath;			//�Զ�������·
	uint32	m_ReturnPowerLimit;					//�Զ��������� �ٷֱ�
	double	m_ExpSpeed;							//��������	kn
	double	m_ArrSpeed;							//���ﺽ��	kn
	double  m_ArrDst;							//�������	m
	double  m_DecDst;							//���پ���	m

	
	uint32	m_ReturnPointSn = 0;				//�Զ�������·ִ�����
	bool	m_bIsReturnPathProc = false;			//�Զ������Ƿ�ִ��
	bool	m_bIsReturnPathFinish = false;		//�Զ�������·�Ƿ�ִ����
	bool	m_bIsReturnPosArrived = false;		//�Զ���������λ���Ƿ񵽴�
	bool	m_bIsAutoGetInDockReady = false;	//�Զ������Ƿ����

	//
	bool	m_IsCfgValid = false;				//�����ļ���Ч
	bool	m_bPowerLowNeedReturn = false;		//��Ҫ�����ͷ���
	bool	m_bForceAutoReturn = false;			//ǿ���Զ�����
	float   _oa_speed_ms;
};



extern CAutoReturn *pAutoReturnInst;	//�͵����Զ�����