//ctrl_semiAuto_headingConstant.h

#ifndef __CTRL_SEMIAUTO_HEADINGCONSTANT__H_
#define __CTRL_SEMIAUTO_HEADINGCONSTANT__H_

typedef struct{
    double double_heading_exp;
}HEADING_CONSTANT_ST;

extern HEADING_CONSTANT_ST heading_const_st;

void GetHeadingConstant(void);  //��ö�����
void SemiAutoHeadingConstant(void); //  ������


#endif /*__CTRL_SEMIAUTO_HEADINGCONSTANT__H_*/