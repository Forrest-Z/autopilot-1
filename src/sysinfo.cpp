/*
* sysinfo.c --��ȡϵͳcpu�ڴ���Ϣ
* �ķ��̱�(  �人)  ������޹�˾
*
* ��ʷ��¼��
*
* V1.00��2016-04-25������������д
*/

#include "stdafx.h"
#include "../include/usv_include.h"

float View_MEM(void);
float View_CPU(void) ; 

void *GetSysinfo(void *aa)
{        
	float CPU_Use=0.0,MEM_Use=0.0;  
	while(1)
	{
	        CPU_Use = View_CPU();  
		MCU_St.RAM_CPU=(uint8)(CPU_Use*200);	
//	        printf("cpu use = %.4f%\n",CPU_Use*100);  
		MEM_Use=View_MEM();
		MCU_St.RAM_Memory=(uint8)(MEM_Use*200);
//		printf("mem use = %.4f%\n",MEM_Use*100);
	}
	return ((void *)0);
}
float View_CPU(void)  
{  
        FILE *fp;  
        char buf[128];  
        char cpu[5];  
         int user,nice,sys,idle,iowait,irq,softirq;  
  
        long int all1,all2,idle1,idle2;  
        float usage;  
          
        fp = fopen("/proc/stat","r");  
        if(fp == NULL)  
        {  
                perror("fopen:");  
                exit (0);  
        }  

        fgets(buf,sizeof(buf),fp);  
#if __DEBUG__  
        printf("buf=%s",buf);  
#endif  
        sscanf_usv(buf,"%s%d%d%d%d%d%d%d",cpu,&user,&nice,&sys,&idle,&iowait,&irq,&softirq);  
/* 
#if __DEBUG__ 
printf("%s,%d,%d,%d,%d,%d,%d,%d\n",cpu,user,nice,sys,idle,iowait,irq,softirq); 
#endif 
*/  
        all1 = user+nice+sys+idle+iowait+irq+softirq;  
        idle1 = idle;  
        rewind(fp);  
        /*\u7b2c\u4e8c\u6b21\u53d6\u6570\u636e*/  
	sleep_1(1000);
        memset(buf,0,sizeof(buf));  
        cpu[0] = '\0';  
        user=nice=sys=idle=iowait=irq=softirq=0;  
        fgets(buf,sizeof(buf),fp);  
#if __DEBUG__  
        printf("buf=%s",buf);  
#endif  
        sscanf_usv(buf,"%s%d%d%d%d%d%d%d",cpu,&user,&nice,&sys,&idle,&iowait,&irq,&softirq);  
/* 
#if __DEBUG__ 
printf("%s,%d,%d,%d,%d,%d,%d,%d\n",cpu,user,nice,sys,idle,iowait,irq,softirq); 
#endif 
*/  
        all2 = user+nice+sys+idle+iowait+irq+softirq;  
        idle2 = idle;  
        usage = (float)(all2-all1-(idle2-idle1)) / (all2-all1) ;  
        fclose(fp);  
        return usage; 
} 
float View_MEM(void)
{
#ifndef WINNT
	FILE   *stream;
//	FILE   *wstream;
	char   buff[1024];
	int    get[50];
//	char   *pLine;
	int    ico=0,count=0,meminfo[4];
	float  mem=0.0;
	memset( buff, '\0', sizeof(buff) );//��ʼ��buf,�������д�����뵽�ļ���
	memset(get,0,sizeof(get));
	memset(meminfo,0,sizeof(meminfo));		
	stream = popen( "cat /proc/meminfo", "r" ); //��"cat /proc/meminfo���������� ͨ���ܵ���ȡ����r����������FILE* stream
//	wstream = fopen( "/media/test_popen.txt", "w+"); //�½�һ����д���ļ�
	fread( buff, sizeof(char), sizeof(buff), stream); //���ո�FILE* stream����������ȡ��buf��
	for(ico=0;ico<1024;ico++)
	{

		if(buff[ico]=='\n')	
		{
			count++;
			if(count>3)
				break;	
		}		
				
		else
		{

			if((48<=buff[ico])&&(buff[ico]<=57))
			{
				meminfo[count]*=10;
				meminfo[count]+=(buff[ico]-48);	
			}
			//printf("meminfo[%d]=%d\n",count,meminfo[count]);
		}		
	}
	mem=1.0-(float)(meminfo[1]+meminfo[2]+meminfo[3])/(float)(meminfo[0]);
//	write( buf, 1, sizeof(buf), wstream );//��buf�е�����д��FILE    *wstream��Ӧ�����У�Ҳ��д���ļ���

	pclose( stream );  
//	fclose( wstream );

	return mem;
#else
	return 0;
#endif
}



