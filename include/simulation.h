#ifndef		SIMULATION_H
#define     SIMULATION_H

//Ŀ�ĵؽṹ��
typedef struct
{
	double dst_lat;
	double dst_lng;
}simu_dst_location;


void *simulation_sailing(void *aa);
void *simu_pid(void *aa);
void *simu_path(void *aa);




#endif /* SIMULATION_H */