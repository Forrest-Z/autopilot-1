#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#ifndef WINNT
#include <unistd.h>
#endif

#include "../include/usv_include.h"
	
//index=200~207,±íÊ¾IDO0~IDO7

int set_direction(char* index, char* direction)
{
int ret = 0;
#ifndef WINNT
	int fd;
	char filename[64];
	

	sprintf_usv(filename, "/sys/class/gpio/gpio%s/direction", index);
//	printf("---%s--\n",filename);
	fd = open(filename, O_RDWR);
	if(-1 == fd)
		return LINUX_ERROR;
	ret = write(fd, direction, strlen(direction));
	if(ret == -1 || ret == 0)
		return LINUX_ERROR;
	ret = close(fd);
#endif
	return ret;
}


int get_direction(char* index, char* direction)
{
int ret = 0;
#ifndef	WINNT
int fd;
char filename[64];
        

        sprintf_usv(filename, "/sys/class/gpio/gpio%s/direction", index);
        //printf("---%s--\n",filename);
        fd = open(filename, O_RDWR);
        if(-1 == fd)
                return LINUX_ERROR;
        ret = read(fd, direction, READLEN);
        if(ret == -1 || ret == 0)
                return LINUX_ERROR;
	if(direction[strlen(direction)-1] == '\n')
		direction[strlen(direction)-1] = '\0';
	ret = close(fd);
#endif
        return ret;
}

int set_value(char* index, char* value)
{
int ret = 0;
#ifndef WINNT
	int fd;
	char filename[64];
	
	sprintf_usv(filename, "/sys/class/gpio/gpio%s/value", index);
//        printf("---%s--\n",filename);
	fd = open(filename, O_RDWR);
	if(-1 == fd)
		return LINUX_ERROR;
	ret = write(fd, value, strlen(value));
	if(ret == -1 || ret == 0)
		return LINUX_ERROR;
	ret = close(fd);
#endif
	return ret;
}

int get_value(char* index, char* value)
{
int ret = 0;
#ifndef WINNT
        int fd;
        char filename[64];
        
        sprintf_usv(filename, "/sys/class/gpio/gpio%s/value", index);
        fd = open(filename, O_RDWR);
        if(-1 == fd)
                return LINUX_ERROR;
        ret = read(fd, value, READLEN);
        if(ret == -1 || ret == 0)
                return LINUX_ERROR;
	if(value[strlen(value)-1] == '\n')
		value[strlen(value)-1] = '\0';
        ret = close(fd);
#endif
        return ret;
}

