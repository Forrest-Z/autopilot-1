#pragma once
#include<sys/types.h>                     
#include<sys/socket.h>         
#include<sys/ioctl.h>           
#include<stdlib.h>                   
#include<netdb.h>                  
#include<arpa/inet.h>           
#include<netinet/in.h>          
#include <unistd.h>
#include <linux/sockios.h>
#include <netpacket/packet.h>
#define CloseSocket(x) close(x)
#define SOCKET int

#include <string.h>
#include <stdint.h>


class UDPDevice{
public:
	UDPDevice()
    {
        int iOpValue = 1;
        st = socket(AF_INET, SOCK_DGRAM, 0);
        setsockopt(st, SOL_SOCKET, SO_REUSEADDR,(const char *)&iOpValue, sizeof(iOpValue));
    }
	virtual ~UDPDevice(){
        close(st);
    }

public: 
	int socket_send(const char* destIP, const char* buf, int len, short int port)
    {
        int iret = 0;
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));	
        addr.sin_family = AF_INET;	
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(destIP);	

        iret = sendto(st, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
        return iret;
    }
    
	int socket_recv(char* buf, int len, char* srcIP)
    {
        
        struct sockaddr_in sendaddr;	
        memset(&sendaddr, 0, sizeof(sendaddr));
        fd_set set;
        int16_t ret;
        int16_t	iLen = 0;
        struct timeval timeout;
        //sockaddr_in from;
        socklen_t sockaddrlen;


        FD_ZERO(&set);
        FD_SET(st, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 5;
        ret = select(st + 1, &set, NULL, NULL, &timeout);
        if (ret < 0){
            return iLen;
        }
        if (FD_ISSET(st, &set)){				                																
            sockaddrlen = sizeof(sockaddr_in);
            iLen = recvfrom(st, buf, len, 0, (struct sockaddr *)&sendaddr, &sockaddrlen);
        }
        return iLen;
    }
	int socket_bind(short int port)
    {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET; 
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);	
        memset(&(addr.sin_zero), 0, 8);
        return bind(st, (struct sockaddr *)&addr, sizeof(addr)); 
    }

    private:
	SOCKET st;	
 };
