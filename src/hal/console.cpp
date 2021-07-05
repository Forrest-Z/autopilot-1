#include "console.hpp"
#include <stdarg.h>
#include <iostream>

 Console *Console::singleton_;


Console::Console()
{
    singleton_ = this;

    udp_ = std::unique_ptr<UDPDevice>(new UDPDevice());
}

void Console::Init(std::string &remote_ip,std::string &local_ip,
    uint16_t remote_port,uint16_t local_port)
    {
        remote_ip_ = remote_ip;
        local_ip_ = local_ip;
        remote_port_ = remote_port;
        local_port_ = local_port;

         udp_->socket_bind(remote_port_);
    }


void Console::Printf(const char *fmt,...)
{
    // va_list arg_list;
    // va_start(arg_list, fmt);
    // send_textv(severity, fmt, arg_list);
    // va_end(arg_list);
    udp_->socket_send(remote_ip_.c_str(),fmt,sizeof(fmt),remote_port_);
}


namespace HAL{
    Console *console()
    {
        return Console::get_singleton();
    }
}