#pragma once

#include <string>
#include <memory>
#include "udp_device.h"

class Console{
public:
    Console();
    virtual ~Console()=default;

    void Init(std::string &remote_ip,std::string &local_ip,
    uint16_t remote_port,uint16_t local_port);

    // Do not allow copies
    Console(const Console &other) = delete;
    Console &operator=(const Console&) = delete;

    // Get singleton instance
    static Console *get_singleton()
    {
        return singleton_;
    }

    // Printf function
    void Printf(const char *fmt,...);

private:
    std::unique_ptr<UDPDevice> udp_;
    static Console *singleton_;

private:
    std::string remote_ip_;
    std::string local_ip_;
    uint16_t remote_port_;
    uint16_t local_port_;
};

namespace HAL{
    Console *console();
};