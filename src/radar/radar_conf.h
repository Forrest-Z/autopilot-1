#pragma once

#include <stdint.h>
#include <string>

class RadarConf{
public:
    RadarConf() = default;
    virtual ~RadarConf() = default;

   // Communication configuration
   // [RADAR_COMMUNICATION]
    std::string  arm_ip_;
	uint16_t     arm_port_;
	std::string  app_ip_;
	uint16_t     app_port_;
    std::uint_fast8_t window_size_;
};