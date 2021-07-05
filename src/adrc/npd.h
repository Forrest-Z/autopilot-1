#pragma once

#include <stdint.h>

class NpdConf;

class NPD{
public:
    NPD() = default;
    virtual ~NPD()=default;

    void Init(const NpdConf &conf);

    void SetNpd(const NpdConf &conf);

    void Reset();

    double Update(double e,double de);
    
private:
    double r_;
    double h_;
    double c_;
};