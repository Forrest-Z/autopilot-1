#include "conf.h"

// Static variables
Conf *Conf::singleton_;

Conf::Conf()
{
    singleton_ = this;
}

namespace AP{
    Conf *conf()
    {
       return Conf::get_singleton();
    }
};