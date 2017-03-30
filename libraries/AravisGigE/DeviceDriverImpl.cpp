#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::open(yarp::os::Searchable &config)
{
    CD_INFO("AravisGigE driver is started!\n");
    return true;
}

bool roboticslab::AravisGigE::close()
{
    CD_INFO("AravisGigE driver is closed!\n");
}
