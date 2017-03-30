#include "AravisGigE.hpp"

bool roboticslab::AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
{
    CD_DEBUG("I was asked for an image, but I don't have any since I'm not implemented yet\n");
    return false; //-- Not implemented yet
}

int roboticslab::AravisGigE::height() const
{
    return 0; //-- Not implemented yet
}

int roboticslab::AravisGigE::width() const
{
    return 0; //-- Not implemented yet
}
