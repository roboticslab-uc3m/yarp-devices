#include "AravisGigE.hpp"

#include <yarp/os/LogStream.h>

bool roboticslab::AravisGigE::getImage(yarp::sig::ImageOf<yarp::sig::PixelMono> &image)
{
    //-- Right now it is implemented as polling (grab + retrieve image)
    //-- I think it could be also implemented with callbacks with ArvStreamCallback

    //-- Grab frame (get raw image)
    //--------------------------------------------------------------------------------
     if (!getRawBuffer(NULL)) //-- NULL as it is stored internally in framebuffer variable
     {
         return false;
     }

    //-- Retrieve frame (convert and send as yarp image)
    //--------------------------------------------------------------------------------
    if (framebuffer!=NULL)
    {
        //-- Create a yarp image container according with the current pixel format
        yarp::sig::Image raw_image;
        if (pixelFormat != ARV_PIXEL_FORMAT_MONO_8)
        {
            yError() << "Unsupported pixel format";
        }

        //-- Write data
        image.zero();
        image.resize(_width, _height);
        mempcpy(image.getRawImage(), framebuffer, _width*_height*image.getPixelSize());
    }
    else
    {
        yError() << "Framebuffer is empty";
        return false;
    }

    return true;
}

int roboticslab::AravisGigE::height() const
{
    return _height;
}

int roboticslab::AravisGigE::width() const
{
    return _width;
}
