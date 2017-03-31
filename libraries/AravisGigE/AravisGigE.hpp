#ifndef __ARAVIS_GIGE__
#define __ARAVIS_GIGE__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/AudioVisualInterfaces.h>

#include <arv.h>

#include "ColorDebug.hpp"

namespace roboticslab {

 /**
 * @ingroup AravisGigE
 * @brief Implementation for GigE cameras using Aravis as driver.
 *
 */

class AravisGigE : public yarp::dev::DeviceDriver, public yarp::dev::IFrameGrabberImage
{
    public:

        AravisGigE() {}

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IFrameGrabberImage Declarations. Implementation in IFrameGrabberImageImpl.cpp ---------
        virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
        virtual int height() const;
        virtual int width() const;

    private:
        ArvCamera       *camera;                // Camera to control.
        ArvStream       *stream;                // Object for video stream reception.
        void *framebuffer; //
};


}
#endif // __ARAVIS_GIGE__
