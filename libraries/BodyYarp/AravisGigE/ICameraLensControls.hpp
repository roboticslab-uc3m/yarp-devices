#ifndef __ICAMERA_LENS_CONTROLS_HPP__
#define __ICAMERA_LENS_CONTROLS_HPP__

namespace roboticslab {

/**
* @ingroup AravisGigE
* @brief Implementation for GigE cameras using Aravis as driver.
*
*/

class ICameraLensControls
{
    public:
        //----------------- Set properties to camera ---------------------
        virtual bool setZoom(int v) = 0;
        virtual bool setFocus(int v) = 0;
        //virtual bool setIris(int v) = 0;


        //----------------- Get properties from camera ---------------------
        virtual int getZoom() = 0;
        virtual int getFocus() = 0;
        //virtual int getIris() = 0;
};

}

#endif // __ICAMERA_LENS_CONTROLS_HPP__
