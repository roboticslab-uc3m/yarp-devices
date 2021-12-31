#ifndef __ARAVIS_GIGE_HPP__
#define __ARAVIS_GIGE_HPP__

#include <map>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberControls.h>
#include <yarp/dev/IFrameGrabberImageRaw.h>

#include <arv.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup AravisGigE
 * @brief Contains roboticslab::AravisGigE.
 */

 /**
  * @ingroup AravisGigE
  * @brief Implementation for GigE cameras using Aravis as driver.
  */
class AravisGigE : public yarp::dev::DeviceDriver,
                   public yarp::dev::IFrameGrabberImageRaw,
                   public yarp::dev::IFrameGrabberControls
{
public:

    ~AravisGigE() override { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //  --------- IFrameGrabberImageRaw Declarations. Implementation in IFrameGrabberImageRawImpl.cpp ---------
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image) override;
    int height() const override;
    int width() const override;

    // ---------- IFrameGrabberControls Declarations. Implementation in IFrameGrabberControlsImpl.cpp ---------
    bool getCameraDescription(CameraDescriptor *camera) override;
    bool hasFeature(int feature, bool *hasFeature) override;
    bool setFeature(int feature, double value) override;
    bool getFeature(int feature, double *value) override;
    bool setFeature(int feature, double value1, double value2) override;
    bool getFeature(int feature, double *value1, double *value2) override;
    bool hasOnOff(int feature, bool *HasOnOff) override;
    bool setActive(int feature, bool onoff) override;
    bool getActive(int feature, bool *isActive) override;
    bool hasAuto(int feature, bool *hasAuto) override;
    bool hasManual(int feature, bool *hasManual) override;
    bool hasOnePush(int feature, bool *hasOnePush) override;
    bool setMode(int feature, FeatureMode mode) override;
    bool getMode(int feature, FeatureMode *mode) override;
    bool setOnePush(int feature) override;

private:
    ArvCamera       *camera;                // Camera to control.
    ArvStream       *stream;                // Object for video stream reception.
    void            *framebuffer;           //

    unsigned int    payload;                // Width x height x Pixel width.

    int             widthMin;               // Camera sensor minium width.
    int             widthMax;               // Camera sensor maximum width.
    int             heightMin;              // Camera sensor minium height.
    int             heightMax;              // Camera sensor maximum height.
    bool            fpsAvailable;
    double          fpsMin;                 // Camera minium fps.
    double          fpsMax;                 // Camera maximum fps.
    bool            gainAvailable;
    double          gainMin;                // Camera minimum gain.
    double          gainMax;                // Camera maximum gain.
    bool            exposureAvailable;
    double          exposureMin;            // Camera's minimum exposure time.
    double          exposureMax;            // Camera's maximum exposure time.

    bool            controlExposure;        // Flag if automatic exposure shall be done by this SW
    bool            autoGain;
    double          targetGrey;             // Target grey value (mid grey))

    gint64          *pixelFormats;
    guint           pixelFormatsCnt;


    int             num_buffers;            // number of payload transmission buffers

    ArvPixelFormat  pixelFormat;            // pixel format

    int             xoffset;                // current frame region x offset
    int             yoffset;                // current frame region y offset
    int             _width;                  // current frame width of frame
    int             _height;                 // current frame height of image

    double          fps;                    // current value of fps
    double          exposure;               // current value of exposure time
    double          gain;                   // current value of gain
    double          midGrey;                // current value of mid grey (brightness)

    unsigned        frameID;                // current frame id
    unsigned        prevFrameID;

    //-- Lens Controls
    bool zoomAvailable;
    gint64 zoomMin;                            // Camera zoom minimum value
    gint64 zoomMax;                            // Camera zoom maximum value
    bool focusAvailable;
    gint64 focusMin;                           // Camera focus minimum value
    gint64 focusMax;                           // Camera focus maximum value
    bool irisAvailable;
    gint64 irisMin;                            // Camera iris minimum value
    gint64 irisMax;                            // Camera iris maximum value

    //-- IFrameGrabberControls2
    std::map<cameraFeature_id_t, const char*> yarp_arv_int_feature_map; //-- Map yarp features with aravis feature ids
    std::map<cameraFeature_id_t, const char*> yarp_arv_float_feat_map;
};

}

#endif // __ARAVIS_GIGE_HPP__
