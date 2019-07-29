// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_TECHNOSOFT_IPOS__
#define __I_TECHNOSOFT_IPOS__

namespace roboticslab
{

/**
 *
 * @brief Abstract base for a TechnosoftIpos.
 *
 */
class ITechnosoftIpos
{
public:
    /**
     * Destructor.
     */
    virtual ~ITechnosoftIpos() {}

    /** reset node */
    virtual bool resetNode(int id) = 0;
    /** reset all nodes */
    virtual bool resetNodes() = 0;
    /** send new point to PT/PVT buffer */
    virtual bool sendLinearInterpolationTarget() = 0;
    /** send start signal to PT/PVT mode */
    virtual bool sendLinearInterpolationStart() = 0;
};

}  // namespace roboticslab

#endif  //  __I_TECHNOSOFT_IPOS__
