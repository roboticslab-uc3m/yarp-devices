// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <algorithm> // std::find

#include <yarp/os/LogStream.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

// ------------------- IControlMode Related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getAvailableControlModes(int j, std::vector<yarp::dev::SelectableControlModeEnum> & avail)
{
    CHECK_JOINT(j);

    avail = {
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_VELOCITY,
        yarp::dev::SelectableControlModeEnum::VOCAB_CM_POSITION_DIRECT
    };

    return yarp::dev::ReturnValue_ok;
}
#endif

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getControlMode(int j, yarp::dev::ControlModeEnum & mode)
#else
bool EmulatedControlBoard::getControlMode(int j, int * mode)
#endif
{
    CHECK_JOINT(j);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    mode = static_cast<yarp::dev::ControlModeEnum>(controlMode);
    return yarp::dev::ReturnValue_ok;
#else
    *mode = controlMode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getControlModes(std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool EmulatedControlBoard::getControlModes(int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(m_axes);
#endif

    for (unsigned int i = 0; i < m_axes; i++)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        ok &= getControlMode(i, modes[i]);
#else
        ok &= getControlMode(i, &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getControlModes(const std::vector<int> & joints, std::vector<yarp::dev::ControlModeEnum> & modes)
#else
bool EmulatedControlBoard::getControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    modes.resize(joints.size());

    for (size_t i = 0; i < joints.size(); i++)
    {
        ok &= getControlMode(joints[i], modes[i]);
#else
    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlMode(int j, yarp::dev::SelectableControlModeEnum mode)
#else
bool EmulatedControlBoard::setControlMode(int j, int mode)
#endif
{
    CHECK_JOINT(j);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    std::vector<yarp::dev::SelectableControlModeEnum> availableModes;
    getAvailableControlModes(j, availableModes);

    if (std::find(availableModes.begin(), availableModes.end(), mode) == availableModes.end())
    {
        yCError(ECB) << "Requested control mode not available for joint:" << j;
        return yarp::dev::ReturnValue_error_method_failed;
    }
#else
    if (mode != VOCAB_CM_POSITION && mode != VOCAB_CM_VELOCITY && mode != VOCAB_CM_POSITION_DIRECT)
    {
        yCError(ECB) << "Requested control mode not available for joint:" << j;
        return false;
    }
#endif

    if (!stop(j))
    {
        yCError(ECB) << "Failed to stop joint before changing control mode:" << j;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    controlMode = static_cast<yarp::conf::vocab32_t>(mode);
    return yarp::dev::ReturnValue_ok;
#else
    controlMode = mode;
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlModes(const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool EmulatedControlBoard::setControlModes(int * modes)
#endif
{
    bool ok = true;

    for (unsigned int i = 0; i < m_axes; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setControlModes(const std::vector<int> & joints, const std::vector<yarp::dev::SelectableControlModeEnum> & modes)
#else
bool EmulatedControlBoard::setControlModes(int n_joint, const int * joints, int * modes)
#endif
{
    bool ok = true;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    for (int j = 0; j < joints.size(); j++)
#else
    for (int j = 0; j < n_joint; j++)
#endif
    {
        ok &= setControlMode(joints[j], modes[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok ? yarp::dev::ReturnValue_ok : yarp::dev::ReturnValue_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
