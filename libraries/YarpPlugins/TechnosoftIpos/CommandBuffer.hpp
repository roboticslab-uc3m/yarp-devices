// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <mutex>

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief A buffer for periodic commands implementing linear interpolation.
 */
class CommandBuffer
{
public:
    void accept(double command);
    double interpolate();
    double getStoredCommand(double * timestamp = nullptr) const;
    void reset(double initialCommand);

private:
    double storedCommand {0.0};
    double interpolationResult {0.0};
    double commandPeriod {0.0};
    double commandTimestamp {0.0};
    double interpolationTimestamp {0.0};
    mutable std::mutex mutex;
};

} // namespace roboticslab

#endif // __COMMAND_BUFFER_HPP__
