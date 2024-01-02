#include "LogComponent.hpp"

YARP_LOG_COMPONENT(JR3M, "rl.Jr3Mbed")

YARP_LOG_COMPONENT(JR3M_QUIET, "rl.Jr3Mbed",
                               yarp::os::Log::minimumPrintLevel(),
                               yarp::os::Log::minimumForwardLevel(),
                               nullptr, // disable print
                               nullptr) // disable forward
