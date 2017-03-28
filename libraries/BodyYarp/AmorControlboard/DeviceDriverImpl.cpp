// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AmorControlboard::open(yarp::os::Searchable& config) {

    //CD_DEBUG("penv: %p\n",*((const OpenRAVE::EnvironmentBase**)(config.find("penv").asBlob())));
    /*penv = *((OpenRAVE::EnvironmentBase**)(config.find("penv").asBlob()));

    int robotIndex = config.check("robotIndex",-1,"robotIndex").asInt();
    if( robotIndex < 0 )  // a.k.a. -1 one line above
    {
        CD_ERROR("Please review robotIndex.\n");
        return false;
    }
    int manipulatorIndex = config.check("manipulatorIndex",-1,"manipulatorIndex").asInt();
    if( manipulatorIndex < 0 )  // a.k.a. -1 one line above
    {
        CD_ERROR("Please review manipulatorIndex.\n");
        return false;
    }

    std::vector<OpenRAVE::RobotBasePtr> vectorOfRobotPtr;
    penv->GetRobots(vectorOfRobotPtr);
    probot = vectorOfRobotPtr[robotIndex];

    dEncRaw.resize( probot->GetDOF() );
    std::fill(dEncRaw.begin(), dEncRaw.end(), 0);

    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> vectorOfManipulatorPtr = probot->GetManipulators();
    manipulatorIDs = vectorOfManipulatorPtr[manipulatorIndex]->GetArmIndices();*/

    int major, minor, build;
    amor_get_library_version(&major, &minor, &build);
    printf("AMOR API library version %d.%d.%d\n\n", major, minor, build);

    std::cout << "Trying to connect to AMOR... ";

#ifdef WIN32
    handle = amor_connect("canlib_esd.dll", 0);
#else
    handle = amor_connect("libeddriver.so", 0);
#endif // UNIX

    if(handle == AMOR_INVALID_HANDLE) {
        std::cout << amor_error() << "[FAILED]" << std::endl;
        close();
        return false;
    }

    axes = 7;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::close() {
    printf("[AmorControlboard] close()\n");
    return true;
}

// -----------------------------------------------------------------------------
