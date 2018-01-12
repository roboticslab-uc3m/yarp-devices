# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico

YARP_PREPARE_PLUGIN(FakeControlboard
                    CATEGORY device
                    TYPE roboticslab::FakeControlboard
                    INCLUDE FakeControlboard.hpp
                    WRAPPER controlboardwrapper2)

if(NOT SKIP_FakeControlboard)

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs})

yarp_add_plugin(FakeControlboard FakeControlboard.cpp
                                 FakeControlboard.hpp
                                 DeviceDriverImpl.cpp
                                 IControlLimitsImpl.cpp
                                 IControlLimits2Impl.cpp
                                 IControlModeImpl.cpp
                                 IControlMode2Impl.cpp
                                 IEncodersImpl.cpp
                                 IEncodersTimedImpl.cpp
                                 IPositionImpl.cpp
                                 IPosition2Impl.cpp
                                 ITorqueImpl.cpp
                                 IVelocityImpl.cpp
                                 IVelocity2Impl.cpp
                                 SharedArea.cpp)

add_dependencies(FakeControlboard COLOR_DEBUG)

target_link_libraries(FakeControlboard YARP::YARP_OS
                                       YARP::YARP_dev)

yarp_install(TARGETS FakeControlboard
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES FakeControlboard.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
