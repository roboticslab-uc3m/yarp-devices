# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(WiimoteSensor
                    CATEGORY device
                    TYPE roboticslab::WiimoteSensor
                    INCLUDE WiimoteSensor.hpp
                    DEFAULT ON)

if(NOT SKIP_WiimoteSensor)

find_package(XWiimote 0.3 REQUIRED)

if(XWIIMOTE_VERSION VERSION_LESS "2")
    add_definitions(-DXWIIMOTE_LEGACY_INTERFACE)
endif()

get_property(_common_dirs GLOBAL PROPERTY ROBOTICSLAB_YARP_DEVICES_INCLUDE_DIRS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${_common_dirs}
                    ${XWIIMOTE_INCLUDE_DIRS})

yarp_add_plugin(WiimoteSensor
          WiimoteSensor.hpp
          WiimoteSensor.cpp
          DeviceDriverImpl.cpp
          IAnalogSensorImpl.cpp
          WiimoteDispatcherThread.cpp)

add_dependencies(WiimoteSensor COLOR_DEBUG)

target_link_libraries(WiimoteSensor ${YARP_LIBRARIES}
                                    ${XWIIMOTE_LIBRARIES})

yarp_install(TARGETS WiimoteSensor
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES WiimoteSensor.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
