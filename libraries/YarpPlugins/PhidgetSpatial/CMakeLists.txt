if(NOT Phidgets_FOUND AND (NOT DEFINED ENABLE_PhidgetSpatial OR PhidgetSpatial))
    message(WARNING "Phidgets package not found, disabling PhidgetSpatial")
endif()

yarp_prepare_plugin(PhidgetSpatial
                    CATEGORY device
                    TYPE PhidgetSpatial
                    INCLUDE PhidgetSpatial.hpp
                    DEFAULT ON
                    DEPENDS Phidgets_FOUND
                    EXTRA_CONFIG WRAPPER=multipleanalogsensorsserver)

if(NOT SKIP_PhidgetSpatial)

    yarp_add_plugin(PhidgetSpatial)

    target_sources(PhidgetSpatial PRIVATE PhidgetSpatial.hpp
                                          PhidgetSpatial.cpp
                                          DeviceDriverImpl.cpp
                                          IThreeAxisLinearAccelerometersImpl.cpp
                                          IThreeAxisGyroscopesImpl.cpp
                                          IThreeAxisMagnetometersImpl.cpp
                                          LogComponent.hpp
                                          LogComponent.cpp)

    target_link_libraries(PhidgetSpatial YARP::YARP_os
                                         YARP::YARP_dev
                                         Phidgets::Phidget21)

    yarp_install(TARGETS PhidgetSpatial
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_PhidgetSpatial OFF CACHE BOOL "Enable/disable PhidgetSpatial device" FORCE)

endif()
