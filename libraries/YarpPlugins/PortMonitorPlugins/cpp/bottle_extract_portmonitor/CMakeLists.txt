# Copyright: (C) 2019 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(BottleExtract
                    CATEGORY portmonitor
                    TYPE roboticslab::BottleExtract
                    INCLUDE BottleExtract.hpp
                    DEFAULT ON)

if(NOT SKIP_BottleExtract)

    yarp_add_plugin(roboticslab_pm_bottleextract)

    target_sources(roboticslab_pm_bottleextract PRIVATE BottleExtract.cpp
                                                        BottleExtract.hpp)
    target_link_libraries(roboticslab_pm_bottleextract YARP::YARP_os)

    yarp_install(TARGETS roboticslab_pm_bottleextract
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

    set_property(TARGET roboticslab_pm_bottleextract PROPERTY FOLDER "Plugins/Port Monitor")
endif()
