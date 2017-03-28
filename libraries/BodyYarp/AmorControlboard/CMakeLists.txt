# Copyright: (C) 2017 Universidad Carlos III de Madrid
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(AmorControlboard
                    CATEGORY device
                    TYPE roboticslab::AmorControlboard
                    INCLUDE AmorControlboard.hpp
                    WRAPPER controlboardwrapper2)

if(NOT SKIP_AmorControlboard)
  find_package(AMOR_API REQUIRED)

  include_directories(${YARP_INCLUDE_DIRS}
                      ${AMOR_API_INCLUDE_DIRS}
                      ${COLOR_DEBUG_INCLUDE_DIRS}
                      ${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(AmorControlboard AmorControlboard.cpp
                                   AmorControlboard.hpp
                                   DeviceDriverImpl.cpp
                                   IControlLimitsImpl.cpp
                                   IControlModeImpl.cpp
                                   IEncodersImpl.cpp
                                   IEncodersTimedImpl.cpp
                                   IPositionControl2Impl.cpp
                                   IPositionControlImpl.cpp
                                   IVelocityControl2Impl.cpp
                                   IVelocityControlImpl.cpp)

  target_link_libraries(AmorControlboard ${YARP_LIBRARIES} ${AMOR_API_LIBRARIES})

  install(TARGETS AmorControlboard DESTINATION lib)

  yarp_install(TARGETS AmorControlboard
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

  yarp_install(FILES AmorControlboard.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
