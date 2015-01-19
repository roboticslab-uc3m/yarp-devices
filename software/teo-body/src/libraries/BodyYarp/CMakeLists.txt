# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.6)

IF (ENABLE_BodyYarp)

  option(ENABLE_testBodyYarp "Choose if you want to compile testBodyYarp" TRUE)

  find_package(YARP REQUIRED)
  include_directories(${YARP_INCLUDE_DIRS})
  list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})  
  include(YarpPlugin)

  yarp_begin_plugin_library(BodyYarp)
    add_subdirectory(motoripos)
    add_subdirectory(bodybot)
  yarp_end_plugin_library(BodyYarp)

  install(TARGETS BodyYarp DESTINATION lib)

  IF (ENABLE_testBodyYarp)

    yarp_add_plugin_library_executable(testBodyYarp BodyYarp)
    install(TARGETS testBodyYarp DESTINATION bin)

  ENDIF (ENABLE_testBodyYarp)

ENDIF (ENABLE_BodyYarp)

