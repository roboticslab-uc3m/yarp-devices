# Copyright: Universidad Carlos III de Madrid (C) 2013-2104
# Authors: Juan G. Victores & Raul de Santos Rico

add_subdirectory(ColorDebug)  # ColorDebug sets COLOR_DEBUG_INCLUDE_DIRS

add_subdirectory(YarpPlugins)  # Old YarpPlugins, a plugin library which contains YARP plugins

add_subdirectory(OneCanBusOneWrapper)
add_subdirectory(TwoCanBusThreeWrappers)

add_subdirectory(AravisGigE) # Aravis driver for GigE cameras
