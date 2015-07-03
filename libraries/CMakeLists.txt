# Copyright: Universidad Carlos III de Madrid (C) 2013-2104
# Authors: Juan G. Victores

add_subdirectory(ColorDebug)  # ColorDebug sets COLOR_DEBUG_INCLUDE_DIRS

add_subdirectory(BodyYarp)  # BodyYarp a plugin library which contains YARP plugins

add_subdirectory(OneCanBusOneWrapper)
add_subdirectory(TwoCanBusThreeWrappers)

