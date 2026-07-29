# Try to find Aravis
# Once done this will define:
#  ARAVIS_FOUND - System has Aravis
#  ARAVIS_INCLUDE_DIRS - The Aravis include directories
#  ARAVIS_LIBRARIES - The libraries needed to use Aravis
#  ARAVIS_LIBRARY_DIRS - The libraries library directories
#  ARAVIS_VERSION - The version of Aravis package

find_package(PkgConfig)
pkg_search_module(ARAVIS aravis>=0.8 aravis-0.8)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ARAVIS REQUIRED_VARS ARAVIS_INCLUDE_DIRS ARAVIS_LIBRARIES ARAVIS_LIBRARY_DIRS
                                         VERSION_VAR ARAVIS_VERSION)

include(CMakeFindDependencyMacro)
find_dependency(GLib)

list(APPEND ARAVIS_INCLUDE_DIRS ${GLib_INCLUDE_DIRS})
list(APPEND ARAVIS_LIBRARIES ${GLib_LIBRARY})
