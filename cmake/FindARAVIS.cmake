# - Try to find Aravis
# Once done this will define
#  ARAVIS_FOUND - System has Aravis
#  ARAVIS_INCLUDE_DIRS - The Aravis include directories
#  ARAVIS_LIBRARIES - The libraries needed to use Aravis
#  ARAVIS_DEFINITIONS - Compiler switches required for using Aravis

find_package(GLib REQUIRED)

# This was used for debugging purposes:
# message("--->")
# message(${GLib_INCLUDE_DIRS})
# message("--->")

find_path(ARAVIS_INCLUDE_DIR arv.h
  $ENV{ARAVIS_DIR}
  /usr/local/include/aravis-0.4
  /usr/include/aravis-0.4
)

find_library(ARAVIS_LIBRARY NAMES aravis-0.4 libaravis-0.4 )

mark_as_advanced(ARAVIS_INCLUDE_DIR ARAVIS_LIBRARY )

set(ARAVIS_INCLUDE_DIRS ${ARAVIS_INCLUDE_DIR} ${GLib_INCLUDE_DIRS})
#set(ARAVIS_LINK_DIRS ${GLib_LIBDIR} )
set(ARAVIS_LIBRARIES ${ARAVIS_LIBRARY} ${GLib_LIBRARY})
