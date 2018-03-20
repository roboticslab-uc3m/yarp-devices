find_package(PythonInterp 3.0 QUIET)

if("${PYTHON_VERSION_STRING}" VERSION_GREATER 3.4 AND (NOT DEFINED ENABLE_grabbercontrols2gui OR ENABLE_grabbercontrols2gui))
    message(WARNING "PythonInterp package not found or unsupported version (>3.4), disabling grabbercontrols2gui program")
endif()

cmake_dependent_option(ENABLE_grabbercontrols2gui "Enable/disable grabbercontrols2gui program" ON
                       "PYTHONINTERP_FOUND;NOT PYTHON_VERSION_STRING VERSION_GREATER 3.4" OFF)

if(ENABLE_grabbercontrols2gui)

    # configure grabbercontrols2gui setup.py
    configure_file("${CMAKE_SOURCE_DIR}/cmake/templates/setup.py.in"
                   "${CMAKE_BINARY_DIR}/setup.py")

    # run setup.py (install rule)
    install(CODE "EXECUTE_PROCESS(COMMAND python3 \"${CMAKE_BINARY_DIR}/setup.py\" install)")

else()

    set(ENABLE_grabbercontrols2gui OFF CACHE BOOL "Enable/disable grabbercontrols2gui program" FORCE)

endif()
