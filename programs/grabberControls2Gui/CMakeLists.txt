if(NOT ("${PYTHON_VERSION_STRING}" VERSION_EQUAL 2.7 OR "${PYTHON_VERSION_STRING}" VERSION_GREATER 3.4) AND (NOT DEFINED ENABLE_grabbercontrols2gui OR ENABLE_grabbercontrols2gui))
    message(WARNING "PythonInterp package not found or unsupported version (only 2.7, 3.5+), disabling grabbercontrols2gui program")
endif()

cmake_dependent_option(ENABLE_grabbercontrols2gui "Enable/disable grabbercontrols2gui program" ON
                       "PYTHONINTERP_FOUND;PYTHON_VERSION_STRING VERSION_EQUAL 2.7 OR PYTHON_VERSION_STRING VERSION_GREATER 3.4" OFF)

if(ENABLE_grabbercontrols2gui)

    # configure grabbercontrols2gui setup.py
    configure_file("${CMAKE_SOURCE_DIR}/cmake/templates/setup.py.in"
                   "${CMAKE_BINARY_DIR}/setup.py")

    # run setup.py (install rule)
    install(CODE "EXECUTE_PROCESS(COMMAND python3 \"${CMAKE_BINARY_DIR}/setup.py\" install)")

else()

    set(ENABLE_grabbercontrols2gui OFF CACHE BOOL "Enable/disable grabbercontrols2gui program" FORCE)

endif()
