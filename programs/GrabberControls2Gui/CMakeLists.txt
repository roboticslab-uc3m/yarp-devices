option(ENABLE_grabbercontrols2gui "Enable/disable grabbercontrols2gui program" OFF)

if(ENABLE_grabbercontrols2gui)

# configure grabbercontrols2gui setup.py
configure_file("${CMAKE_SOURCE_DIR}/cmake/template/setup.py.in"
               "${CMAKE_BINARY_DIR}/setup.py")

# run setup.py (install rule)
install(CODE "EXECUTE_PROCESS(COMMAND python3 \"${CMAKE_BINARY_DIR}/setup.py\" install)")

endif()
