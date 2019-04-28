if(NOT Python_FOUND AND (NOT DEFINED ENABLE_sensorReader OR ENABLE_sensorReader))
    message(WARNING "Python interpreter not found, disabling sensorReader program")
endif()

cmake_dependent_option(ENABLE_sensorReader "Enable/disable sensorReader program" ON
                       Python_FOUND OFF)

if(ENABLE_sensorReader)

    install(PROGRAMS sensorReader.py
            DESTINATION ${CMAKE_INSTALL_BINDIR}
            RENAME sensorReader)

else()

    set(ENABLE_sensorReader OFF CACHE BOOL "Enable/disable sensorReader program" FORCE)

endif()
