option(ENABLE_examples "Enable/disable C++ examples" OFF)

if(ENABLE_examples)
    add_subdirectory(exampleJr3)
    add_subdirectory(examplePositionDirect)
    add_subdirectory(exampleRemoteControlboard)
    add_subdirectory(exampleRemoteControlboardModule)
    add_subdirectory(exampleRemoteJr3)
endif()
