option(ENABLE_examples "Enable/disable C++ examples" OFF)

if(ENABLE_examples)
    add_subdirectory(exampleJr3)
    add_subdirectory(exampleRemoteControlBoard)
    add_subdirectory(exampleRemoteControlBoardModule)
    add_subdirectory(exampleRemoteJr3)
    add_subdirectory(exampleTrajectory)
endif()
