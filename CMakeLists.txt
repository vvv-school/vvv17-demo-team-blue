cmake_minimum_required(VERSION 2.8.9)
project(blue_demo)

find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
#include(ICUBcontribOptions)


add_subdirectory(duck-detector)
add_subdirectory(mood-module)
add_subdirectory(movement-control)
add_subdirectory(gaze-module)
add_subdirectory(state-machine)

icubcontrib_add_uninstall_target()

yarp_install(FILES blue_card_demo.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

message("************************************* CARDTRACKER NEEDS SEPARATE BUILD ***************************")
