# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(movement-controller)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
include_directories(${YARP_INCLUDE_DIRS}
                     ${ctrlLib_INCLUDE_DIRS}
                     ${iKin_INCLUDE_DIRS}
                     ${perceptiveModels_INCLUDE_DIRS}
                     ${actionPrimitives_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/include)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${YARP_INCLUDE_DIRS} src include)
add_executable(blue_movement_module src/main.cpp
                               src/slidingController_IDL.cpp
                               src/helpers.h
                               src/helpers.cpp)
target_link_libraries(blue_movement_module ${YARP_LIBRARIES} actionPrimitives perceptiveModels iKin ctrlLib)
install(TARGETS blue_movement_module DESTINATION bin)
set_property(TARGET blue_movement_module PROPERTY CXX_STANDARD 11)

file(GLOB conf    ${CMAKE_SOURCE_DIR}/app/conf/*.ini)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

yarp_install(FILES blue-movement-control.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
