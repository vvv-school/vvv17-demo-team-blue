# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(blue_slide_module)

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
# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
#add_definitions(-D_MY_DEFINITION)

# building module
include_directories(${YARP_INCLUDE_DIRS}
                     ${ctrlLib_INCLUDE_DIRS}
                     ${iKin_INCLUDE_DIRS}
                     ${perceptiveModels_INCLUDE_DIRS}
                     ${actionPrimitives_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}
               ${CMAKE_SOURCE_DIR}/src/slidingController_IDL.cpp
               ${CMAKE_SOURCE_DIR}/src/helpers.cpp
               ${CMAKE_SOURCE_DIR}/src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} actionPrimitives perceptiveModels iKin ctrlLib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB conf    ${CMAKE_SOURCE_DIR}/app/conf/*.ini)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

yarp_install(FILES DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
