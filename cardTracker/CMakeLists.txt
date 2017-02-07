# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)
project(cardTracker)


find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpIDL)
include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

set(idl ${PROJECT_NAME}.thrift)
set(doc ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc})

include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

yarp_install(FILES ${doc} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

# run the python install, assumes being build in the build dir
install(CODE "execute_process(COMMAND python setup.py install -f --prefix=${CMAKE_INSTALL_PREFIX} WORKING_DIRECTORY ..)")


add_subdirectory(app)
