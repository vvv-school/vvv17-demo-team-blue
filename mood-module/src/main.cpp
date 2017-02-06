/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <Codec.h>

using namespace yarp::os;


int main(int argc, char * argv[])
{
    Network yarp;

    Codec module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("tutorial_RFModule");
    //rf.setDefaultConfigFile("tutorial_RFModule.ini");
    // rf.setVerbose(true);

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}


