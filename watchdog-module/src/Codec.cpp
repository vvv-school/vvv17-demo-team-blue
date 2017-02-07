/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <Codec.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


Codec::Codec() { }

Codec::~Codec() { }


bool Codec::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the codedc module...";

    // open all ports
    bool ret = true;//commandPort.open("/Codec/rpc");
    ret &= skinInPort.open("/blue_watchdog/skin:i");
    ret &= alertOutPort.open("/blue_watchdog/alert:o");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    if(!attach(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // set some paramters
//    modeParam = rf.check("mode", Value("coder")).asString();
//    if((modeParam != "coder") && (modeParam !="decoder")) {
//        yError()<<"Unknown mode value "<<modeParam;
//        return false;
//    }

    // everything is fine
    return true;
}


double Codec::getPeriod() {
    return 0.01; // module periodicity (seconds)
}


bool Codec::updateModule() {

    Bottle* input = skinInPort.read();
    if(input==NULL)
        return false;
    std::string data;

    Bottle& output = alertOutPort.prepare();
    output.clear();
    output.addString(data.c_str());
    alertOutPort.write();
    return true;
}


bool Codec::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else {
        reply.clear();
        reply.addString("error");
    }
    return true;
}


bool Codec::interruptModule() {

    skinInPort.interrupt();
    return true;
}


bool Codec::close() {
    skinInPort.close();
    // you can force writing remaining data
    // using outPort.writeStrict();
    alertOutPort.close();
    return true;
}
