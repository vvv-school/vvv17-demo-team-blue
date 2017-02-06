// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>
#include <yarp/os/PortReport.h>
#include <yarp/os/PortInfo.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>


class ObjectRetriever : yarp::os::PortReport
{
    bool simulation;
    yarp::os::RpcClient portLocation;
    yarp::os::RpcClient portCalibration;
    virtual void report(const yarp::os::PortInfo &info);
    bool calibrate(yarp::sig::Vector &location, const std::string &hand);

public:
    ObjectRetriever();
    bool getLocation(yarp::sig::Vector &location, const std::string &hand="dummy");
    virtual ~ObjectRetriever();
};
