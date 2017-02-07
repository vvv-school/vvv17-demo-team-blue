// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
ObjectRetriever::ObjectRetriever() : simulation(false)
{
    portLocation.open("/location");
    portCalibration.open("/calibration");

    portLocation.asPort().setTimeout(1.0);
    portCalibration.asPort().setTimeout(1.0);

    portLocation.setReporter(*this);
}

/***************************************************/
ObjectRetriever::~ObjectRetriever()
{
    portLocation.close();
    portCalibration.close();
}

/***************************************************/
void ObjectRetriever::report(const PortInfo &info)
{
    if (info.created && !info.incoming)
    {
        simulation=(info.targetName=="/icubSim/world");
        yInfo()<<"We are talking to "<<(simulation?"icubSim":"icub");
    }
}

/***************************************************/
bool ObjectRetriever::calibrate(Vector &location,
                                const string &hand)
{
    if ((portCalibration.getOutputCount()>0) &&
        (location.length()>=3))
    {
        Bottle cmd,reply;
        cmd.addString("get_location_nolook");
        cmd.addString("iol-"+hand);
        cmd.addDouble(location[0]);
        cmd.addDouble(location[1]);
        cmd.addDouble(location[2]);
        portCalibration.write(cmd,reply);

        location.resize(3);
        location[0]=reply.get(1).asDouble();
        location[1]=reply.get(2).asDouble();
        location[2]=reply.get(3).asDouble();
        return true;
    }

    return false;
}

/***************************************************/
bool ObjectRetriever::getLocation(Vector &location,
                                  const string &hand)
{
    if (portLocation.getOutputCount()>0)
    {
        Bottle cmd,reply;
        if (simulation)
        {
            cmd.addString("world");
            cmd.addString("get");
            cmd.addString("ball");
            if (portLocation.write(cmd,reply))
            {
                if (reply.size()>=3)
                {
                    location.resize(3);
                    location[0]=reply.get(0).asDouble();
                    location[1]=reply.get(1).asDouble();
                    location[2]=reply.get(2).asDouble();

                    // compute ball position in robot's root frame
                    Matrix T=zeros(4,4);
                    T(0,1)=-1.0;
                    T(1,2)=1.0;  T(1,3)=0.5976;
                    T(2,0)=-1.0; T(2,3)=-0.026;
                    T(3,3)=1.0;
                    location.push_back(1.0);
                    location=SE3inv(T)*location;
                    location.pop_back();
                    location[2]+=0.05;  // safe margin
                    return true;
                }
            }
        }
        else
        {
            cmd.addVocab(Vocab::encode("ask"));
            Bottle &content=cmd.addList().addList();
            content.addString("name");
            content.addString("==");
            content.addString("Toy");
            portLocation.write(cmd,reply);

            if (reply.size()>1)
            {
                if (reply.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idField=reply.get(1).asList())
                    {
                        if (Bottle *idValues=idField->get(1).asList())
                        {
                            int id=idValues->get(0).asInt();

                            cmd.clear();
                            cmd.addVocab(Vocab::encode("get"));
                            Bottle &content=cmd.addList();
                            Bottle &list_bid=content.addList();
                            list_bid.addString("id");
                            list_bid.addInt(id);
                            Bottle &list_propSet=content.addList();
                            list_propSet.addString("propSet");
                            Bottle &list_items=list_propSet.addList();
                            list_items.addString("position_3d");
                            Bottle replyProp;
                            portLocation.write(cmd,replyProp);

                            if (replyProp.get(0).asVocab()==Vocab::encode("ack"))
                            {
                                if (Bottle *propField=replyProp.get(1).asList())
                                {
                                    if (Bottle *position_3d=propField->find("position_3d").asList())
                                    {
                                        if (position_3d->size()>=3)
                                        {
                                            location.resize(3);
                                            location[0]=position_3d->get(0).asDouble();
                                            location[1]=position_3d->get(1).asDouble();
                                            location[2]=position_3d->get(2).asDouble();
                                            if (calibrate(location,hand))
                                                return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    yError()<<"Unable to retrieve location";
    return false;
}
