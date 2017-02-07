/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <string>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <map>

enum string_code {
    s_won,
    s_lost,
    s_looking_of_human,
    s_looking_at_cards,
    s_ask_for_new_card,
    s_bet,
    s_dont_bet
};


class Mood : public yarp::os::RFModule
{
public:

    Mood();
    virtual ~Mood();

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler and etc.
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
     * set the period with which updateModule() should be called
     */
    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();

    /*
    * Message handler. Just echo all received messages.
    */
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

private:
    std::map<std::string, string_code> hashit;

private:
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;    // input port
    yarp::os::BufferedPort<yarp::os::Bottle> speechOutPort;   // output port
    yarp::os::BufferedPort<yarp::os::Bottle> faceOutPort;   // output port
};

