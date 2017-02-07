// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class gazeControlModule: public RFModule
{
protected:
    PolyDriver drvGaze;
    IGazeControl      *igaze;

    BufferedPort<Bottle> stateInput;

    RpcServer rpcPort;

    bool ok_look_down ;

    /***************************************************/
    void look_down()
    {
        Vector ang(3,0.0);
        ang[1] = -40.0 ; //elevation [deg]
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();

        // set trajectory time
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.4); // Faster than the neck

        ok_look_down = true ;
    }

    /***************************************************/
    void look_up()
    {
        Vector ang(3,0.0);
        ang[1] = 0.0 ; //elevation [deg]
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();

        // set trajectory time
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.4); // Faster than the neck

        ok_look_down = false ;
    }


public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        // FILL IN THE CODE
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!drvGaze.open(optGaze)) // open Gaze interface (no solver and controller component here)
        {
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }

        // open the view
        drvGaze.view(igaze);

        stateInput.open("/gaze-control/state:i");

        rpcPort.open("/service");
        attach(rpcPort);

        ok_look_down = false ;

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        stateInput.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvGaze.close();
        stateInput.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply) // list of instruction that are required to implemet
    {
        string cmd=command.get(0).asString();

        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- look_up");
            reply.addString("- quit");
        }
        else if (cmd=="look_up")
        {
          look_up();
          if (!ok_look_down){ reply.addString("Yep! I'm looking up now!"); }
        }
        else if (cmd=="look_down")
        {
          look_down();
          if (ok_look_down) { reply.addString("Yep! I'm looking down now!"); }
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images - the update module continuously update: this happens when you do read true
    }

    /***************************************************/
    bool updateModule()
    {
        // get states
        String *state = stateInput.read();

        if (state==NULL)
           return false;

        return true;
    }
};


/***************************************************/
int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    gazeControlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}
