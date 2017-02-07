// Main code + state-machine

// Author: Nolwenn

#include <string>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/IControlMode.h>

#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class Module: public RFModule
{
protected:

    RpcServer rpcPort;
    ObjectRetriever object;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        // string robot=rf.check("robot",Value("icubSim")).asString();
        //
        // // FILL IN THE CODE
        // Property optGaze;
        // optGaze.put("device","gazecontrollerclient");
        // optGaze.put("remote","/iKinGazeCtrl");
        // optGaze.put("local","/gaze_client");
        //
        // // open a client interface to connect to the joint controller // Where we say what you want to control
        // Property optJointL;
        // optJointL.put("device","remote_controlboard");
        // optJointL.put("remote","/"+robot+"/left_arm");
        // optJointL.put("local","/position/left_arm");
        //
        // Property optJointR;
        // optJointR.put("device","remote_controlboard");
        // optJointR.put("remote","/"+robot+"/right_arm");
        // optJointR.put("local","/position/right_arm");
        //
        // // "/"+robot+"/cartesianController/"+arm);
        //
        //
        // if (!drvHandL.open(optJointL))
        // {
        //     yError()<<"Unable to connect to /icubSim/left_arm";
        //     return false;
        // }
        //
        //
        // if (!drvHandR.open(optJointR))
        // {
        //     yError()<<"Unable to connect to /icubSim/right_arm";
        //     return false;
        // }
        //
        // if (!drvGaze.open(optGaze)) // open Gaze interface (no solver and controller component here)
        // {
        //     yError()<<"Unable to open the Gaze Controller";
        //     drvArmL.close(); // if something goes bad, we have to close everything
        //     drvArmR.close(); // if something goes bad, we have to close everything
        //     return false;
        // }
        //
        //
        // // save startup contexts
        // drvArmR.view(iarm);
        // iarm->storeContext(&startup_ctxt_arm_right);
        //
        // drvArmL.view(iarm);
        // iarm->storeContext(&startup_ctxt_arm_left);
        //
        // drvGaze.view(igaze);
        // igaze->storeContext(&startup_ctxt_gaze);
        //
        // rpcPort.open("/service");
        // attach(rpcPort);
        // return true;

        int score_robot = 0 ;
        int score_human = 0 ;

        bool ok_look_down = false ;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addString("Available commands:");
            reply.addString("- start");
            reply.addString("- quit");
        }
        else if (cmd=="start")
        {

            if (!ok_look_down)
            {
              look_down(); // ok_look_down has to be tuned true
              reply.addString("I look down");
            }

            cardRecognition() ; // update both scores inside

            if (score_robot > score_human){
              reply.addString("I want to bet");
              pushObject() ;
            }
            else {
              reply.addString("I don't want to bet");
            }

            // The dealer distributes cards
            if (ok_look_down){
              look_up() ; // ok_look_down has to be tuned false
              Time::delay(2.0);
              reply.addString("Waiting for the dealer to distribute");
            }

            if (!ok_look_down){
              look_down() ; // ok_look_down has to be tuned true
              reply.addString("I look down");
            }

            cardRecognition() ;  // update both scores inside

            if (score_robot > score_human){
                  pullObject() ;
                  reply.addString("I have won !");
                }
                else {
                  reply.addString("I've lost...");
                }
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/
    bool interrupt()
    {
        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /***************************************************/
    bool updateModule()
    {
        return !closing;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{

    yarp::os::Network::init();

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    Module module;
    ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
