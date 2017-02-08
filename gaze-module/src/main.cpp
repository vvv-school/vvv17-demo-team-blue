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

    RpcServer port;


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

        port.open("/gaze-module/look");
        attach(port);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvGaze.close();
        port.close();
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
        // prepare input and output bottles
        Bottle cmd; // La commande
        Bottle response; // La reponse
        port.read(cmd,true);

        // get states
        string state = cmd.get(0).asString();

        if (state==NULL)
           return false;
        else if (state == "look down"){
           response.append(cmd);
           look_down() ;
           response.clear();
           response.addString("look down ok"); // 1 if look_down
        }
        else if (state == "look up"){
          response.append(cmd);
          look_up() ;
          response.clear();
          response.addString("look up ok"); // 1 if look_down           
        }

        port.reply(response); // Il envoit la reponse

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
