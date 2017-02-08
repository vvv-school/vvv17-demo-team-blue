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
    yarp::os::BufferedPort<yarp::os::Bottle> stateOutPort;

    bool                        closing;

    // state variables
    bool ok_look_down;
    int score_robot;
    int score_human;


public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();

        bool ret = true;

        ret &= stateOutPort.open("/state-machine/state:o");
        if(!ret) {
            yError()<<"Cannot open some of the ports";
            return false;
        }

        rpcPort.open("/state-machine/command");
        attach(rpcPort);


        int score_robot = 0 ;
        int score_human = 0 ;

        bool ok_look_down = false ;

        return true;
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
            publishState("starting");

            //we delay a bit so that the robot can speak & ppl can marvel at it
            yarp::os::Time::delay(3.0);

            look_down(); // ok_look_down has to be tuned true

            updateScoresFromCards() ; // update both scores inside

            play();

            // The dealer distributes cards
            look_up() ; // ok_look_down has to be tuned false
            Time::delay(2.0);

// ask for card
            look_down() ; // ok_look_down has to be tuned true
            publishState("looking at cards");

            updateScoresFromCards() ;  // update both scores inside

            if (score_robot > score_human){
                pullObject() ;
                publishState("won");
            }
            else
            {
                publishState("lost");
            }
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    void pullObject()
    {
        // FILL IN HERE
    }

    void updateScoresFromCards()
    {
        publishState("looking at cards");
        // FILL IN HERE
        // check that we have card data
        // set score_robot and score_human


    }

    void look_down()
    {
        publishState("looking at cards");

        // FILL IN HERE
    }

    void look_up()
    {
        publishState("look up");

        // FILL IN HERE
    }

    void play()
    {
        if (score_robot > score_human)
        {
            publishState("bet");
            pushObject() ;
        }
        else
        {
            publishState("don't bet");
        }

    }

    void pushObject()
    {
        // FILL IN HERE
    }



    /***************************************************/
    bool interrupt()
    {
        return true;
    }

    /**********************************************************/
    bool close()
    {
        // we have to close ports here
        rpcPort.close();
        stateOutPort.close();
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

    void publishState(const std::string& state)
    {
        Bottle& output = stateOutPort.prepare();
        output.clear();
        output.addString(state.c_str());
        stateOutPort.write();
        yarp::os::Time::delay(1.0);
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
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
