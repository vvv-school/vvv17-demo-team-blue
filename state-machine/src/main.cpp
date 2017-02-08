// Main code + state-machine

// Author: Nolwenn

#include <string>
#include <cmath>
#include <algorithm>
#include <map>

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
    yarp::os::BufferedPort<yarp::os::Bottle> gazeInport;
    yarp::os::BufferedPort<yarp::os::Bottle> cardInport;

    bool                        closing;

    // state variables
    bool ok_look_down;
    int score_robot;
    int score_human;

    std::map<std::string, std::pair<double, double> > last_card_locations;


public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();


        if(!stateOutPort.open("/state-machine/state:o")) {
            yError()<<"Cannot open the stateOutPort";
            return false;
        }

        if(!gazeInport.open("/state-machine/look:i")) {
            yError()<<"Cannot open the gazeInport";
            return false;
        }

        if(!cardInport.open("/state-machine/card:i")) {
            yError()<<"Cannot open the cardInport";
            return false;
        }

        rpcPort.open("/state-machine/command");
        attach(rpcPort);

        int score_robot = 0 ;
        int score_human = 0 ;

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
            reply.addString("- demo");
            reply.addString("- quit");
        }
        else if (cmd=="start")
        {
            Bottle *gaze_input = gazeInport.read();
            int look_down = gaze_input->get(0).asInt();

            // Let's start !

            publishState("starting");

            //we delay a bit so that the robot can speak & ppl can marvel at it
            yarp::os::Time::delay(3.0);
            publishState("look down");

            if (look_down){
                publishState("looking at cards");
                Bottle *card_input = cardInport.read();
                readCardsAndUpdateScore(card_input);

            }
            else {
                publishState("look down");
            }

            if (score_robot > score_human){
                publishState("bet");
            }
            else {
                publishState("don't bet");
            }

            if (look_down)
            {
                publishState("look up");
            }

            // The dealer distributes cards in the meantime
            Time::delay(2.0);

            if (!look_down){
                publishState("looking at cards");
                Bottle *card_input = cardInport.read();
                readCardsAndUpdateScore(card_input);
            }

            if (score_robot > score_human){
                publishState("pull object");
                publishState("won");
            }
            else
            {
                publishState("lost");
            }
        }
        else if( cmd == "demo")
        {
            publishState("starting");
            yarp::os::Time::delay(1.0);
            publishState("look down");
            yarp::os::Time::delay(1.0);
            publishState("looking at cards");
            yarp::os::Time::delay(1.0);
            publishState("bet");
            yarp::os::Time::delay(1.0);
            publishState("don't bet");
            yarp::os::Time::delay(1.0);
            publishState("look up");
            yarp::os::Time::delay(1.0);
            publishState("looking at cards");
            yarp::os::Time::delay(1.0);
            publishState("pull object");
            yarp::os::Time::delay(1.0);
            publishState("won");
            yarp::os::Time::delay(1.0);
            publishState("lost");
            yarp::os::Time::delay(1.0);
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/

    void readCardsAndUpdateScore(Bottle* card_input)
    {

        for (int i=0; i<(card_input->size() / 5); i++)
        {
            int id = card_input->get(i+0).asInt();
            double cx = card_input->get(i+1).asDouble();
            double cy = card_input->get(i+2).asDouble();
            std::string owner = card_input->get(i+3).asString();
            int label = card_input->get(i+4).asInt();

            last_card_locations[owner] = std::pair<double, double>(cx, cy);

            if ( owner == "icub" )
            {
                score_robot += label;
                publishState(std::string("icub got ") + score_robot + " total score");
            }
            else if ( owner == "human" )
            {
                score_human += label;
                publishState(std::string("human got ") + score_human + " total score");

            }
            else
            {
                //wtf
                publishState("unknown user field in card message: " + owner);
            }
        }
    }

    /***************************************************/
    bool interrupt()
    {
        stateOutPort.interrupt();
        gazeInport.interrupt();
        cardInport.interrupt();
        return true;
    }

    /**********************************************************/
    bool close()
    {
        rpcPort.close();
        stateOutPort.close();
        gazeInport.close();
        cardInport.close();
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

    /***************************************************/

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
