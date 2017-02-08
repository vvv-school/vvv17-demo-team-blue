// Main code + state-machine

// Author: Nolwenn

#include <string>
#include <cmath>
#include <stdlib.h>
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


struct Position3D
{
    double x,y,z;
    Position3D()
     : x(0) , y(0) , z(0) { }
    Position3D(double x_, double y_, double z_ = 0.0)
     : x(x_) , y(y_) , z(z_) { }
};

/***************************************************/
class Module: public RFModule
{
protected:

    RpcServer rpcPort;
    RpcClient movementPort;
    RpcClient gazePort;

    yarp::os::BufferedPort<yarp::os::Bottle> stateOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> cardInport;
    yarp::os::BufferedPort<yarp::os::Bottle> duckInport;

    bool  closing;

    // state variables
    int score_robot;
    int score_human;

    std::map<std::string, Position3D > last_card_locations;

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();


        if(!stateOutPort.open("/state-machine/state:o")) {
            yError()<<"Cannot open the stateOutPort";
            return false;
        }

        if(!cardInport.open("/state-machine/card:i")) {
            yError()<<"Cannot open the cardInport";
            return false;
        }

        if(!duckInport.open("/state-machine/duck:i")) {
            yError()<<"Cannot open the cardInport";
            return false;
        }

        rpcPort.open("/state-machine/command");
        attach(rpcPort);

        movementPort.open("/state-machine/position");

        gazePort.open("/state-machine/look");

        score_robot = 0 ;
        score_human = 0 ;

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
            // Let's start !
            string currentGaze ;

            publishState("starting");

            //we delay a bit so that the robot can speak & ppl can marvel at it
            yarp::os::Time::delay(3.0);
            publishState("look down");
            currentGaze = gazeState("look down");

            if (currentGaze == "look down ok"){
                publishState("looking at cards");
                Bottle *card_input = cardInport.read();
                readCardsAndUpdateScore(card_input);
            }
            else
            {
                publishState("look down");
                currentGaze = gazeState("look down");
            }

            if (score_robot > score_human){
                publishState("ask duck position");
                Bottle *duck_position = duckInport.read();

                Bottle move_cmd ;
                move_cmd.clear();
                move_cmd.addString("push duck");
                for (int i=0; i<3; i++){
                  move_cmd.addDouble(duck_position->get(i).asDouble());
                }

                Bottle response ;
                publishState("push duck");
                if(!movementPort.write(move_cmd,response))
                {
                    publishState("movement rpc call failed");
                }
            }
            else
            {
                publishState("don't bet");

                // COMMENT HERE TO DISABLE PUSH CARD
                Bottle move_cmd ;
                move_cmd.addString("push card");
                move_cmd.addDouble(last_card_locations["icub"].x);
                move_cmd.addDouble(last_card_locations["icub"].y);
                move_cmd.addDouble(last_card_locations["icub"].z);

                Bottle response ;
                publishState("push card");
                movementPort.write(move_cmd,response) ;
            }

            if (currentGaze == "look down ok")
            {
                publishState("look up");
                currentGaze = gazeState("look up");
            }

            // The dealer distributes cards in the meantime
            Time::delay(2.0);

            if (currentGaze == "look up ok")
            {
                publishState("look down");
                currentGaze = gazeState("look down");
                publishState("looking at cards");
                Bottle *card_input = cardInport.read();
                readCardsAndUpdateScore(card_input);
            }

            if (score_robot > score_human)
            {
                publishState("ask duck position");
                Bottle *duck_position = duckInport.read();

                Bottle move_cmd ;
                move_cmd.addString("pull duck");
                for (int i=0; i<3; i++){
                  move_cmd.addDouble(duck_position->get(i).asDouble());
                }

                Bottle response ;
                publishState("pull duck");
                movementPort.write(move_cmd,response) ;

                publishState("won");
                reply.addString("I've won!");
            }
            else
            {
                publishState("lost");
                reply.addString("I've lost :(");
            }

            publishState("A good state machine never prints this.");
            return RFModule::respond(command,reply);
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
            return RFModule::respond(command,reply);
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/

    void readCardsAndUpdateScore(Bottle* card_input)
    {
        int message_size = 5;
        for (int i=0; i<(card_input->size() / message_size); i++)
        {
            int base_ind = message_size*i;
            // parse messages
            double cx = card_input->get(base_ind+0).asDouble();
            double cy = card_input->get(base_ind+1).asDouble();
            double cz = card_input->get(base_ind+2).asDouble();
            std::string owner = card_input->get(base_ind+3).asString();
            int label = card_input->get(base_ind+4).asInt();

            // save one card location of each for later use
            last_card_locations[owner] = Position3D(cx, cy, cz);

            // update scores
            if ( owner == "icub" )
            {
                score_robot += label;
                stringstream ss; ss << "icub got " << score_robot << " total score";
                publishState(ss.str());
            }
            else if ( owner == "human" )
            {
                score_human += label;
                stringstream ss; ss << "human got " << score_human << " total score";
                publishState(ss.str());

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
        cardInport.interrupt();
        duckInport.interrupt();
        return true;
    }

    /**********************************************************/
    bool close()
    {
        rpcPort.close();
        movementPort.close();
        gazePort.close();
        stateOutPort.close();
        cardInport.close();
        duckInport.close();
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

    /***************************************************/

    string gazeState(const std::string& look)
    {
        Bottle output ;
        Bottle response ;
        output.clear();
        output.addString(look.c_str());
        bool success = gazePort.write(output,response);
        if(success)
        {
            publishState("Yaaaaaaaaaaaaaaaaaaaaaaaay we are looking down");
        }
        else
        {
            publishState("Failed RPC call, bye bye");
        }

//        return response.get(0).asString();

        // here we fake away this RPC call

        return look + std::string(" ok");
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
