/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <mood.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Random.h>

using namespace std;
using namespace yarp::os;

Mood::Mood() { }

Mood::~Mood() { }


bool Mood::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the codedc module...";

    // open all ports
    bool ret = true;
    //commandPort.open("/mood/rpc");
    ret &= inPort.open("/mood/in");
    ret &= speechOutPort.open("/mood/speech_out");
    ret &= faceOutPort.open("/mood/face_out");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    if(!attach(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }
    hashit["starting"] = s_starting;
    hashit["looking at cards"] = s_looking_at_cards;
    hashit["pull duck"] = s_bet;
    hashit["don't bet"] = s_dont_bet;
    hashit["look up"] = s_looking_of_human;
    hashit["won"] = s_won;
    hashit["lost"] = s_lost;

    // set some paramters
//    modeParam = rf.check("mode", Value("coder")).asString();
//    if((modeParam != "coder") && (modeParam !="decoder")) {
//        yError()<<"Unknown mode value "<<modeParam;
//        return false;
//    }

    // everything is fine
    return true;
}


double Mood::getPeriod() {
    return 0.01; // module periodicity (seconds)
}


bool Mood::updateModule() {

    Bottle* input = inPort.read();
    if(input==NULL)
        return false;
    std::string data = input-> get(0).asString();

    yInfo() << "Got a text:" << data.c_str();
// moods


    std::string sentence;
    std::string face;
    int x = Random::uniform(0, 4);
    if(hashit.find(data) == hashit.end()) {
        sentence = "Sorry! I do not know what is " + string(data);
        face = "cur";
    }
    else {
        switch (hashit[data]) {

            case s_starting: {
                SendMessages("Let's start the game!",  "hap");
                        break;
            }

            case s_won: {

                switch (x) {
                    case 0:
                        SendMessages("Yes! I won!","hap");
                        break;

                    case 1:
                        SendMessages("Yes! eye cub is the best. Forget about the rest.", "hap");
                        break;

                    case 2:
                        SendMessages("O yeah! once a loser! always a loser!", "hap");
                        break;

                    case 3:
                        SendMessages("Whatever! actually you programmed me. There is no point to be happy", "neu");
                        break;

                    case 4:
                        SendMessages("today I win the game. Tomorrow, I will control the humanity. Ha Ha Ha Ha Ha ", "evi");
                        break;
                }
                break;
            }

            case s_looking_at_cards: {
                switch (x) {
                    case 0:
                        SendMessages("Do you see what I see","cun");
                        break;

                    case 1:
                        SendMessages("Good. Let's decide.", "neu");
                        break;

                    case 2:
                        SendMessages("OK. What I can do now?", "shy");
                        break;

                    case 3:
                        SendMessages("Do I how to play?", "cun");
                        break;

                    case 4:
                        SendMessages("Classsic!", "hap");
                        break;
                }
                break;
            }

            case s_dont_bet: {
                switch (x) {
                    case 0:
                        SendMessages("Nope!","ang");
                        break;

                    case 1:
                        SendMessages("How about no.", "neu");
                        break;

                    case 2:
                        SendMessages("No. This is not good.", "neu");
                        break;

                    case 3:
                        SendMessages("Damn it. no", "neu");
                        break;

                    case 4:
                        SendMessages("oh no. I don't bet.", "ang");
                        break;
                }
                break;
            }

            case s_bet: {
                switch (x) {
                    case 0:
                        SendMessages("Let's bet!","hap");
                        break;

                    case 1:
                        SendMessages("Easy! I am in!", "neu");
                        break;

                    case 2:
                        SendMessages("OK. Let's do this!", "hap");
                        break;

                    case 3:
                        SendMessages("I will take it", "hap");
                        break;

                    case 4:
                        SendMessages("OK. it seems good.", "hap");
                        break;
                }
                break;
            }

            case s_looking_of_human: {
                switch (x) {
                    case 0:
                        SendMessages("I don't believe you","neu");
                        break;

                    case 1:
                        SendMessages("What is your next move!", "neu");
                        break;

                    case 2:
                        SendMessages("Don't mess with the eye cub!", "cun");
                        break;

                    case 3:
                        SendMessages("I am watching you baby!", "hap");
                        break;

                    case 4:
                        SendMessages("Well!", "neu");
                        break;
                }
                break;
            }

            case s_lost: {
                switch (x) {
                    case 0:
                        SendMessages("Oh! no, I lost","ang");
                        break;

                    case 1:
                        SendMessages("Damn it! this game does not make any sense.", "ang");
                        break;

                    case 2:
                        SendMessages("Who programmed me? Think people.","sad");
                        break;

                    case 3:
                        SendMessages("Oh no. Everybody be quite. I need to concentrate", "ang");
                        break;

                    case 4:
                        SendMessages("No!", "ang");
                        break;
                }
                break;
            }

        }
    }
    // endof mood


    return true;
}


bool Mood::respond(const Bottle& command, Bottle& reply) {
    yInfo()<<"Got something, echo is on";
    if (command.get(0).asString()=="quit")
        return false;
    else {
        reply.clear();
        reply.addString("error");
    }
    return true;
}

void Mood::SendMessages(std::string sentence, std::string face){
    Bottle& output = speechOutPort.prepare();
    output.clear();
    output.addString(sentence.c_str());

    Bottle& outputFace = faceOutPort.prepare();
    outputFace.clear();
    outputFace.addString("set");
    outputFace.addString("all");
    outputFace.addString(face.c_str());
    speechOutPort.write();
    faceOutPort.write();
}

bool Mood::interruptModule() {
    yInfo()<<"Interrupting Mood module";
    inPort.interrupt();
    return true;
}


bool Mood::close() {
    yInfo()<<"closing Mood module";
    //commandPort.close();
    inPort.close();
    // you can force writing remaining data
    // using outPort.writeStrict();
    speechOutPort.close();
    faceOutPort.close();
    return true;
}
