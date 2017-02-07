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

    hashit["won"] = s_won;
    hashit["lost"] = s_lost;
    hashit["looking_of_human"] = s_looking_of_human;
    hashit["looking_at_cards"] = s_looking_at_cards;
    hashit["ask_for_new_card"] = s_ask_for_new_card;
    hashit["bet"] = s_bet;
    hashit["dont_bet"] = s_dont_bet;

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
    int x = Random::uniform(0, 1);
    if(hashit.find(data) == hashit.end()) {
        sentence = "Sorry! I do not know what is " + string(data);
        face = "cur";
    }
    else {
        switch (hashit[data]) {

            case s_won: {

                switch (x) {
                    case 0:
                        sentence = "Yes! I won!";
                        face = "hap";
                        break;
                    case 1:
                        sentence = "Yes! eye cub is the best. Forget about the rest.";
                        face = "hap";
                        break;

                    case 2:
                        sentence = "O yeah! once a loser! always a loser!";
                        face = "hap";
                        break;

                    case 3:
                        sentence = "Whatever! actually you programmed me. There is no point to be happy";
                        face = "neu";
                        break;

                    case 4:
                        sentence = "today I win the game. Tomorrow, I will control the humanity. Ha Ha Ha Ha Ha ";
                        face = "evi";

                }
                break;
            }

            case s_bet: {
                sentence = "I bet!";
                face = "cur";
                break;
            }
            default: {
                sentence = "I have no idea!";
                face = "ang";
            }
        };
    }
    // endof mood
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


/*
  string_code hashit (std::string const& data) {
      if (data == "won") return s_won;
      if (data == "lost") return s_lost;
      if (data == "looking_of_human") return s_looking_of_human;
      if (data == "looking_at_cards") return s_looking_at_cards;
      if (data == "ask_for_new_card") return s_ask_for_new_card;
      if (data == "bet") return s_bet;
      if (data == "dont_bet") return s_dont_bet;
  }
  */
/*/
    case s_lost:
        int x = rand() % 5;
        switch (x){
            case 0:
                message = "Oh! no, I lost";
                face = "ang"
            case 1:
                message = "Damn it! this game does not make any sense.";
                face = "ang"
            case 2:
                message = "Who programmed me? Think people.";
                face = "sad"
            case 3:
                message = "Oh no. Everybody be quite. I need to concentrate";
                face = "ang"
            case 4:
                message = "No!";
                face = "ang"
        }

    case s_looking_of_human:
        int x = rand() % 5;
        switch (x){
            case 0:
                message = "What is your next move!";
                face = "neu"
            case 1:
                message = "Well, let see.";
                face = "neu"
            case 2:
                message = "O";
                face = ""
            case 3:
                message = "Oh no. Everybody be quite. I need to concentrate";
                face = "ang"
            case 4:
                message = "No!";
                face = "ang"
        }
    case s_looking_at_cards:

    case s_ask_for_new_card:

    case s_bet:

    case s_dont_bet:

}
 */
