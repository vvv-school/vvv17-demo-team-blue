// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>
#include <cmath>
#include <algorithm>
#include <memory>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>

#include "slidingController_IDL.h"

#define EXPLORATION_TOL     5e-3

#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::perception;
using namespace iCub::action;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArmR, drvArmL, drvGaze;
    PolyDriver driverJoint;
    PolyDriver drvHandR, drvHandL;
    ICartesianControl *iarm;
    string arm;
    bool impedanceSw,oldImpedanceSw;
    IGazeControl      *igaze;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;

    RpcServer rpcPort;
    ObjectRetriever object;

    ActionPrimitivesLayer1  action;
    string graspModelFileToWrite;
    deque<string> handKeys;
    double exploration_max_force;

    Mutex mainMutex;
    Mutex expMutex;
    /******************Touch model***********/
    bool calibrateGraspModel(const bool forceCalibration)
    {
        Model *model; action.getGraspModel(model);
        if (model!=NULL)
        {
            if (forceCalibration || !model->isCalibrated())
            {
                Bottle fingers;
                Bottle &fng=fingers.addList();
                fng.addString("index");
                fng.addString("middle");
                fng.addString("ring");
                fng.addString("little");

                Property prop;
                prop.put("finger",fingers.get(0));
                model->calibrate(prop);

                prop.clear();
                prop.put("finger","thumb");
                model->calibrate(prop);

                ofstream fout;
                fout.open(graspModelFileToWrite.c_str());
                model->toStream(fout);
                fout.close();
            }

            return true;
        }

        return false;
    }

    /******************ARM CONTROLLER*******************/
    void setImpedance(const bool enable=true, const bool forceSet=false)
    {
        if (!forceSet && (enable==impedanceSw))
            return;

        IInteractionMode *imode;
        drvArmR.view(imode);

        if (enable)
        {
            IImpedanceControl *iimp;
            drvArmR.view(iimp);
            //Thanks Ugo ;)
            imode->setInteractionMode(0,VOCAB_IM_COMPLIANT); iimp->setImpedance(0,0.4,0.03);
            imode->setInteractionMode(1,VOCAB_IM_COMPLIANT); iimp->setImpedance(1,0.4,0.03);
            imode->setInteractionMode(2,VOCAB_IM_COMPLIANT); iimp->setImpedance(2,0.4,0.03);
            imode->setInteractionMode(3,VOCAB_IM_COMPLIANT); iimp->setImpedance(3,0.2,0.01);
            imode->setInteractionMode(4,VOCAB_IM_COMPLIANT); iimp->setImpedance(4,0.2,0.0);
        }
        else for (int j=0; j<5; j++)
            imode->setInteractionMode(j,VOCAB_IM_STIFF);

        impedanceSw=enable;
    }

    void fixate(const Vector &x)
    {
        // simply look at x,
        // but when the movement is over
        // ensure that we'll still be looking at x

        igaze->setTrackingMode(true);
        igaze->lookAtFixationPoint(x);
        igaze->waitMotionDone();
    }



    /***************************************************/
    Vector computeHandOrientation(const string &hand)
    {
        // we have to provide a 4x1 vector representing the
        // final orientation for the specified hand, with
        // the palm pointing downward

        Matrix Rot(3,3);
        if (hand == "right")
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)=-1.0;
        }
        else
        {
            Rot(0,0)=-1.0; Rot(0,1)=  0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= -1.0; Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)=  0.0; Rot(2,2)= 1.0;
        }
        Vector o(4);
        o[0] = 0.0;
        o[1] = -1.0;
        o[2] = 0.0;
        o[3] = 30.0 * (M_PI / 180.0);

        Matrix RotY = axis2dcm(o).submatrix(0, 2, 0, 2);

        yInfo() << Rot.rows() << "x " << Rot.cols() << " * "
                << RotY.rows() << "x" << RotY.cols();

        return dcm2axis(RotY * Rot);
    }

    /***************************************************/
    Vector computeHandOrientationForRetrieve(const string &hand)
    {
        // we have to provide a 4x1 vector representing the
        // final orientation for the specified hand, with
        // the palm pointing downward

        Matrix Rot(3,3);
        if (hand == "right")
        {
            Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)=-1.0;
        }
        else
        {
            Rot(0,0)=-1.0; Rot(0,1)=  0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= -1.0; Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)=  0.0; Rot(2,2)= 1.0;
        }
        Vector o(4);
        o[0] = 0.0;
        o[1] = -1.0;
        o[2] = 0.0;
        o[3] = 50.0 * (M_PI / 180.0);

        Matrix RotY = axis2dcm(o).submatrix(0, 2, 0, 2);

        yInfo() << Rot.rows() << "x " << Rot.cols() << " * "
                << RotY.rows() << "x" << RotY.cols();

        return dcm2axis(RotY * Rot);
    }

    /***************************************************/
    void approachTargetWithHand(const string &hand,
                                  const Vector &x,
                                 const Vector &o)
    {
        // select the correct interface
        if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // enable all dofs but the roll of the torso

        Vector dof(10, 1.0);
        dof[1] = 0.0;
        Vector curDof;
        iarm->setDOF(dof, curDof);

        // reach the first via-point
        // located 5 cm above the target x

        Vector approach = x;
        approach[2] += 0.02;
        iarm->goToPoseSync(approach, o);
        iarm->waitMotionDone();

        // reach the final target x;

        iarm->goToPoseSync(x, o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void approach_target_for_push(const string &hand,
                                  const Vector &x,
                                 const Vector &o)
    {
        // select the correct interface
        if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        Model *model; action.getGraspModel(model);
          if (model!=NULL)
          {
              Value out;
              model->getOutput(out);
              double contact_force=out.asList()->get(1).asDouble();   // 1 => index finger
              if (contact_force>exploration_max_force)
              {
                  printf("contact detected: (%g>%g)\n",contact_force,exploration_max_force);

                  //INSERT PUSH CARD HERE

                  expMutex.unlock();
              }
           }
        // enable all dofs but the roll of the torso

        Vector dof(10, 1.0);
        dof[1] = 0.0;
        Vector curDof;
        iarm->setDOF(dof, curDof);

        // TODO: Make the
        Vector approach = x;
        approach[0] += 0.10;
        approach[2] += 0.02;
        iarm->goToPoseSync(approach, o);
        iarm->waitMotionDone();

        // reach the final target x;

        Vector afterPush = x;
        afterPush[0] -= 0.03;
        iarm->goToPoseSync(afterPush, o);
        iarm->waitMotionDone();

        Vector goUp = x;
        goUp[2] += 0.08;
        iarm->goToPoseSync(goUp, o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void liftObject(const string &hand)
    {
        // select the correct interface
        if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // just lift the hand of few centimeters
        // wrt the current position

        Vector x, o;
        if (iarm->getPose(x, o))
        {
            x[2] += 0.1;
            iarm->goToPoseSync(x, o);
            iarm->waitMotionDone();
        }
    }

    /***************************************************/
    void moveFingers(const string &hand,
                     const VectorOf<int> &joints,
                     const double fingers_closure)
    {
        // select the correct interface
        IControlLimits   *ilim;
        IPositionControl *ipos;
        IControlMode2     *imod;
        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(ipos);
            drvHandR.view(imod);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(ipos);
            drvHandR.view(imod);
        }

        // enforce [0,1] interval
        double fingers_closure_sat=std::min(1.0,std::max(0.0,fingers_closure));

        // move each finger first:
        // if min_j and max_j are the minimum and maximum bounds of joint j,
        // then we should move to min_j+fingers_closure_sat*(max_j-min_j)

        for (size_t i=0; i<joints.size(); i++)
        {
            int j=joints[i];

            double min_j, max_j;
            if (ilim->getLimits(j, &min_j, &max_j))
            {
                double jointValue = min_j + fingers_closure_sat * (max_j - min_j);
                imod->setControlMode(j, VOCAB_CM_POSITION);
                ipos->positionMove(j, jointValue);
            }
        }

        // wait until all fingers have attained their set-points
        bool done;
        do
        {
            done = true;
            for (size_t i=0; i<joints.size(); i++)
            {
                int j = joints[i];
                bool jointDone = false;
                ipos->checkMotionDone(j, &jointDone);
                done = done && jointDone;
            }
        } while (!done);
    }

    /***************************************************/
    bool home(const string &hand)
    {
        Vector homeX, homeO;

        // select the correct interface
        if (hand=="right")
        {
            drvArmR.view(iarm);
            homeX = rightArmStartX;
            homeO = rightArmStartO;
        }
        else
        {
            drvArmL.view(iarm);
            homeX = leftArmStartX;
            homeO = leftArmStartO;
        }


        iarm->goToPoseSync(homeX, homeO);
        iarm->waitMotionDone();

        yInfo() << "Moved arms home";

        igaze->lookAtAbsAngles(Vector(3,0.0));
        igaze->setTrackingMode(false);
        //igaze->waitMotionDone();


        return true;
    }

    /***************************************************/
    void look_down()
    {
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        igaze->blockEyes(5.0);
        Vector ang(3,0.0);
        ang[1]=-60.0;
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
    }

    bool push_object()
    {
        Vector x; string hand;
        if (object.getLocation(x))
        {
            yInfo()<<"retrieved 3D location = ("<<x.toString(3,3)<<")";

            // we select the hand accordingly
            hand=(x[1]>0.0?"right":"left");
            yInfo()<<"selected hand = \""<<hand<<'\"';
        }
        else
            return false;

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // refine the localization of the object
        // with a proper hand-related map
        if (object.getLocation(x,hand))
        {
            yInfo()<<"refined 3D location = ("<<x.toString(3,3)<<")";

            Vector o=computeHandOrientation(hand);
            yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

            // we set up here the lists of joints we need to actuate
            VectorOf<int> abduction,thumb,fingers;
            abduction.push_back(7);
            thumb.push_back(8);
            for (int i=9; i<16; i++)
                fingers.push_back(i);

            // let's put the hand in the pre-grasp configuration
            moveFingers(hand,abduction,0.7);
            moveFingers(hand,thumb,1.0);
            moveFingers(hand,fingers,0.9);
            yInfo()<<"prepared hand";

            approach_target_for_push(hand,x,o);
            yInfo()<<"approached object";

            return true;
        }
        return false;
    }

    bool retrieve_object(double fingers_closure)
    {
        Vector x; string hand;
        if (object.getLocation(x))
        {
            yInfo()<<"retrieved 3D location = ("<<x.toString(3,3)<<")";

            // we select the hand accordingly
            hand=(x[1]>0.0?"right":"left");
            yInfo()<<"selected hand = \""<<hand<<'\"';
        }
        else
            return false;

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // refine the localization of the object
        // with a proper hand-related map
        if (object.getLocation(x,hand))
        {
            yInfo()<<"refined 3D location = ("<<x.toString(3,3)<<")";

            Vector o=computeHandOrientationForRetrieve(hand);
            yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

            // we set up here the lists of joints we need to actuate
            VectorOf<int> abduction,thumb,fingers;
            abduction.push_back(7);
            thumb.push_back(8);
            for (int i=9; i<16; i++)
                fingers.push_back(i);

            // let's put the hand in the pre-grasp configuration
            moveFingers(hand,abduction,0.7);
            moveFingers(hand,thumb,1.0);
            moveFingers(hand,fingers,0.2);
            yInfo()<<"prepared hand";

            //approach_target_for_push(hand,x,o);
            yInfo()<<"approached object";

            return true;
        }
        return false;
    }



    /***************************************************/
    bool openCartesian(const string &robot, const string &arm)
    {
        PolyDriver &drvArm=(arm=="right_arm"?drvArmR:drvArmL);

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/"+arm);
        optArm.put("local","/cartesian_client/"+arm);

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller for "<<arm;
            return false;
        }
        return true;
    }

    bool openHandControl(std::string const& robot, PolyDriver& driver, std::string const& name)
    {
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/" + robot + "/" + name);
        optJoint.put("local","/position/" + name);

        return driver.open(optJoint);
    }

public:
    yarp::sig::Vector rightArmStartX;
    yarp::sig::Vector rightArmStartO;
    yarp::sig::Vector leftArmStartX;
    yarp::sig::Vector leftArmStartO;

    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        //string robot=rf.check("robot",Value("icub")).asString().c_str();
        string robot=rf.check("robot",Value("icubSim")).asString();

        if (!openCartesian(robot,"right_arm"))
            return false;

        if (!openCartesian(robot,"left_arm"))
        {
            drvArmR.close();
            return false;
        }

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!drvGaze.open(optGaze))
        {
            yError() << "Unable to open Gaze controller";
            drvArmR.close();
            drvArmL.close();
            return false;
        }

        if (!openHandControl(robot, drvHandL, "left_arm"))
        {
            yError()<<"Unable to connect to /left_arm";
            drvArmR.close();
            drvArmL.close();
            drvGaze.close();
            return false;
        }

        if (!openHandControl(robot, drvHandR, "right_arm"))
        {
            yError()<<"Unable to connect to /right_arm";
            drvArmR.close();
            drvArmL.close();
            drvGaze.close();
            drvHandL.close();
            return false;
        }
        Property optionAction;
        string name=rf.check("name",Value("slidingController")).asString().c_str();

        string grasp_model_file=(arm=="left"?"grasp_model_file_left":"grasp_model_file_right");
        optionAction.put("robot",robot.c_str());
        optionAction.put("local",(name+"/action").c_str());
        optionAction.put("part",(arm+"_arm").c_str());
        optionAction.put("torso_pitch","on");
        optionAction.put("torso_roll","off");
        optionAction.put("torso_yaw","on");
        optionAction.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionAction.put("grasp_model_file",rf.findFile(grasp_model_file.c_str()).c_str());
        optionAction.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());
        graspModelFileToWrite=rf.getHomeContextPath().c_str();
        graspModelFileToWrite+="/";
        graspModelFileToWrite+=rf.find(grasp_model_file.c_str()).asString().c_str();

        if (!action.open(optionAction))
        {
            drvHandR.close();
            driverJoint.close();
            return false;
        }

        handKeys=action.getHandSeqList();
        printf("***** List of available hand sequence keys:\n");
        for (size_t i=0; i<handKeys.size(); i++)
            printf("%s\n",handKeys[i].c_str());
        calibrateGraspModel(false);

        // save startup contexts
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);
        while (!iarm->getPose(rightArmStartX, rightArmStartO))
        {
            yInfo() << "Waiting for right arm pos";
            Time::delay(0.1);
        }

        drvArmL.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_left);
        while (!iarm->getPose(leftArmStartX, leftArmStartO))
        {
            yInfo() << "Waiting for left arm pos";
            Time::delay(0.1);
        }

        drvGaze.view(igaze);
        igaze->storeContext(&startup_ctxt_gaze);

        rpcPort.open("/service");
        attach(rpcPort);
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
        drvArmR.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_right);

        drvArmL.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_left);

        igaze->restoreContext(startup_ctxt_gaze);

        drvArmR.close();
        drvArmL.close();
        drvGaze.close();
        drvHandR.close();
        drvHandL.close();
        rpcPort.close();
        return true;
    }

    bool approach_card(Vector cardPos)
    {
        // TODO: bla
        Vector approachPos = cardPos;
        approachPos[2] += 0.05;

        std::string hand=(cardPos[1]>0.0?"right":"left");

        fixate(cardPos);

        Vector handOrientation(4, 0.0);
        {
            Matrix Rot(3,3);
            if (hand == "right")
            {
                Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
                Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
                Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)=-1.0;
            }
            else
            {
                Rot(0,0)=-1.0; Rot(0,1)=  0.0; Rot(0,2)= 0.0;
                Rot(1,0)= 0.0; Rot(1,1)= -1.0; Rot(1,2)= 0.0;
                Rot(2,0)= 0.0; Rot(2,1)=  0.0; Rot(2,2)= 1.0;
            }
            Vector o(4);
            o[0] = 0.0;
            o[1] = -1.0;
            o[2] = 0.0;
            o[3] = 30.0 * (M_PI / 180.0);

            Matrix RotY = axis2dcm(o).submatrix(0, 2, 0, 2);

            yInfo() << Rot.rows() << "x " << Rot.cols() << " * "
                    << RotY.rows() << "x" << RotY.cols();

            handOrientation = dcm2axis(RotY * Rot);
        }


        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- grasp_it");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd == "push_object")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            double fingers_closure=0.9; // default value

            // we can pass a new value via rpc
            if (command.size()>1)
                fingers_closure=command.get(1).asDouble();

            bool ok=push_object();
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd == "approach_card")
        {
            Vector cardPos(3, 0.0);
            cardPos[0] = command.get(1).asDouble();
            cardPos[1] = command.get(2).asDouble();
            cardPos[2] = command.get(3).asDouble();

            bool ok = approach_card(cardPos);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd == "home")
        {
            bool ok = home("right") && home("left");
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I couldn't get home :(");
            }
        }
        else if (cmd == "retrieve_object")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            double fingers_closure=0.2; // default value

            // we can pass a new value via rpc
            if (command.size()>1)
                fingers_closure=command.get(1).asDouble();

            bool ok=retrieve_object(fingers_closure);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd == "home")
        {
            bool ok = home("right") && home("left");
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I couldn't get home :(");
            }
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /***************************************************/
    bool updateModule()
    {
        return true;
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

    CtrlModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("slidingController");
    rf.setDefault("grasp_model_type","springy");
    rf.setDefault("grasp_model_file_left","grasp_model_left.ini");
    rf.setDefault("grasp_model_file_right","grasp_model_right.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.configure(argc,argv);

    return mod.runModule(rf);
}
