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

#include <iCub/ctrl/math.h>
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
class CtrlModule: public RFModule, public slidingController_IDL
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

    RpcClient port_forces;

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

                /*Property prop;
                prop.put("finger",fingers.get(0));
                model->calibrate(prop);

                prop.clear();
                prop.put("finger","thumb");
                model->calibrate(prop);

                ofstream fout;
                fout.open(graspModelFileToWrite.c_str());
                model->toStream(fout);
                fout.close();*/
                Property prop("(finger all_parallel)");
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
        o[3] = 30.0 * (M_PI / 180.0);

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


        iarm->goToPoseSync(homeX, homeO, 30);
        iarm->waitMotionDone();

        yInfo() << "Moved arms home";

        //igaze->lookAtAbsAngles(Vector(3,0.0));
        //igaze->setTrackingMode(false);
        //igaze->waitMotionDone();


        return true;
    }

    bool push_object(const Vector &pos)
    {
        /*if (object.getLocation(x))
        {
            yInfo()<<"retrieved 3D location = ("<<x.toString(3,3)<<")";

            // we select the hand accordingly
            hand=(x[1]>0.0?"right":"left");
            yInfo()<<"selected hand = \""<<hand<<'\"';
        }
        else
            return false;*/
        Vector x=pos;
        string hand="right";

        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // refine the localization of the object
        // with a proper hand-related map
//        if (x[1]>0.0)
//        {
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

            home("right");

            return true;
//        }
        return false;
    }

    bool retrieve_object(const Vector &pos)
    {
        Vector x; string hand;
        x = pos;
        hand = "right";

        // refine the localization of the object
        // with a proper hand-related map

            yInfo()<<"refined 3D location = ("<<x.toString(3,3)<<")";

            Vector handOrientation(4, 0.0);
            {
                Matrix Rot(3,3);

                Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
                Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
                Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)= -1.0;

                handOrientation = dcm2axis(Rot);
            }

            yInfo()<<"computed orientation = ("<<handOrientation.toString(3,3)<<")";

            // we set up here the lists of joints we need to actuate
            VectorOf<int> abduction,thumb,fingers;
            abduction.push_back(7);
            thumb.push_back(8);
            for (int i=9; i<16; i++)
                fingers.push_back(i);

            // let's put the hand in the pre-grasp configuration
            moveFingers(hand,abduction,0.7);
            moveFingers(hand,thumb,0.0);
            moveFingers(hand,fingers,0.3);
            yInfo()<<"prepared hand";

            //approach_target_for_push(hand,x,o);
            yInfo()<<"approached object";

            Vector approachFromAbove = x;
            approachFromAbove[2] += 0.05; // be higher than object
            approachFromAbove[0] -= 0.00; // be behind object
            drvArmR.view(iarm);
            iarm->goToPoseSync(approachFromAbove,handOrientation,30);
            iarm->waitMotionDone();
            Vector goDown = approachFromAbove;
            goDown[2] = x[2];
            iarm->goToPoseSync(goDown,handOrientation,30);
            iarm->waitMotionDone();
            Vector pullBack;
            pullBack = goDown;
            pullBack[0] += 0.1;
            iarm->goToPoseSync(pullBack,handOrientation,30);
            iarm->waitMotionDone();

            Vector goUp = pullBack;
            goUp[2] += 0.06;
            iarm->goToPoseSync(goUp, handOrientation, 30);
            iarm->waitMotionDone();

            home("right");

            return true;

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

    bool inSimulation;

    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        http://www.google.com
        https://www.facebook.com

        //string robot=rf.check("robot",Value("icub")).asString().c_str();
        /* Calibration of the force sensor*/
        port_forces.open("/wholeBodyDynamics/rpc:i");
        Bottle cmd;
        cmd.addInt(0);
        Bottle response;
        port_forces.write(cmd, response);
        yInfo()<<"Force sensor:"<<response.toString();
        /* Calibration of the force sensor*/
        string robot=rf.check("robot",Value("icubSim")).asString();
        arm=rf.check("arm",Value("right")).asString().c_str();
        string name=rf.check("name",Value("movement-controller")).asString().c_str();
        inSimulation = robot == "icubSim";

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

        robot=rf.check("robot",Value("icub")).asString().c_str();
        string grasp_model_file=(arm=="left"?"grasp_model_file_left":"grasp_model_file_right");
        optionAction.put("robot",robot.c_str());
        optionAction.put("local",(name+"/action").c_str());
        optionAction.put("part",(arm+"_arm").c_str());
        optionAction.put("torso_pitch","on");
        optionAction.put("torso_roll","off");
        optionAction.put("torso_yaw","on");
        optionAction.put("verbosity","on");
        optionAction.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        optionAction.put("grasp_model_file",rf.findFile(grasp_model_file.c_str()).c_str());
        optionAction.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());
        graspModelFileToWrite=rf.getHomeContextPath().c_str();
        graspModelFileToWrite+="/";
        graspModelFileToWrite+=rf.find(grasp_model_file.c_str()).asString().c_str();
        yInfo() << "home path: " << rf.getHomeContextPath();

//        robot=rf.check("robot",Value("icubSim")).asString();
        if (!inSimulation)
        {
            if (!action.open(optionAction))
            {
                yError() << "Could not open action";
                drvHandR.close();
                driverJoint.close();
                return false;
            }

            handKeys=action.getHandSeqList();
            printf("***** List of available hand sequence keys:\n");
            for (size_t i=0; i<handKeys.size(); i++)
                printf("%s\n",handKeys[i].c_str());
            calibrateGraspModel(false);
        }

        // save startup contexts
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);
        while (!iarm->getPose(rightArmStartX, rightArmStartO))
        {
            yInfo() << "Waiting for right arm pos";
            Time::delay(0.1);
        }
        // Activate the torso for arm movements
        Vector dof(10,1.0),dummy;
        dof[1] = 0.0;
        iarm->setDOF(dof,dummy);

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
        approachPos[0] += 0.00;
        approachPos[2] += 0.05;

        // We only use the right hand
        std::string hand="right";
        yInfo() << "Choosing hand: " << hand;

        Vector handOrientation(4, 0.0);
        {
            Matrix Rot(3,3);

            Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)= -1.0;

            handOrientation = dcm2axis(Rot);
        }

        VectorOf<int> abduction,thumb,fingers, index;
        abduction.push_back(7);
        thumb.push_back(8);
        int indexFingerJ = 11;
        index.push_back(indexFingerJ);
        for (int i=9; i<16; i++)
        {
            if (i != indexFingerJ)
                fingers.push_back(i);
        }

        Vector fingerJoints;
        for (int i = 7; i < 16; ++i)
        {
            fingerJoints.push_back(i);
        }

        Vector fingerValues(fingerJoints.size());
        fingerValues[0] = 50.0;
        fingerValues[1] = 80.0;
        fingerValues[2] = 10.0;
        fingerValues[3] = 120.0;
        fingerValues[4] = 22.5;
        fingerValues[5] = 72.0;
        fingerValues[6] = 0.0;
        fingerValues[7] = 0.0;
        fingerValues[8] = 0.0;

        IControlLimits   *ilim;
        IPositionControl *ipos;
        IControlMode2     *imod;
        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(ipos);
            drvHandR.view(imod);
            drvArmR.view(iarm);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(ipos);
            drvHandL.view(imod);
            drvArmL.view(iarm);
        }

        for (size_t i=0; i<fingerJoints.size(); i++)
        {
            int j=fingerJoints[i];
            double value = fingerValues[i];

            imod->setControlMode(j, VOCAB_CM_POSITION);
            ipos->positionMove(j, value);
        }

        // wait until all fingers have attained their set-points
        bool done;
        do
        {
            done = true;
            for (size_t i=0; i<fingerJoints.size(); i++)
            {
                int j = fingerJoints[i];
                bool jointDone = false;
                ipos->checkMotionDone(j, &jointDone);
                done = done && jointDone;
            }
        } while (!done);

        if (!iarm->goToPoseSync(approachPos, handOrientation, 20.0))
        {
            yError() << "Could not move to approach position";
            return false;
        }
        iarm->waitMotionDone();



        return true;
    }

    bool touch_card(Vector cardPos)
    {
        expMutex.lock();
        Vector approachPos = cardPos;
        approachPos[2] -= 0.1;

        // We only use the right hand
        std::string hand="right";
        yInfo() << "Choosing hand: " << hand;

        Vector handOrientation(4, 0.0);
        {
            Matrix Rot(3,3);

            Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)= -1.0;

            handOrientation = dcm2axis(Rot);
        }

        // wait until all fingers have attained their set-points

        if (!iarm->goToPoseSync(approachPos, handOrientation, 20.0))
        {
            yError() << "Could not move to approach position";
            return false;
        }
        //iarm->waitMotionDone();
        Model *model; action.getGraspModel(model);
        if(!inSimulation){
        if (model!=NULL)
        {
            Value out;
            model->getOutput(out);
            double contact_force=out.asList()->get(1).asDouble();   // 1 => index finger
            while (contact_force<exploration_max_force)
            {
                model->getOutput(out);
                contact_force=out.asList()->get(1).asDouble();   // 1 => index finger
                printf("going down... (%g>%g)\n",contact_force,exploration_max_force);//INSERT PUSH CARD HERE

            }
            iarm->stopControl();
            printf("contact detected: (%g>%g)\n",contact_force,exploration_max_force);
        }
        }
        expMutex.unlock();

        return true;
    }
    bool push_card(const Vector &pos)
    {
        Vector approachPos=pos;
        string hand="right";

        Vector handOrientation(4, 0.0);
        {
            Matrix Rot(3,3);

            Rot(0,0)=-1.0; Rot(0,1)= 0.0;  Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0;  Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0;  Rot(2,2)= -1.0;

            handOrientation = dcm2axis(Rot);
        }

        if (!iarm->goToPoseSync(approachPos, handOrientation, 20.0))
        {
            yError() << "Could not move to approach position";
            return false;
        }
        else{
          iarm->waitMotionDone();
          yInfo()<<"pushing card";
          return true;
        }

    }
    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- push duck X Y Z");
            reply.addString("- pull duck X Y Z");
            reply.addString("- push card X Y Z");
            reply.addString("- home");
        }
        else if (cmd == "push duck")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            Vector duckPos(3, 0.0);
            duckPos[0] = command.get(1).asDouble();
            duckPos[1] = command.get(2).asDouble();
            duckPos[2] = command.get(3).asDouble();

            bool ok=push_object(duckPos);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ok");
            }
            else
            {
                reply.addString("fail");
            }
        }
        else if (cmd == "pull duck")
        {
            Vector duckPos(3, 0.0);
            duckPos[0] = command.get(1).asDouble();
            duckPos[1] = command.get(2).asDouble();
            duckPos[2] = command.get(3).asDouble();

            // reach the first via-point
            // located 5 cm above the target x
            //iarm->goToPoseSync(approach,o);

            bool ok = retrieve_object(duckPos);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ok");
            }
            else
            {
                reply.addString("fail");
            }
        }
        else if (cmd == "push card")
        {
            Vector cardPos(3, 0.0);
            cardPos[0] = command.get(1).asDouble();
            cardPos[1] = command.get(2).asDouble();
            cardPos[2] = command.get(3).asDouble();

            // reach the first via-point
            // located 5 cm above the target x
            //iarm->goToPoseSync(approach,o);

            bool ok = approach_card(cardPos);
            ok = ok && touch_card(cardPos);
            ok = ok && push_card(cardPos);

            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ok");
            }
            else
            {
                reply.addString("fail");
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
        else if (cmd == "touch_card")
        {
            Vector cardPos(3, 0.0);
            cardPos[0] = command.get(1).asDouble();
            cardPos[1] = command.get(2).asDouble();
            cardPos[2] = command.get(3).asDouble();

            // reach the first via-point
            // located 5 cm above the target x
            //iarm->goToPoseSync(approach,o);

            bool ok = touch_card(cardPos);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any card!");
            }
        }
        else if (cmd == "push_card")
        {
            Vector cardPos(3, 0.0);
            cardPos[0] = command.get(1).asDouble()-0.05;//moves 5 cm the cart to the front
            cardPos[1] = command.get(2).asDouble();
            cardPos[2] = command.get(3).asDouble();

            // reach the first via-point
            // located 5 cm above the target x
            //iarm->goToPoseSync(approach,o);

            bool ok = push_card(cardPos);
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any card!");
            }
        }
        else if (cmd == "get_object_loc")
        {
            // Temporary
            Vector loc;
            if (object.getLocation(loc))
            {
                reply.addString("ack");
                reply.addString(loc.toString());
            }
            else
            {
                reply.addString("nack");
            }
        }
        else if (cmd == "home")
        {
            bool ok = home("right") && home("left");
            if (ok)
            {
                reply.addString("ok");
            }
            else
            {
                reply.addString("fail");
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
    rf.setDefaultContext("movement-controller");
    rf.setDefault("grasp_model_type","springy");
    rf.setDefault("grasp_model_file_left","grasp_model_left.ini");
    rf.setDefault("grasp_model_file_right","grasp_model_right.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.configure(argc,argv);

    return mod.runModule(rf);
}
