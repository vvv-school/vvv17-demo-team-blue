// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArmR, drvArmL, drvGaze;
    PolyDriver drvHandR, drvHandL;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    int startup_ctxt_arm_right;
    int startup_ctxt_arm_left;
    int startup_ctxt_gaze;

    RpcServer rpcPort;
    ObjectRetriever object;

    /***************************************************/
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
        approach[2] += 0.05; // 5 cm
        iarm->goToPoseSync(approach, o);
        iarm->waitMotionDone();

        // reach the final target x;

        iarm->goToPoseSync(x, o);
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
    void home(const string &hand)
    {
        Vector home_x(3);
        home_x[0]=-0.2;
        home_x[2]=0.08;

        // select the correct interface
        if (hand=="right")
        {
            drvArmR.view(iarm);
            home_x[1]=0.3;
        }
        else
        {
            drvArmL.view(iarm);
            home_x[1]=-0.3;
        }

        igaze->lookAtAbsAngles(Vector(3,0.0));
        iarm->goToPositionSync(home_x);

        iarm->waitMotionDone();
        igaze->waitMotionDone();
        igaze->setTrackingMode(false);
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

    /***************************************************/
    bool grasp_it(const double fingers_closure)
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
            moveFingers(hand,fingers,0.0);
            yInfo()<<"prepared hand";

            approachTargetWithHand(hand,x,o);
            yInfo()<<"approached object";

            moveFingers(hand,fingers,fingers_closure);
            yInfo()<<"grasped";

            liftObject(hand);
            yInfo()<<"lifted";

            moveFingers(hand,fingers,0.0);
            yInfo()<<"released";

            home(hand);
            yInfo()<<"gone home";
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
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        http://www.google.com
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

        // save startup contexts
        drvArmR.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_right);

        drvArmL.view(iarm);
        iarm->storeContext(&startup_ctxt_arm_left);

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

    void deny_card()
    {
        Vector denyConfig(16, 0.0);
        denyConfig[0] = 36.0;
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
        else if (cmd=="grasp_it")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            double fingers_closure=0.5; // default value

            // we can pass a new value via rpc
            if (command.size()>1)
                fingers_closure=command.get(1).asDouble();

            bool ok=grasp_it(fingers_closure);
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
    rf.configure(argc,argv);
    return mod.runModule(rf);
}

