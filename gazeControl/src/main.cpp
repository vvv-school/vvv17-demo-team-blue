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
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvGaze;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        // FILL IN THE CODE
        Vector x;
        igaze->triangulate3DPoint(cogL,cogR,x); // ONLY IN SIMULATION !! otherwise looping stereo
        return x;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        // FILL IN THE CODE
        igaze->lookAtFixationPoint(x) ;
        igaze->waitMotionDone();
    }

    /***************************************************/
    void look_down()
    {
        // FILL IN THE CODE
        // Vector xd(3) ;
        // xd[0] = -0.5 ; // in front of the robot
        // xd[1] = 0.0 ; // in the middle
        // xd[2] = -0.5 ;
        // igaze->lookAtFixationPoint(xd) ;

        Vector ang(3,0.0);
        ang[1] = -40.0 ; //elevation [deg]
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();

    }

    /***************************************************/
    void look_up()
    {
        // FILL IN THE CODE
        // Vector xd(3) ;
        // xd[0] = -0.5 ; // in front of the robot
        // xd[1] = 0.0 ; // in the middle
        // xd[2] = -0.5 ;
        // igaze->lookAtFixationPoint(xd) ;

        Vector ang(3,0.0);
        ang[1] = 0.0 ; //elevation [deg]
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();

    }

    /***************************************************/
    void detectImage(const Vector &cogL, const Vector &cogR)
    {
        yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

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

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply) // list of instruction that are required to implemet
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- look_up");
            reply.addString("- quit");
        }
        else if (cmd=="look_up")
        {
            look_up();
            reply.addString("Yep! I'm looking up now!");
        }
        else if (cmd=="look_down")
        {
          look_down();
          reply.addString("Yep! I'm looking down now!");


            bool go = okL && okR ;
            Vector cogL = this->cogL ; // create a copy which is local to protect acess
            Vector cogR = this->cogR ;

            if (go)
            {
                detectImage(cogL,cogR);
                reply.addString("Yeah! I've detect something down !");
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
        return 0.0;     // sync upon incoming images - the update module continuously update: this happens when you do read true
    }

    /***************************************************/
    bool updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read(); // read() means read(true) by default : it means it is blocked here until it receives an image (contrary to previously which was unblocking)
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL)) // when we press ctrl+c so stopped but normally pointing to valid objects so process
            return false;

        // compute the center-of-mass of pixels of our color
        mutex.lock(); // protected
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0; // we make the ball blue

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();

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

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}
