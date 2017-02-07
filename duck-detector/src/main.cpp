/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "duckDetector_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort;
    yarp::os::Port positionPort;

    yarp::os::RpcClient rpc;

public:
    /********************************************************/

    Processing( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {

    };

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( "/" + moduleName + "/rgb:i" );
        outPort.open("/"+ moduleName + "/output");
        positionPort.open("/" + moduleName + "/position:o");

//        yarp::os::Network::connect("/SFM/disp:o", "/" + moduleName + "/disparity:i");
//        yarp::os::Network::connect("/SFM/rpc", "/" + moduleName + "/disparity:i");
        return true;
    }

    /********************************************************/
    void close()
    {
        outPort.close();
        positionPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &dispImage )
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();

        outImage.resize(dispImage.width(), dispImage.height());
        outImage.zero();
        
        // cv::Mat disp = cv::cvarrToMat((IplImage *)dispImage.getIplImage());
        cv::Mat inDisp_cv = cv::cvarrToMat((IplImage *)dispImage.getIplImage());  
        cv::Mat disp = inDisp_cv.clone();

        cv::GaussianBlur(disp, disp, cv::Size(5, 5),2,2);

        // cv::dilate(disp, disp, cv::Mat(), cv::Point(-1,-1),
        //     4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        // cv::Point min_loc, max_loc;
        // double min, max;
        // cv::minMaxLoc(disp, &min, &max, &min_loc, &max_loc);


        // cv::threshold(disp, disp, 80, 255, cv::THRESH_TOZERO);

        // std::vector<std::vector<cv::Point> > contours;

        // cv::findContours( disp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // cv::drawContours(disp, contours, 0, cv::Scalar(255,255,0));        

        // std::vector<cv::Moments> mu(contours.size() );
        // for( int i = 0; i < contours.size(); i++ )
        //{ mu[i] = moments( contours[i], false ); }


        //Mass center
        // std::vector<cv::Point2f> mc( contours.size() );
        // for( int i = 0; i < contours.size(); i++ )
        // { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
        //         cv::circle(disp, mc[i], 3, cv::Scalar(0,255,0), -1, 8, 0);
                // cv::Rect ROI = cv::boundingRect(contours[0]);
                // cv::rectangle(inColour_cv, cv::Point(ROI.x,ROI.y),
        //             cv::Point(ROI.x+ROI.height, ROI.y+ROI.width),cv::Scalar(0,255,0));

        //                 outTargets.clear();
        //                 yarp::os::Bottle &t = outTargets.addList();
        //             t.addDouble(ROI.x);
        //             t.addDouble(ROI.y);
        //             t.addDouble(ROI.x+ROI.width);
        //             t.addDouble(ROI.y+ROI.height);

        //             if (outTargets.size() >0 )
        //                 targetPort.write();  
        // }

        // cvtColor(disp, disp, CV_GRAY2RGB);        

        // IplImage out = dispImage.getIplImage();
        // // outImage.resize(out.width, out.height);
        // // cvCopy( &out, (IplImage *) outImage.getIplImage());
        // cvCopy( &out, (IplImage *) outImage.getIplImage());
        // outPort.write();

        yarp::os::Bottle position_out;
        position_out.addDouble(20.0);
        position_out.addDouble(30.0);
        position_out.addDouble(40.0);

        positionPort.write(position_out);

        IplImage out = disp;
        // outImage.resize(out.width, out.height);
        cvCopy( &out, (IplImage *) outImage.getIplImage());
        outPort.write();
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public duckDetector_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("duck-detector"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}