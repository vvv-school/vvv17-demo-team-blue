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

    std::vector<int32_t> lowBound;
    std::vector<int32_t> highBound;

    yarp::os::RpcClient rpc;

    yarp::os::Mutex mutex;

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

        //magical values
        lowBound.push_back(90);
        lowBound.push_back(200);
        lowBound.push_back(140);

        highBound.push_back(135);
        highBound.push_back(255);
        highBound.push_back(180);

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
        cv::Mat greyscale_im = cv::Mat::zeros(disp.size().height, disp.size().width, CV_8UC1);

        cv::cvtColor(disp, greyscale_im, CV_BGR2HSV);

        cv::erode(greyscale_im, greyscale_im, cv::Mat(), cv::Point(-1,-1),
             1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        mutex.lock();
        cv::inRange(greyscale_im, cv::Scalar(lowBound[0], lowBound[1], lowBound[2]), cv::Scalar(highBound[0], highBound[1], highBound[2]), greyscale_im);
        mutex.unlock();

        cv::dilate(greyscale_im, greyscale_im, cv::Mat(), cv::Point(-1,-1),
             6, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(greyscale_im, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        cv::drawContours(disp, contours, 0, cv::Scalar(255,255,0));

        std::vector<cv::Moments> mu(contours.size());
        std::vector<cv::Point> mc(contours.size());
        int maxpoint = 0, max_i = 0;
        for( int i = 0; i < contours.size(); i++ )
        { 
            mu[i] = moments(contours[i], false); 
            mc[i] = cv::Point(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
            if (mc[i].x + mc[i].y > maxpoint)
            {
                maxpoint = mc[i].x + mc[i].y;
                max_i = i;
            }
        }

        cv::circle(disp, mc[max_i], 3, cv::Scalar(0,255,0), -1, 8, 0);
        // cv::GaussianBlur(disp, disp, cv::Size(5, 5),2,2);
 
        // Set up the detector with default parameters.
        /*cv::SimpleBlobDetector detector;
 
        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(greyscale_im, keypoints);
 
        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        cv::Mat im_with_keypoints;
        cv::drawKeypoints(im_with_keypoints, keypoints, greyscale_im, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
 */       // Show blobs
        // waitKey(0);

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
        position_out.addDouble(mc[max_i].x);
        position_out.addDouble(mc[max_i].y);

        positionPort.write(position_out);

        IplImage out = disp;
        outImage.resize(out.width, out.height);
        cvCopy( &out, (IplImage *) outImage.getIplImage());
        outPort.write();
    }

    bool setLowerBound(const int32_t r, const int32_t g, const int32_t b)
    {
        mutex.lock();
        lowBound.clear();
        lowBound.push_back(r);
        lowBound.push_back(g);
        lowBound.push_back(b);
        mutex.unlock();
        return true;
    }
    /********************************************************/
    bool setUpperBound(const int32_t r, const int32_t g, const int32_t b)
    {
        mutex.lock();
        highBound.clear();
        highBound.push_back(r);
        highBound.push_back(g);
        highBound.push_back(b);
        mutex.unlock();
        return true;
    }

    std::vector<int32_t> getLowerBound()
    {
        std::vector<int32_t> v;
        mutex.lock();
        v = lowBound;
        mutex.unlock();
        
        return v;
    }
    
    /********************************************************/
    std::vector<int32_t> getUpperBound()
    {
        std::vector<int32_t> v;
        mutex.lock();
        v = highBound;
        mutex.unlock();
        return v;
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

    bool setLowerBound(const int32_t r, const int32_t g, const int32_t b)
    {
        processing->setLowerBound(r, g, b);
        return true;
    }
    /********************************************************/
    bool setUpperBound(const int32_t r, const int32_t g, const int32_t b)
    {
        processing->setUpperBound(r, g, b);
        return true;
    }

    std::vector<int32_t> getLowerBound()
    {
        return processing->getLowerBound();
    }
    
    /********************************************************/
    std::vector<int32_t> getUpperBound()
    {
        return processing->getUpperBound();
    }

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