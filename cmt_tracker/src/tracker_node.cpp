#include "CMT.h"
#include "gui.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/thread.hpp>
#include "mycommbase//InitTrackObject.h"
#include "mycommbase/GetImage.h"

#include "mycommbase/TargetData.h"
#include "mycommbase/TrackingData.h"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace image_transport;

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;

class TrackCMTRos
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::string image_topic_,target_topic_;
    image_transport::CameraSubscriber camera_sub_;
    ros::ServiceServer srv_server_;
    ros::ServiceServer getImage_server_;
    ros::Publisher target_pub_;
    boost::mutex g_image_mutex_;

    mycommbase::TargetData targetData;
    mycommbase::TrackingData trackData;
    bool isInit;

    CMT cmt_;
    Rect rect_;
    Mat initImg;
    ros::Time lastInitWarning;
    sensor_msgs::Image rect_image;



public:
    TrackCMTRos():nh_priv_("~"),isInit(false)
    {

        nh_priv_.param("image",image_topic_,std::string("image_raw"));
        nh_priv_.param("target",target_topic_,std::string("target"));

        srv_server_=nh_priv_.advertiseService<>("init_tracker",&TrackCMTRos::initCallback,this);
        getImage_server_=nh_priv_.advertiseService<>("get_image",&TrackCMTRos::getImagecallback,this);

        image_transport::ImageTransport itp(nh_priv_);
        camera_sub_=itp.subscribeCamera(image_topic_,1,&TrackCMTRos::OnImageCallBack,this);

        target_pub_=nh_priv_.advertise<mycommbase::TrackingData>(target_topic_,1);

        //Set up logging
        FILELog::ReportingLevel() = logINFO;
        Output2FILE::Stream() = stdout; //Log to stdout
        ROS_INFO("Requesting the tracking target object...");
        ros::Rate r(10);
        while(!isInit && ros::ok())
        {

            ros::spinOnce();
            r.sleep();
        }
        ROS_INFO("receive the tracking target object rectangle (%d %d %d %d)",
                 rect_.tl().x,rect_.tl().y,rect_.width,rect_.height);


    }
    int display(Mat im, CMT & cmt)
    {
        //Visualize the output
        //It is ok to draw on im itself, as CMT only uses the grayscale image
        for(size_t i = 0; i < cmt.points_active.size(); i++)
        {
            circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
        }

        Point2f vertices[4];
        cmt.bb_rot.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
        }

        imshow("view", im);

        return waitKey(5);
    }
    void OnImageCallBack(const sensor_msgs::ImageConstPtr& imagePtr,
                         const sensor_msgs::CameraInfoConstPtr& cameraInfoPtr)
    {
        boost::mutex::scoped_lock lock(g_image_mutex_);
        //std::cout<<"image callback"<<endl;
        rect_image=*imagePtr;

        if(!isInit )
        {
            //ROS_INFO("no tracking target object!");
            return;
        }
        boost::shared_ptr<void const> tracked_object;

        Mat im=cv_bridge::toCvShare(*imagePtr,tracked_object,imagePtr->encoding=="mono8"?"8UC1":"bgr8")->image;
//        imshow("imagecallback",im);
//        waitKey(5);

        if(im.empty()) return;
        Mat im_gray;
        if (im.channels() > 1) {
            cvtColor(im, im_gray, CV_BGR2GRAY);
        } else {
            im_gray = im;
        }

        cmt_.processFrame(im_gray);

        trackData.active_size=cmt_.points_active.size();
        trackData.center_x=cmt_.bb_rot.center.x;
        trackData.center_y=cmt_.bb_rot.center.y;
        trackData.width=cmt_.bb_rot.size.width;
        trackData.height=cmt_.bb_rot.size.height;
        trackData.angle=cmt_.bb_rot.angle;
        target_pub_.publish(trackData);

        FILE_LOG(logINFO)  << " active: " << cmt_.points_active.size();
        display(im,cmt_);
    }
    bool getImagecallback(mycommbase::GetImage::Request& req,mycommbase::GetImage::Response& rsp)
    {
        boost::mutex::scoped_lock lock(g_image_mutex_);
        ROS_INFO("getImagecallback be called ,get a rect image.");
        rsp.success=true;
        rsp.image=rect_image;
        return true;
    }

    bool initCallback(mycommbase::InitTrackObject::Request& req,mycommbase::InitTrackObject::Response& rsp)
    {

        boost::mutex::scoped_lock lock(g_image_mutex_);
        rect_=Rect(req.data.target_x,req.data.target_y,req.data.target_width,req.data.target_height);

        try
        {
            ROS_INFO("initcallback called. image encode is '%s' width %d height %d",
            req.data.image.encoding.c_str(),req.data.image.width,req.data.image.height);
            initImg=cv_bridge::toCvCopy(req.data.image,req.data.image.encoding=="mono8"?"8UC1":"bgr8")->image;

            Mat temp=initImg.clone();
            cv::rectangle(temp,rect_,Scalar(255,0,0));
            cv::imshow("target",temp);
            waitKey(5);
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s'.",req.data.image.encoding.c_str());
            return false;
        }

        //Convert im0 to grayscale
        Mat im0_gray;
        if (initImg.channels() > 1) {
            cvtColor(initImg, im0_gray, CV_BGR2GRAY);
        } else {
            im0_gray = initImg;
        }
        cmt_.initialize(im0_gray,rect_);
        isInit=true;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"cmt_tracker");
    TrackCMTRos cmt_trackObje;
    ros::spin();
    return 0;
}
