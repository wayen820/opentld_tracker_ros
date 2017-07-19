#include <ros/ros.h>
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
#include "TLD.h"

using cv::Mat;
using cv::Rect;


class TLDTrackerRos
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
    bool istldetectorInit;

    Rect rect_;
    Mat initImg;
    ros::Time lastInitWarning;
    sensor_msgs::Image rect_image;
private:
    tld::TLD* _tld;
    double threshold;
    Mat init_grey;
public:
    TLDTrackerRos():nh_priv_("~"),isInit(false),istldetectorInit(false)
    {

        nh_priv_.param("image",image_topic_,std::string("/usb_cam/image_raw"));
        nh_priv_.param("target",target_topic_,std::string("trackingObject"));

        srv_server_=nh_priv_.advertiseService<>("init_tracker",&TLDTrackerRos::initCallback,this);
        getImage_server_=nh_priv_.advertiseService<>("get_image",&TLDTrackerRos::getImagecallback,this);

        image_transport::ImageTransport itp(nh_priv_);
        camera_sub_=itp.subscribeCamera(image_topic_,1,&TLDTrackerRos::OnImageCallBack,this);

        target_pub_=nh_priv_.advertise<mycommbase::TrackingData>(target_topic_,1);

        inittld();

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
    void inittld()
    {
        _tld=new tld::TLD();

        _tld->trackerEnabled=true;
        _tld->alternating=false;
        _tld->learningEnabled=true;


        tld::DetectorCascade *detectorCascade=_tld->detectorCascade;
        detectorCascade->varianceFilter->enabled=true;
        detectorCascade->ensembleClassifier->enabled=true;
        detectorCascade->nnClassifier->enabled=true;
        detectorCascade->useShift=true;
        detectorCascade->shift=0.1;
        detectorCascade->minScale = -10;
        detectorCascade->maxScale = 10;
        detectorCascade->minSize = 25;
        detectorCascade->numTrees = 10;
        detectorCascade->numFeatures = 13;
        detectorCascade->nnClassifier->thetaTP = 0.65;
        detectorCascade->nnClassifier->thetaFP = 0.5;
        threshold=0.7;

        srand(0);
    }

    ~TLDTrackerRos()
    {
        delete _tld;
    }

    void OnImageCallBack(const sensor_msgs::ImageConstPtr& imagePtr,
                         const sensor_msgs::CameraInfoConstPtr& cameraInfoPtr)
    {
        boost::mutex::scoped_lock lock(g_image_mutex_);
        //std::cout<<"image callback"<<endl;
        rect_image=*imagePtr;



//        if(!isInit )
//        {
//            //ROS_INFO("no tracking target object!");
//            return;
//        }
        double tic = cvGetTickCount();

        boost::shared_ptr<void const> tracked_object;

        Mat im=cv_bridge::toCvShare(*imagePtr,tracked_object,imagePtr->encoding=="mono8"?"8UC1":"bgr8")->image;
//        imshow("imagecallback",im);
//        cv::waitKey(5);
//        return;

//        if(im.empty()) return;
//        Mat im_gray;
//        if (im.channels() > 1) {
//            cvtColor(im, im_gray, CV_BGR2GRAY);
//        } else {
//            im_gray = im;
//        }

                imshow("imagecallback",im);
                cv::waitKey(5);
//                return;

        if(!istldetectorInit)
        {
            Mat im0_gray;
            if (im.channels() > 1) {
                cvtColor(im, im0_gray, CV_BGR2GRAY);
            } else {
                im0_gray = im;
            }
            _tld->detectorCascade->imgWidth=im0_gray.cols;
            _tld->detectorCascade->imgWidth=im0_gray.rows;
            _tld->detectorCascade->imgWidthStep=im0_gray.step;

            istldetectorInit=true;
        }


        _tld->processImage(im);

        double toc = (cvGetTickCount() - tic) / cvGetTickFrequency();

        toc = toc / 1000000;

        float fps = 1 / toc;

        std::cout<<fps<<" "<<std::endl;

        if(_tld->currBB!=NULL)
        {
            trackData.active_size=(int)_tld->currConf;
            trackData.center_x=_tld->currBB->x+_tld->currBB->width/2;
            trackData.center_y=_tld->currBB->y+_tld->currBB->height/2;
            trackData.width=_tld->currBB->width;
            trackData.height=_tld->currBB->height;
            trackData.angle=0;


        }
        else
        {
            trackData.active_size=0;
            trackData.center_x=0;
            trackData.center_y=0;
            trackData.width=0;
            trackData.height=0;
            trackData.angle=0;
        }
        target_pub_.publish(trackData);
        //cmt_.processFrame(im_gray);

//        trackData.active_size=cmt_.points_active.size();
//        trackData.center_x=cmt_.bb_rot.center.x;
//        trackData.center_y=cmt_.bb_rot.center.y;
//        trackData.width=cmt_.bb_rot.size.width;
//        trackData.height=cmt_.bb_rot.size.height;
//        trackData.angle=cmt_.bb_rot.angle;
//        target_pub_.publish(trackData);

        //FILE_LOG(logINFO)  << " active: " << cmt_.points_active.size();
        //display(im,cmt_);
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
            cv::rectangle(temp,rect_,cv::Scalar(255,0,0));
            cv::imshow("target",temp);
            cv::waitKey(5);

        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s'.",req.data.image.encoding.c_str());
            return false;
        }

        //Convert im0 to grayscale
        //Mat im0_gray;
        if (initImg.channels() > 1) {
            cvtColor(initImg, init_grey, CV_BGR2GRAY);
        } else {
            init_grey = initImg;
        }
//        _tld->detectorCascade->imgWidth=im0_gray.cols;
//        _tld->detectorCascade->imgWidth=im0_gray.rows;
//        _tld->detectorCascade->imgWidthStep=im0_gray.step;

        _tld->selectObject(init_grey,&rect_);


        //cmt_.initialize(im0_gray,rect_);
        isInit=true;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"opentld_node");
    TLDTrackerRos tracker;
    ros::spin();
    return 0;
}
