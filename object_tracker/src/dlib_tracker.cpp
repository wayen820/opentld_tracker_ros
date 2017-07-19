#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
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
#include <boost/thread.hpp>

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include<boost/format.hpp>
#include<time.h>

class TrackdlibRos
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

    dlib::correlation_tracker tracker;
    cv::Rect rect_;
    sensor_msgs::Image rect_image;
    cv::Mat initImg;
    int imageCount=0;

    long oldtime=getcurrentMill();
    int fps=0;

public:
    TrackdlibRos():nh_priv_("~"),isInit(false)
    {
        nh_priv_.param("image",image_topic_,std::string("image_raw"));
        nh_priv_.param("target",target_topic_,std::string("target"));

        srv_server_=nh_priv_.advertiseService<>("init_tracker",&TrackdlibRos::initCallback,this);
        getImage_server_=nh_priv_.advertiseService<>("get_image",&TrackdlibRos::getImagecallback,this);

        image_transport::ImageTransport itp(nh_priv_);
        camera_sub_=itp.subscribeCamera(image_topic_,1,&TrackdlibRos::OnImageCallBack,this);

        target_pub_=nh_priv_.advertise<mycommbase::TrackingData>(target_topic_,1);

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
    void OnImageCallBack(const sensor_msgs::ImageConstPtr& imagePtr,
                         const sensor_msgs::CameraInfoConstPtr& cameraInfoPtr)
    {
        boost::mutex::scoped_lock lock(g_image_mutex_);
        //std::cout<<"image callback"<<endl;
        rect_image=*imagePtr;



        if(!isInit)
        {
            //ROS_INFO("no tracking target object!");
            return;
        }
        boost::shared_ptr<void const> tracked_object;

        cv::Mat im=cv_bridge::toCvShare(*imagePtr,tracked_object,imagePtr->encoding=="mono8"?"8UC1":"bgr8")->image;
//        imshow("imagecallback",im);
//        waitKey(5);

//        boost::format formater("/home/liwei/work/images/office/%d.jpg");
//        formater%imageCount++;
//                std::cout<<"write to file!!----fuck"<<formater.str()<<std::endl;
//        cv::imwrite(formater.str(),im);


       //return;

        if(im.empty()) return;




        dlib::cv_image<dlib::bgr_pixel> cimg(im);
        if(tracker.update(cimg))
        {
            trackData.active_size=0;
            trackData.center_x=(tracker.get_position().left()+tracker.get_position().right())/2;
            trackData.center_y=(tracker.get_position().top()+tracker.get_position().bottom())/2;
            trackData.width=tracker.get_position().width();
            trackData.height=tracker.get_position().height();
            trackData.angle=0;
            target_pub_.publish(trackData);


            display(im,tracker);
        }

        fps++;
        if(getcurrentMill()>oldtime+1000)
        {
            std::cout<<fps<<" ";
            fps=0;
            oldtime=getcurrentMill();
            std::cout.flush();
        }

    }
    long getcurrentMill()
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        return tv.tv_sec*1000+tv.tv_usec/1000;
    }

    int display(cv::Mat im, dlib::correlation_tracker & tracker)
    {

        cv::rectangle(im,cv::Rect(tracker.get_position().left(),
                                  tracker.get_position().top(),
                                  tracker.get_position().width(),
                                  tracker.get_position().height()),cv::Scalar(255,0,0));




        imshow("view", im);

        return cv::waitKey(5);
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
        rect_=cv::Rect(req.data.target_x,req.data.target_y,req.data.target_width,req.data.target_height);

        try
        {
            ROS_INFO("initcallback called. image encode is '%s' width %d height %d",
            req.data.image.encoding.c_str(),req.data.image.width,req.data.image.height);
            initImg=cv_bridge::toCvCopy(req.data.image,req.data.image.encoding=="mono8"?"8UC1":"bgr8")->image;


            cv::Mat temp=initImg.clone();
            cv::rectangle(temp,rect_,cv::Scalar(255,0,0));
            cv::imshow("target",temp);
            cv::waitKey(5);
        }
        catch (cv::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s'.",req.data.image.encoding.c_str());
            return false;
        }

       dlib::cv_image<dlib::bgr_pixel> cimg(initImg);

       tracker.start_track(cimg, dlib::drectangle(rect_.x,rect_.y,rect_.x+rect_.width,rect_.y+rect_.height));
        isInit=true;
        return true;
    }
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"dlib_tracker");
    TrackdlibRos tracker;
    ros::spin();
    return 0;
}
