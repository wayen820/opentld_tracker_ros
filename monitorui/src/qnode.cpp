/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/


#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/monitorui/qnode.hpp"
#include <sensor_msgs/fill_image.h>
#include <QTime>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monitorui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"monitorui");
	if ( ! ros::master::check() ) {
		return false;
	}


    ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
    ros::NodeHandle n_priv("~");
    n_priv.param("getimageservice",getImage_service_name,std::string("getimage"));
    n_priv.param("initservice",init_service_name,std::string("initTrack"));
    n_priv.param("trackobject",trackObjectTopic,std::string("trackobject"));
    n_priv.param("camera",camera_topic,std::string("camera"));
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    image_transport::ImageTransport it=image_transport::ImageTransport(n);
    camera_sub_=it.subscribeCamera(camera_topic,1,&QNode::OnImageCallBack,this);
    trackObject_sub=n.subscribe(trackObjectTopic,1,&QNode::OnTrackObjectCallback,this);

    getImage_client_=n.serviceClient<mycommbase::GetImage>(getImage_service_name);
    init_client_=n.serviceClient<mycommbase::InitTrackObject>(init_service_name);

	start();
	return true;
}


bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"monitorui");
	if ( ! ros::master::check() ) {
		return false;
	}

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    ros::NodeHandle n_priv("~");
    n_priv.param("getimageservice",getImage_service_name,std::string("get_image"));
    n_priv.param("initservice",init_service_name,std::string("init_tracker"));
    n_priv.param("trackobject",trackObjectTopic,std::string("target"));
    n_priv.param("camera",camera_topic,std::string("camera"));

    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    image_transport::ImageTransport it=image_transport::ImageTransport(n_priv);
    camera_sub_=it.subscribeCamera(camera_topic,1,&QNode::OnImageCallBack,this);
    trackObject_sub=n.subscribe(trackObjectTopic,1,&QNode::OnTrackObjectCallback,this);

    getImage_client_=n.serviceClient<mycommbase::GetImage>(getImage_service_name);
    init_client_=n.serviceClient<mycommbase::InitTrackObject>(init_service_name);
	start();
	return true;
}
void QNode::OnTrackObjectCallback(const mycommbase::TrackingDataPtr& target)
{
    trackObjectRect_.active_size=target->active_size;
    trackObjectRect_.angle=target->angle;
    trackObjectRect_.center_x=target->center_x;
    trackObjectRect_.center_y=target->center_y;
    trackObjectRect_.height=target->height;
    trackObjectRect_.width=target->width;
    Q_EMIT trackingObject(trackObjectRect_);
}
//QTime t2;
void QNode::OnImageCallBack(const sensor_msgs::ImageConstPtr& imagePtr,
                            const sensor_msgs::CameraInfoConstPtr& cameraInfoPtr)
{
    //std::cout<<"monitorUI image callback!";
    //std::cout<<(imagePtr->header.stamp==cameraInfoPtr->header.stamp)<<std::endl;
    //qDebug("%d",t2.elapsed());
    boost::mutex::scoped_lock lock(update_mutex);

//    QImage img=QImage((const uchar*)(imagePtr->data.data()),
//                      imagePtr->width,imagePtr->height,QImage::Format_Indexed8);
//    QImage::Format format;
//    if(imagePtr->encoding=="mono8")
//    {
//        format=QImage::Format_Indexed8;
//    }
//    else if(imagePtr->encoding=="rgb8")
//    {
//        format=QImage::Format_RGB888;
//    }
//    else if(imagePtr->encoding=="bgr8")
//    {
//        format=QImage::Format_
//    }
    QImage img=QImage((const uchar*)(imagePtr->data.data()),
                      imagePtr->width,imagePtr->height,imagePtr->encoding=="mono8"?QImage::Format_Indexed8: QImage::Format_RGB888);

    imgcpy_=img.copy();
    Q_EMIT processedImage(&imgcpy_);

    //t2.start();
}
void QNode::iniTrackObject(const QImage &image, const QRect &rect)
{
    mycommbase::InitTrackObject srv;
    srv.request.data.target_x=rect.x();
    srv.request.data.target_y=rect.y();
    srv.request.data.target_width=rect.width();
    srv.request.data.target_height=rect.height();
    sensor_msgs::Image img;

//    sensor_msgs::fillImage(img,"mono8",image.height(),image.width(),image.width(),image.constBits());
    sensor_msgs::fillImage(img,image.format()==QImage::Format_Indexed8?"mono8":"rgb8",image.height(),image.width(),image.width()*image.depth()/8,image.constBits());
    srv.request.data.image=img;
    if(init_client_.call(srv))
    {
        ROS_INFO("iniTrackObject成功！");
    }
    else
    {
        ROS_WARN("iniTrackObject失败！");
    }
}


QImage QNode::getStillImage()
{
    std::cout<<"monitorui:getStillImage() called! service name:"<<getImage_client_.getService()<<std::endl;
    //getImage_client_.getService()
    mycommbase::GetImage srv;
    srv.request.timeout=ros::Duration(1,0);
    if(getImage_client_.call(srv))
    {
//        QImage img=QImage((const uchar*)(srv.response.image.data.data()),
//                          (int)srv.response.image.width,(int)srv.response.image.height,QImage::Format_Indexed8);
        QImage img=QImage((const uchar*)(srv.response.image.data.data()),
                          (int)srv.response.image.width,(int)srv.response.image.height,srv.response.image.encoding=="mono8"?QImage::Format_Indexed8: QImage::Format_RGB888);
        imgStill_=img.copy();

    }
    else
    {
        imgStill_=QImage();
        ROS_WARN("getStillImage 调用失败！");
    }
    return imgStill_;
}
void QNode::run() {
    ros::spin();
//    ros::Rate loop_rate(100);
//	int count = 0;
//	while ( ros::ok() ) {

//		std_msgs::String msg;
//		std::stringstream ss;
//		ss << "hello world " << count;
//		msg.data = ss.str();
//		chatter_publisher.publish(msg);
//		log(Info,std::string("I sent: ")+msg.data);
//		ros::spinOnce();
//		loop_rate.sleep();
//		++count;
//	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



}  // namespace monitorui
