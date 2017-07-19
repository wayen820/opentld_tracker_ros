/**
 * @file /include/monitorui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef monitorui_QNODE_HPP_
#define monitorui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QImage>
#include <QRect>
#include <boost/thread.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include "mycommbase/GetImage.h"
#include "mycommbase/InitTrackObject.h"
#include "mycommbase/TrackingData.h"
#include <ctype.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace monitorui {

/*****************************************************************************
** Class
*****************************************************************************/
struct TrackingObject
{
    float center_x;
    float center_y;
    float width;
    float height;
    float angle;
    uint active_size;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    void iniTrackObject(const QImage& image,const QRect& rect);//初始化跟踪对象
    QImage getStillImage();//获取静态图像，拍照
    //boost::shared_ptr<QImage> getStreamImage();
    //QRect getTrackObject();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void processedImage(QImage* imagePtr);//收到图像
    void trackingObject(TrackingObject rect);//收到跟踪结果
private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    boost::mutex update_mutex;
    QImage imgcpy_;  //收到的图片
    QImage imgStill_; //收到的静态图片
    sensor_msgs::Image imgsnsStill_;
    TrackingObject trackObjectRect_;  //追踪的结果


    image_transport::CameraSubscriber camera_sub_;
    ros::ServiceClient getImage_client_;
    ros::ServiceClient init_client_;
    ros::Subscriber trackObject_sub;
    std::string getImage_service_name,init_service_name,trackObjectTopic,camera_topic;


private:
    void OnImageCallBack(const sensor_msgs::ImageConstPtr& imagePtr,const sensor_msgs::CameraInfoConstPtr& cameraInfoPtr);
    void OnTrackObjectCallback(const mycommbase::TrackingDataPtr& target);
};

}  // namespace monitorui

#endif /* monitorui_QNODE_HPP_ */
