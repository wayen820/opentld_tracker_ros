/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/*
 * main.h
 *
 *  Created on: Nov 18, 2011
 *      Author: Georg Nebehay
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "TLD.h"
#include "ImAcq.h"
#include "Gui.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "mycommbase//InitTrackObject.h"
#include "mycommbase/GetImage.h"

#include "mycommbase/TargetData.h"
#include "mycommbase/TrackingData.h"
#include <boost/thread.hpp>

enum Retval
{
    PROGRAM_EXIT = 0,
    SUCCESS = 1
};

class Main
{
public:
    tld::TLD *tld;
    ImAcq *imAcq;
    tld::Gui *gui;
    bool showOutput;
	bool showTrajectory;
	int trajectoryLength;
    const char *printResults;
    const char *saveDir;
    double threshold;
    bool showForeground;
    bool showNotConfident;
    bool selectManually;
    int *initialBB;
    bool reinit;
    bool exportModelAfterRun;
    bool loadModel;
    const char *modelPath;
    const char *modelExportFile;
    int seed;

    int camNo;

    // private ROS node handle
    ros::NodeHandle node_;

    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    ros::ServiceServer srv_server_;
    ros::ServiceServer getImage_server_;
    ros::Publisher target_pub_;
    //mycommbase::TargetData targetData;
    mycommbase::TrackingData trackData;
    // parameters
    std::string video_device_name_, camera_name_, camera_info_url_;
    std::string target_topic_;
    //std::string start_service_name_, start_service_name_;
    boost::mutex g_image_mutex_;
    sensor_msgs::Image rect_image;
    //sensor_msgs::Image img_;
    cv::Rect rect_;
    cv::Mat initImg;
    bool havenewTarget;
    Main():
        node_("~"),havenewTarget(false)
    {
        tld = new tld::TLD();
        showOutput = 1;
        printResults = NULL;
        saveDir = ".";
        threshold = 0.5;
        showForeground = 0;

		showTrajectory = false;
		trajectoryLength = 0;

        selectManually = 0;

        initialBB = NULL;
        showNotConfident = true;

        reinit = 0;

        loadModel = false;

        exportModelAfterRun = false;
        modelExportFile = "model";
        seed = 0;

        gui = NULL;
        modelPath = NULL;
        imAcq = NULL;

        // advertise the main image topic

        // load the camera info
        node_.param("camera_frame_id", img_.header.frame_id, std::string("usb_camera"));
        node_.param("camera_name", camera_name_, std::string("head_camera"));
        node_.param("camera_info_url", camera_info_url_, std::string(""));
        node_.param("target",target_topic_,std::string("trackingObject"));
        node_.param("cameraNo",camNo,0);

        image_transport::ImageTransport it(node_);
        image_pub_ = it.advertiseCamera("image_raw", 1);
        srv_server_=node_.advertiseService<>("init_tracker",&Main::initCallback,this);
        getImage_server_=node_.advertiseService<>("get_image",&Main::getImagecallback,this);

        target_pub_=node_.advertise<mycommbase::TrackingData>(target_topic_,1);

        cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));
        // check for default camera info
        if (!cinfo_->isCalibrated())
        {
          cinfo_->setCameraName(video_device_name_);
          sensor_msgs::CameraInfo camera_info;
          camera_info.header.frame_id = img_.header.frame_id;
          camera_info.width = 640;
          camera_info.height = 480;
          cinfo_->setCameraInfo(camera_info);
        }
    }

    ~Main()
    {
        delete tld;
        imAcqFree(imAcq);
    }

    void doWork();
        bool initCallback(mycommbase::InitTrackObject::Request& req,mycommbase::InitTrackObject::Response& rsp);
            bool getImagecallback(mycommbase::GetImage::Request& req,mycommbase::GetImage::Response& rsp);
};

#endif /* MAIN_H_ */
