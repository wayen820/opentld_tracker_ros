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
 * MainX.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#include "Main.h"

#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include "TLDUtil.h"
#include "Trajectory.h"


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
#include <sensor_msgs/fill_image.h>


using namespace tld;
using namespace cv;


bool Main::getImagecallback(mycommbase::GetImage::Request& req,mycommbase::GetImage::Response& rsp)
{
    boost::mutex::scoped_lock lock(g_image_mutex_);

    rsp.success=true;
    rsp.image=rect_image;
    return true;
}

bool Main::initCallback(mycommbase::InitTrackObject::Request& req,mycommbase::InitTrackObject::Response& rsp)
{

    boost::mutex::scoped_lock lock(g_image_mutex_);
    rect_=cv::Rect(req.data.target_x,req.data.target_y,req.data.target_width,req.data.target_height);

    try
    {
        ROS_INFO("initcallback called. image encode is '%s' width %d height %d",
        req.data.image.encoding.c_str(),req.data.image.width,req.data.image.height);
        initImg=cv_bridge::toCvCopy(req.data.image,req.data.image.encoding=="mono8"?"8UC1":"bgr8")->image;


//        cv::Mat temp=initImg.clone();
//        cv::rectangle(temp,rect_,cv::Scalar(255,0,0));
//        cv::imshow("target",temp);
//        cv::waitKey(5);
    }
    catch (cv::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s'.",req.data.image.encoding.c_str());
        return false;
    }
    havenewTarget=true;

    return true;
}

void Main::doWork()
{
	Trajectory trajectory;
    IplImage *img = imAcqGetImg(imAcq);
    Mat grey(img->height, img->width, CV_8UC1);
    Mat rgbMat_(img->height,img->width,CV_8UC3);

    cvtColor(cvarrToMat(img), grey, CV_BGR2GRAY);

    tld->detectorCascade->imgWidth = grey.cols;
    tld->detectorCascade->imgHeight = grey.rows;
    tld->detectorCascade->imgWidthStep = grey.step;

	if(showTrajectory)
	{
		trajectory.init(trajectoryLength);
	}

    if(selectManually)
    {

        CvRect box;

        if(getBBFromUser(img, box, gui) == PROGRAM_EXIT)
        {
            return;
        }

        if(initialBB == NULL)
        {
            initialBB = new int[4];
        }

        initialBB[0] = box.x;
        initialBB[1] = box.y;
        initialBB[2] = box.width;
        initialBB[3] = box.height;
    }

    FILE *resultsFile = NULL;

    if(printResults != NULL)
    {
        resultsFile = fopen(printResults, "w");
        if(!resultsFile)
        {
            fprintf(stderr, "Error: Unable to create results-file \"%s\"\n", printResults);
            exit(-1);
        }
    }

    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;

    if(loadModel && modelPath != NULL)
    {
        tld->readFromFile(modelPath);
        reuseFrameOnce = true;
    }
    else if(initialBB != NULL)
    {
        Rect bb = tldArrayToRect(initialBB);

        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);

        tld->selectObject(grey, &bb);
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }
    ros::Rate r(10);

    while(imAcqHasMoreFrames(imAcq) && ros::ok())
    {
        r.sleep();

        ros::spinOnce();

        double tic = cvGetTickCount();

        if(!reuseFrameOnce)
        {
            boost::mutex::scoped_lock lock(g_image_mutex_);

            cvReleaseImage(&img);
            img = imAcqGetImg(imAcq);

            if(img == NULL)
            {
                printf("current image is NULL, assuming end of input.\n");
                break;
            }

            cvtColor(cvarrToMat(img), grey, CV_BGR2GRAY);


            cvtColor(cvarrToMat(img),rgbMat_,CV_BGR2RGB);
            cv_bridge::CvImage img_bridge;
            sensor_msgs::Image im; // >> message to be sent

            std_msgs::Header header; // empty header
            //header.seq = counter; // user defined counter
            header.stamp = ros::Time::now(); // time
            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rgbMat_);
            img_bridge.toImageMsg(im); // from cv_bridge to sensor_msgs::Image


           // sensor_msgs::fillImage(im,sensor_msgs::image_encodings::RGB8,)
//            cv_bridge::CvImage cvi;
//            cvi.header.stamp=ros::Time::now();
//            cvi.header.frame_id="image";
//            cvi.encoding=sensor_msgs::image_encodings::RGB8;
//            cvi.image=cvarrToMat(img);

            //sensor_msgs::Image im;
            //cvi.toImageMsg(im);


            sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci->header.frame_id = im.header.frame_id;
            ci->header.stamp = im.header.stamp;

            image_pub_.publish(im,*ci);

            rect_image=im;
        }

        if(!skipProcessingOnce)
        {
            tld->processImage(cvarrToMat(img));
        }
        else
        {
            skipProcessingOnce = false;
        }

        if(printResults != NULL)
        {
            if(tld->currBB != NULL)
            {
                fprintf(resultsFile, "%d %.2d %.2d %.2d %.2d %f\n", imAcq->currentFrame - 1, tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
            }
            else
            {
                fprintf(resultsFile, "%d NaN NaN NaN NaN NaN\n", imAcq->currentFrame - 1);
            }
        }

        double toc = (cvGetTickCount() - tic) / cvGetTickFrequency();

        toc = toc / 1000000;

        float fps = 1 / toc;

        int confident = (tld->currConf >= threshold) ? 1 : 0;

        if(showOutput || saveDir != NULL)
        {
            char string[128];

            char learningString[10] = "";

            if(tld->learning)
            {
                strcpy(learningString, "Learning");
            }

            sprintf(string, "#%d,Posterior %.2f; fps: %.2f, #numwindows:%d, %s", imAcq->currentFrame - 1, tld->currConf, fps, tld->detectorCascade->numWindows, learningString);
            CvScalar yellow = CV_RGB(255, 255, 0);
            CvScalar blue = CV_RGB(0, 0, 255);
            CvScalar black = CV_RGB(0, 0, 0);
            CvScalar white = CV_RGB(255, 255, 255);
            if(tld->currBB!=NULL)
            {
                trackData.active_size=(int)tld->currConf;
                trackData.center_x=tld->currBB->x+tld->currBB->width/2;
                trackData.center_y=tld->currBB->y+tld->currBB->height/2;
                trackData.width=tld->currBB->width;
                trackData.height=tld->currBB->height;
                trackData.angle=0;
                trackData.active_size=(int)(tld->currConf*100);
                target_pub_.publish(trackData);
            }
            else
            {
                trackData.active_size=0;
                trackData.center_x=0;
                trackData.center_y=0;
                trackData.width=0;
                trackData.height=0;
                trackData.angle=0;
                trackData.active_size=0;
                target_pub_.publish(trackData);
            }
            if(tld->currBB != NULL)
            {
                CvScalar rectangleColor = (confident) ? blue : yellow;
                cvRectangle(img, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 8, 8, 0);

				if(showTrajectory)
				{
					CvPoint center = cvPoint(tld->currBB->x+tld->currBB->width/2, tld->currBB->y+tld->currBB->height/2);
					cvLine(img, cvPoint(center.x-2, center.y-2), cvPoint(center.x+2, center.y+2), rectangleColor, 2);
					cvLine(img, cvPoint(center.x-2, center.y+2), cvPoint(center.x+2, center.y-2), rectangleColor, 2);
					trajectory.addPoint(center, rectangleColor);
				}
            }
			else if(showTrajectory)
			{
				trajectory.addPoint(cvPoint(-1, -1), cvScalar(-1, -1, -1));
			}

			if(showTrajectory)
			{
				trajectory.drawTrajectory(img);
			}

            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1, 8);
            cvRectangle(img, cvPoint(0, 0), cvPoint(img->width, 50), black, CV_FILLED, 8, 0);
            cvPutText(img, string, cvPoint(25, 25), &font, white);

            if(showForeground)
            {

                for(size_t i = 0; i < tld->detectorCascade->detectionResult->fgList->size(); i++)
                {
                    Rect r = tld->detectorCascade->detectionResult->fgList->at(i);
                    cvRectangle(img, r.tl(), r.br(), white, 1);
                }

            }

            if(havenewTarget)
            {
                //cvtColor(initImg, grey, CV_BGR2GRAY);
                tld->selectObject(grey, &rect_);
                havenewTarget=false;
            }
            if(showOutput)
            {
                //gui->showImage(img);
                char key = gui->getKey();

                if(key == 'q') break;

                if(key == 'b')
                {

                    ForegroundDetector *fg = tld->detectorCascade->foregroundDetector;

                    if(fg->bgImg.empty())
                    {
                        fg->bgImg = grey.clone();
                    }
                    else
                    {
                        fg->bgImg.release();
                    }
                }

                if(key == 'c')
                {
                    //clear everything
                    tld->release();
                }

                if(key == 'l')
                {
                    tld->learningEnabled = !tld->learningEnabled;
                    printf("LearningEnabled: %d\n", tld->learningEnabled);
                }

                if(key == 'a')
                {
                    tld->alternating = !tld->alternating;
                    printf("alternating: %d\n", tld->alternating);
                }

                if(key == 'e')
                {
                    tld->writeToFile(modelExportFile);
                }

                if(key == 'i')
                {
                    tld->readFromFile(modelPath);
                }

                if(key == 'r')
                {
                    CvRect box;

                    if(getBBFromUser(img, box, gui) == PROGRAM_EXIT)
                    {
                        break;
                    }

                    Rect r = Rect(box);

                    tld->selectObject(grey, &r);
                }
            }

            if(saveDir != NULL)
            {
                char fileName[256];
                sprintf(fileName, "%s/%.5d.png", saveDir, imAcq->currentFrame - 1);

                cvSaveImage(fileName, img);
            }
        }

        if(reuseFrameOnce)
        {
            reuseFrameOnce = false;
        }
    }

    cvReleaseImage(&img);
    img = NULL;

    if(exportModelAfterRun)
    {
        tld->writeToFile(modelExportFile);
    }

    if(resultsFile)
    {
        fclose(resultsFile);
    }
}
