// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This example shows how to use the correlation_tracker from the dlib C++ library.  This
    object lets you track the position of an object as it moves from frame to frame in a
    video sequence.  To use it, you give the correlation_tracker the bounding box of the
    object you want to track in the current video frame.  Then it will identify the
    location of the object in subsequent frames.

    In this particular example, we are going to run on the video sequence that comes with
    dlib, which can be found in the examples/video_frames folder.  This video shows a juice
    box sitting on a table and someone is waving the camera around.  The task is to track the
    position of the juice box as the camera moves around.
*/
#include<time.h>

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include<opencv2/core/utility.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include <iostream>
#include <signal.h>
#include <termios.h>

using namespace cv;
//using namespace dlib;
using namespace std;

static Mat image;
static Rect boundingBox;
static bool paused;
static bool selectObject=false;
static bool startSelection=false;
static void onMouse(int event,int x,int y,int,void*)
{
    if(!selectObject)
    {
        switch(event)
        {
            case EVENT_LBUTTONDOWN:
                startSelection=true;
                boundingBox.x=x;
                boundingBox.y=y;
                break;
            case EVENT_LBUTTONUP:
                boundingBox.width=std::abs(x-boundingBox.x);
                boundingBox.height=std::abs(y-boundingBox.y);
                paused=false;
                selectObject=true;
            break;
        case EVENT_MOUSEMOVE:
            if(startSelection && !selectObject)
            {
                Mat currentFrame;
                image.copyTo(currentFrame);
                rectangle(currentFrame,Point(boundingBox.x,boundingBox.y),Point(x,y),Scalar(255,0,0),2,1);
                imshow("Tracking API",currentFrame);
            }
            break;
        }
    }
}
long getcurrentMill()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000+tv.tv_usec/1000;
}

bool compare1(dlib::file& f1,dlib::file& f2)
{
    int index1= f1.full_name().find_last_of('/');
    std::string name1= f1.full_name().substr(index1+1);
    int temp1= name1.find_first_of('.');
    std::string nameonly1=name1.substr(0,temp1);
    int namenum1=std::atoi(nameonly1.c_str());

    int index2= f2.full_name().find_last_of('/');
    std::string name2= f2.full_name().substr(index2+1);
    int temp2= name2.find_first_of('.');
    std::string nameonly2=name2.substr(0,temp2);
    int namenum2=std::atoi(nameonly2.c_str());

    return namenum1<namenum2;
}

int main(int argc, char** argv) try
{

    //CommandLineParser parser(argc,argv,"param error!");
    //String video_name=parser.get<String>(1);

//    if (video_name.empty())
//    {
//        cout << "Call this program like this: " << endl;
//        cout << "./video_tracking_ex /dev/video0" << endl;
//        return 1;
//    }
    cout<<"fuck!";
    if(argc!=3)
    {
        cout<<"param wrong!";
        return 0;
    }

    //Mat frame;
    paused=true;
    namedWindow("Tracking API",1);
    setMouseCallback("Tracking API",onMouse,0);

    dlib::correlation_tracker tracker;

     //Get the list of video frames.
    std::vector<dlib::file> files = dlib::get_files_in_directory_tree(argv[1], dlib::match_ending(argv[2]));
    std::sort(files.begin(), files.end(),compare1);
    if (files.size() == 0)
    {
        cout << "No images found in " << argv[1] << endl;
        return 1;
    }
    // Load the first frame.
    dlib::array2d<dlib::bgr_pixel> img;
    //dlib::cv_image<dlib::bgr_pixel> img;
    dlib::load_image(img, files[0]);

    Mat frame=dlib::toMat(img);


    frame.copyTo(image);
    imshow("Tracking API",image);

    long oldtime=getcurrentMill();
    int fps=0;
    bool initialized=false;
    int fileIndex=1;
    for(;;)
    {
        if(!paused)
        {
            fps++;
            if(getcurrentMill()>oldtime+1000)
            {
                cout<<fps<<" ";
                fps=0;
                oldtime=getcurrentMill();
                cout.flush();
            }
            //cout<<getcurrentMill()<<endl;
            
            //cap>>frame;

            //dlib::cv_image<dlib::bgr_pixel> cimg(frame);

            if(!initialized && selectObject)
            {
                tracker.start_track(img, dlib::drectangle(boundingBox.x,boundingBox.y,boundingBox.x+boundingBox.width,boundingBox.y+boundingBox.height));
                initialized=true;
            }
            else if(initialized)
            {
                if(fileIndex>=files.size())
                    break;
                dlib::load_image(img,files[fileIndex++]);
                frame=dlib::toMat(img);
                frame.copyTo(image);

                double rtn=tracker.update(img);
                if(rtn)
                {
                    Rect targRect;
                    targRect.x=tracker.get_position().left();
                    targRect.y=tracker.get_position().top();
                    targRect.width=tracker.get_position().width();
                    targRect.height=tracker.get_position().height();
                    cv::rectangle(image,targRect,Scalar(255,0,0),2,1);
                    std::stringstream ss;
                    ss<<rtn;
                    string rtnStr;
                    ss>>rtnStr;
                    std::cout<<rtn<<std::endl;
                    //cv::putText(image,rtnStr,targRect.br(),2,0.5,Scalar(255,0,0));
//                    time_t new_time= time(NULL);

//                    count++;
//                    time_t new_time= time(NULL);
//                    if(new_time>=timer+1)
//                    {
//                        cout<<count<<" ";
//                        timer=new_time;
//                        count=0;
//                    }
                }
                //usleep(500);
                //sleep(1);
            }
            imshow("Tracking API",image);
        }
        char c=(char)waitKey(2);
        if(c=='q')
            break;
        if(c=='p')
            paused=!paused;
    }
    return 0;
            // Get the list of video frames.
    //std::vector<file> files = dlib::get_files_in_directory_tree(argv[1], match_ending(".jpg"));
    //std::sort(files.begin(), files.end());
//    if (files.size() == 0)
//    {
//        cout << "No images found in " << argv[1] << endl;
//        return 1;
//    }
    // Load the first frame.  
    //dlib::array2d<unsigned char> img;
    //dlib::load_image(img, files[0]);
    // Now create a tracker and start a track on the juice box.  If you look at the first
    // frame you will see that the juice box is centered at pixel point(92,110) and 38
    // pixels wide and 86 pixels tall.
    //correlation_tracker tracker;
    //tracker.start_track(img, centered_rect(point(93,110), 38, 86));

    // Now run the tracker.  All we have to do is call tracker.update() and it will keep
    // track of the juice box!
//    image_window win;
//    for (unsigned long i = 1; i < files.size(); ++i)
//    {
//        load_image(img, files[i]);
//        tracker.update(img);

//        win.set_image(img);
//        win.clear_overlay();
//        win.add_overlay(tracker.get_position());

//        cout << "hit enter to process next frame" << endl;
//        cin.get();
//    }
}
catch (std::exception& e)
{
    cout << e.what() << endl;
}


