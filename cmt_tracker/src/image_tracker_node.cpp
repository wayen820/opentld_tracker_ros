#include<opencv2/core/utility.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include <iostream>

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
int main(int argc,char** argv)
{

    return 0;
}
