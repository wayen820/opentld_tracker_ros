
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <boost/thread.hpp>
#include <QDateTime>
#include <ctime>
#include <ros/ros.h>
#include <QTime>
#include "QTrackLabel.hpp"


QTrackLabel::QTrackLabel(QWidget* parent):
    QLabel(parent),islearning(false)
{

}
QTrackLabel::QTrackLabel(const QString &text, QWidget *parent):
    QLabel(parent),isold(true),islearning(false)
{
    setText(text);
}

void QTrackLabel::init( monitorui::QNode& node)
{
    startTimer(25);//每秒更新25Hz
    connect(&node,SIGNAL(processedImage(QImage*)),this,SLOT(updatePlayerUI(QImage*)));
    qRegisterMetaType<TrackingObject>("TrackingObject");
    connect(&node,SIGNAL(trackingObject(TrackingObject)),this,SLOT(updateTrackingObject(TrackingObject)));
    qnodePtr=&node;

}
//QTime t1;
void QTrackLabel::updatePlayerUI(QImage* img)
{
    boost::mutex::scoped_lock l(updater_mutex);
    //qDebug("%d",t1.elapsed());
    if(isold)//防止频繁的拷贝
    {
        img_=img->copy();

        isold=false;
    }
    //t1.start();
//    if(!img.isNull())
//    {
//        //setAlignment(Qt::AlignCenter);

        //setPixmap(QPixmap::fromImage(*img).scaled(this->size(),
                                //Qt::KeepAspectRatio,Qt::FastTransformation));
//    }
//    update();
}
//QTime tx;
void QTrackLabel::updateTrackingObject(TrackingObject rect)
{
    //qDebug("%d---rect x:%f,y:%f,w:%f,h:%f,a:%f,s:%d",tx.elapsed(),rect.center_x,rect.center_y,rect.width,rect.height,rect.angle,rect.active_size);
    boost::mutex::scoped_lock l(updater_mutex);
    Rect_=rect;

    //tx.start();
}

void QTrackLabel::mouseReleaseEvent(QMouseEvent *ev)
{
        //std::cout<<"mouseRelease "<<std::endl;
        isdown=false;
        islearning=false;
        qnodePtr->iniTrackObject(learningImg,QRect(startPoint,endPoint));
}

void QTrackLabel::mouseMoveEvent(QMouseEvent *event)
{
    if(isdown)
    {
        endPoint=event->pos();
        //std::cout<<startPoint.x()<<","<<startPoint.y()<<"|"<<endPoint.x()<<","<<endPoint.y()<<std::endl;

    }
}

void QTrackLabel::mousePressEvent(QMouseEvent *event)
{
    //std::cout<<"mousePress "<<std::endl;
    isdown=true;
    startPoint=event->pos();
    endPoint=startPoint;

    learningImg=qnodePtr->getStillImage();
    islearning=true;

}
    //QTime t;
void QTrackLabel::paintEvent(QPaintEvent *event)
{
    boost::mutex::scoped_lock l(updater_mutex);
    isold=true;

    //qDebug("%d",t.elapsed());


    QPainter painter(this);
    QPen pen(Qt::red,2,Qt::SolidLine);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    if(!islearning)
    {
        if(!img_.isNull())
            painter.drawPixmap(0,0,QPixmap::fromImage(img_));//.scaled(this->size(),Qt::KeepAspectRatio,Qt::FastTransformation));
    }
    else
    {
        if(!learningImg.isNull())
            painter.drawPixmap(0,0,QPixmap::fromImage(learningImg));//.scaled(this->size(),Qt::KeepAspectRatio,Qt::FastTransformation));
    }
    if(isdown)
        painter.drawRect(QRect(startPoint,endPoint));
    painter.drawRect(QRect(Rect_.center_x-Rect_.width/2,Rect_.center_y-Rect_.height/2,Rect_.width,Rect_.height));

    //qDebug("islearning=%s,learningImg=%s,img_=%s",islearning?"true":"false",learningImg.isNull()?"null":"ok",img_.isNull()?"null":"ok");
    //t.start();
}

void QTrackLabel::timerEvent(QTimerEvent *event)
{
    update();
    //setPixmap(QPixmap::fromImage(img_).scaled(this->size(),
                            //Qt::KeepAspectRatio,Qt::FastTransformation));
}
