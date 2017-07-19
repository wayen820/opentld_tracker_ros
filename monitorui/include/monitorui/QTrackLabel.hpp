#ifndef _QLABEL__H
#define _QLABEL__H
#include <QLabel>
#include "qnode.hpp"
using namespace monitorui;

class QTrackLabel:public QLabel
{
Q_OBJECT
public:
    explicit QTrackLabel(QWidget* parent=0);
    QTrackLabel(const QString& text,QWidget* parent=0);
    void init( monitorui::QNode& node);
Q_SIGNALS:
    void clicked();
protected:
    void mouseReleaseEvent(QMouseEvent*) ;
    void mouseMoveEvent(QMouseEvent *event) ;
    void mousePressEvent(QMouseEvent* event) ;
    void paintEvent(QPaintEvent* event) ;
     void timerEvent(QTimerEvent *event);
private:
    bool isdown;
    QPoint startPoint,endPoint;
     monitorui::QNode* qnodePtr;
    QImage img_;
    monitorui::TrackingObject Rect_;
    bool islearning;
    QImage learningImg;

    boost::mutex updater_mutex;
    bool isold;
public Q_SLOTS:
    void updatePlayerUI(QImage* img);
    void updateTrackingObject(TrackingObject rect);
};

#endif
