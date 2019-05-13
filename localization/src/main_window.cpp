/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include "../include/localization/main_window.hpp"
#include "../include/localization/myitem.h"
#include "../include/localization/robitvision.hpp"
#include "../include/localization/robit_master_vision.h"
#include "../include/localization/robitparticlefilter.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/
#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

#define ROBIT               17      //37


namespace localization {

using namespace Qt;
using namespace cv;
using namespace std;


RobitParticleFilter localization;

extern double panAngle;
extern double tiltAngle;
extern double yaw;

extern ros::Publisher localPub;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();

    fieldW = 700; fieldH = 500;         //ROBIT Field W = 700, H = 500

    localization = RobitParticleFilter(fieldW, fieldH, 2000, 6);

    //--Draw&init Field
    drawField();
    initField();

    isSet = false;
    isUpdate = false;
    step = false;

    robot = new MyItem();
    ball = new MyBall();
    mTimer = new QTimer(this);

    t = 0;

    QObject::connect(&qnode, SIGNAL(recvImu()), this, SLOT(updateYaw()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(step()), this, SLOT(updateOdometry()));
    QObject::connect(&qnode, SIGNAL(localUpdate()), this, SLOT(visionCallback()));
    QObject::connect(mTimer, SIGNAL(timeout()), this, SLOT(update()));
}

void MainWindow::on_pushButton_set_clicked()
{
    if(!isSet)
    {
        isSet = true;
        localization.addParticles(cv::Point(nowX, nowY));

        cout << localization.getParticles().size() << endl;
        for(size_t s = 0; s < localization.getParticles().size(); s++)
        {
            int x = localization.getParticles()[s].x;
            int y = localization.getParticles()[s].y;

            transform(x, y);

//            scene->addEllipse(x, y, 1, 1, QPen(Qt::red));
        }

        for(int i = 0; i < 2; i++)  // xcross
        {
            int x = fieldW / 2, y = fieldH * (3 + 2 * i)/ 8;
            localization.addFeatures(Point(x, y));

            transform(x, y);
            scene->addEllipse(x, y, 5, 5, QPen(Qt::red));
        }

        for(int i = 0; i < 4; i++)  // goalpost
        {
            int x = fieldW * (i / 2), y = (fieldH / 2 - 130) + 260 * (i % 2);
            localization.addFeatures(Point(x, y));

            transform(x, y);
            scene->addEllipse(x, y, 5, 5, QPen(Qt::red));
        }

        localization.createSamples();

//        srand(time(NULL));

//        int randP = rand() % 2000;

//        for(int i = 0; i < 6; i++)
//        {
//            double temp1 = localization.getSamples()[randP][i].dist;
//            double temp2 = localization.getSamples()[randP][i].theta;

//            cout << "random sample d = " << temp1 << endl;
//            cout << "random sample th = " << temp2 << endl;
//        }

//        int x = localization.getParticles()[randP].x;
//        int y = localization.getParticles()[randP].y;

//        transform(x, y);

//        scene->addEllipse(x, y, 1, 1, QPen(Qt::red));

        mTimer->start(1000);

        ui.lineEdit_time->setText(QString::number(t));

        scene->addItem(ball);
    }

    datumYaw = yaw;
}

void MainWindow::on_pushButton_reset_clicked()
{
    scene->removeItem(robot);
    nowX = 0; nowY = 0;

    if(isSet)
    {
        isSet = false;

        mTimer->stop();
        t = 0;

        ui.lineEdit_time->clear();

        drawField();
    }
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if(!isSet)
    {
        scene->addItem(robot);
        QPoint remapped = ui.graphicsView->mapFromParent(event->pos());
        if(ui.graphicsView->rect().contains(remapped))
        {
            QPointF mousePoint = ui.graphicsView->mapToScene(remapped);

            nowX = mousePoint.x();
            nowY = mousePoint.y();

            pastX = mousePoint.x();
            pastY = mousePoint.y();

            cout<<"X = "<<mousePoint.x()<<"  Y = "<<mousePoint.y()<<endl;


            robot->setPos(nowX, nowY);

            nowX += fieldW / 2;
            nowY += fieldH / 2;

            pastX += fieldW / 2;
            pastY += fieldH / 2;

            cout << "now X = " << nowX << ", now Y = " << nowY << endl;

            ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));
        }
    }
}

void MainWindow::updateYaw()
{
    currentYaw = yaw - datumYaw;
    if(currentYaw > 180) currentYaw -= 360;
    else if(currentYaw <= -180) currentYaw += 360;

    if(isSet) robot->setRotation(-currentYaw);
}

void MainWindow::updateOdometry()
{
    if(isSet && !isUpdate)
    {
        cout << "step = " << step << endl;
        if(!step)
        {
            double x = 0, y = 0;
            double f = 0.9999999;

            x = qnode.Xmoved * cos(currentYaw * DEG2RAD) + qnode.Ymoved * sin(currentYaw * DEG2RAD);
            y = qnode.Xmoved * sin(currentYaw * DEG2RAD) - qnode.Ymoved * cos(currentYaw * DEG2RAD);

            if(x < 0)
            {
                f = -0.9999999;
                x = (int)(x + f);
            }
            else
            {
                f = 0.9999999;
                x = (int)(x + f);
            }

            if(y < 0)
            {
                f = -0.9999999;
                y = (int)(y + f);
            }
            else
            {
                f = 0.9999999;
                y = (int)(y + f);
            }

            cv::Point odometry(x, y);

//            int tempX = odometry.x, tempY = odometry.y;

//            odometry.x += sqrt(abs(qnode.Xmoved * qnode.Xmoved - tempY * tempY));
//            odometry.y += sqrt(abs(qnode.Xmoved * qnode.Xmoved - tempX * tempX));

//            odometry.x += sqrt(abs(qnode.Ymoved * qnode.Ymoved - tempY * tempY));
//            odometry.y += sqrt(abs(qnode.Ymoved * qnode.Ymoved - tempX * tempX));

            RobitParticleFilter::transform(odometry);

            cout << "odometry x = " << odometry.x << endl;
            cout << "odometry y = " << odometry.y << endl;

            nowX += odometry.x;
            nowY += odometry.y;

            pastX += odometry.x;
            pastY += odometry.y;

            dX += odometry.x;
            dY += odometry.y;

            vector<cv::Point> p = localization.getParticles();

            for(size_t s = 0; s < p.size(); s++)
            {
//                p[s].x += odometry.x;
//                p[s].y += odometry.y;

                p[s] += odometry;
            }
        }

//        step ^= 1;
    }
}

void MainWindow::visionCallback()
{
    if(isSet && !isUpdate)
    {
//        cout << "vision update" << endl;
        vector<ObjectPos> KPoints = vector<ObjectPos>(6);
        vector<ObjectPos> nowPoints;

        vector<cv::Point> f = localization.getFeatures();
        vector<cv::Point> tf = localization.getTranstFeatures();

//        cout << "feature size = " << f.size() << endl;
//        cout << "transfeature size = " << tf.size() << endl;

        for(size_t s = 0; s < f.size(); s++)
        {
            cv::Point nowPt(nowX, nowY); ObjectPos temp;
            temp.dist = cv::norm(nowPt - f[s]) * 10;

            RobitParticleFilter::transform(nowPt);
            temp.theta = atan2(tf[s].y - nowPt.y, tf[s].x - nowPt.x) * RAD2DEG;

            nowPoints.push_back(temp);
        }

        for(int i = 0; i < qnode.localMsg.xcrossTheta.size(); i++)
        {
            double xcrossTh = qnode.localMsg.xcrossTheta[i] + currentYaw + panAngle;

            if(xcrossTh > 180) xcrossTh -= 360;
            else if(xcrossTh <= -180) xcrossTh += 360;

            int minIdx = -1; double minDiff = 360;
            for(int j = 0; j < 2; j++)
            {
                double diff = fabs(xcrossTh - nowPoints[j].theta);
                if(diff < minDiff)
                {
                    minDiff = diff;
                    minIdx = j;
                }
            }

            if(minIdx != -1)
            {
                KPoints[minIdx].dist = qnode.localMsg.xcrossDist[i];
                KPoints[minIdx].theta = xcrossTh;

//                if(KPoints[minIdx].theta > 180) KPoints[minIdx].theta -= 360;
//                else if(KPoints[minIdx].theta <= -180) KPoints[minIdx].theta += 360;
            }
        }

        for(int i = 0; i < qnode.localMsg.goalpostTheta.size(); i++)
        {
            double goalpostTh = qnode.localMsg.goalpostTheta[i] + currentYaw + panAngle;

            if(goalpostTh > 180) goalpostTh -= 360;
            else if(goalpostTh <= -180) goalpostTh += 360;

            if(goalpostTh > 0)
            {
                int minIdx = -1; double minDiff = 360;
                for(int j = 2; j < 4; j++)
                {
                    double diff = fabs(goalpostTh - nowPoints[j].theta);
                    if(diff < minDiff)
                    {
                        minDiff = diff;
                        minIdx = j;
                    }
                }

                if(minIdx != -1)
                {
                    KPoints[minIdx].dist = qnode.localMsg.goalpostDist[i];
                    KPoints[minIdx].theta = goalpostTh;

//                    if(KPoints[minIdx].theta > 180) KPoints[minIdx].theta -= 360;
//                    else if(KPoints[minIdx].theta <= -180) KPoints[minIdx].theta += 360;
                }
            }
            else
            {
                int minIdx = -1; double minDiff = 360;
                for(int j = 4; j < 6; j++)
                {
                    double diff = fabs(goalpostTh - nowPoints[j].theta);
                    if(diff < minDiff)
                    {
                        minDiff = diff;
                        minIdx = j;
                    }
                }

                if(minIdx != -1)
                {
                    KPoints[minIdx].dist = qnode.localMsg.goalpostDist[i];
                    KPoints[minIdx].theta = goalpostTh;

//                    if(KPoints[minIdx].theta > 180) KPoints[minIdx].theta -= 360;
//                    else if(KPoints[minIdx].theta <= -180) KPoints[minIdx].theta += 360;
                }
            }
        }

        localization.calcWeights(KPoints, cv::Point(nowX, nowY));

        if(qnode.localMsg.ballDist && qnode.localMsg.ballTheta)
        {
            double ballTh = qnode.localMsg.ballTheta + currentYaw + panAngle;

            if(ballTh > 180) ballTh -= 360;
            else if(ballTh <= -180) ballTh += 360;

            ballX = qnode.localMsg.ballDist * cos(ballTh * DEG2RAD) / 10;
            ballY = qnode.localMsg.ballDist * sin(ballTh * DEG2RAD) / 10;


            cv::Point ballPos(ballX, ballY);

            RobitParticleFilter::transform(ballPos);

//            cout << "ball x = " << nowX + ballPos.x - fieldW / 2;
//            cout << ", ball y = " << nowY + ballPos.y - fieldH / 2 << endl;

            scene->addItem(ball);
            ball->setPos(nowX + ballPos.x - fieldW / 2, nowY + ballPos.y - fieldH / 2);
            ui.textEdit_area_2->setText(QString::number(Where_is_Robot(nowX + ballPos.x, nowY + ballPos.y)));
            ball_pos.x = nowX + ballPos.x;
            ball_pos.y = nowY + ballPos.y;
        }

//        cout << "nowPt.dist [" << nowPoints[0].dist;
//        cout << ", " << nowPoints[1].dist;
//        cout << ", " << nowPoints[2].dist;
//        cout << ", " << nowPoints[3].dist;
//        cout << ", " << nowPoints[4].dist;
//        cout << ", " << nowPoints[5].dist << " ]" << endl;

//        cout << "nowPt.theta [" << nowPoints[0].theta;
//        cout << ", " << nowPoints[1].theta;
//        cout << ", " << nowPoints[2].theta;
//        cout << ", " << nowPoints[3].theta;
//        cout << ", " << nowPoints[4].theta;
//        cout << ", " << nowPoints[5].theta << " ]" << endl;

//        cout << "KPoints.dist [" << KPoints[0].dist;
//        cout << ", " << KPoints[1].dist;
//        cout << ", " << KPoints[2].dist;
//        cout << ", " << KPoints[3].dist;
//        cout << ", " << KPoints[4].dist;
//        cout << ", " << KPoints[5].dist << " ]" << endl;

//        cout << "KPoints.theta [" << KPoints[0].theta;
//        cout << ", " << KPoints[1].theta;
//        cout << ", " << KPoints[2].theta;
//        cout << ", " << KPoints[3].theta;
//        cout << ", " << KPoints[4].theta;
//        cout << ", " << KPoints[5].theta << " ]" << endl;
    }
}

void MainWindow::update()
{
    isUpdate = true;

    msg_generate::position_msg posMsg;

    drawField();
    ui.textEdit_area->clear();

    for(size_t s = 0; s < localization.getParticles().size(); s++)
    {
        int x = localization.getParticles()[s].x;
        int y = localization.getParticles()[s].y;

        transform(x, y);

        scene->addEllipse(x, y, 1, 1, QPen(Qt::red));
    }

    scene->addItem(robot);

    int p = localization.update();

    if(p != -1)
    {
        nowX = localization.getParticles()[p].x;
        nowY = localization.getParticles()[p].y;

//        pastX += dX;
//        pastY += dY;

        if(cv::norm(Point(nowX, nowY) - Point(pastX, pastY)) > 100)
            nowX = pastX; nowY = pastY;
    }
    else
        cout << "p = " << p << endl;

    robot->setPos(nowX - fieldW / 2, nowY - fieldH / 2);
    ui.textEdit_area->setText(QString::number(Where_is_Robot(nowX, nowY)));

//    if(!qnode.localMsg.ballDist && !qnode.localMsg.ballTheta)
//    {
//        double ballTh = qnode.localMsg.ballTheta + currentYaw + panAngle;

//        if(ballTh > 180) ballTh -= 360;
//        else if(ballTh <= -180) ballTh += 360;

//        ballX = qnode.localMsg.ballDist * cos(ballTh * DEG2RAD);
//        ballY = qnode.localMsg.ballDist * sin(ballTh * DEG2RAD);

//        cv::Point ballPos(ballX, ballY);

//        RobitParticleFilter::transform(ballPos);

//        ball->setPos(nowX + ballPos.x - fieldW / 2, nowY + ballPos.y - fieldH / 2);
//        ui.textEdit_area_2->setText(QString::number(Where_is_Robot(nowX + ballPos.x, nowY + ballPos.y)));
//    }

    posMsg.robot_area = Where_is_Robot(nowX, nowY);
    posMsg.ball_area = Where_is_Robot(ball_pos.x, ball_pos.y);
    posMsg.yaw = currentYaw;

    scene->addItem(ball);
    ball->setPos(ball_pos.x - fieldW / 2, ball_pos.y - fieldH / 2);
    ui.textEdit_area_2->setText(QString::number(Where_is_Robot(ball_pos.x, ball_pos.y)));


//    localization.clearWeights();

    localization.addParticles(cv::Point(nowX, nowY));
    //

    localization.createSamples();

    dX = 0; dY = 0;
    pastX = nowX; pastY = nowY;
    isUpdate = false;

    localPub.publish(posMsg);

    ui.lineEdit_time->setText(QString::number(++t));
}

void MainWindow::drawField()
{
    Point Origin(0,0);
    whitePen = QPen(Qt::white);
    yellowPen = QPen(Qt::yellow);
    blackPen = QPen(Qt::black);

    whitePen.setWidth(7); yellowPen.setWidth(10); blackPen.setWidth(3);

    scene = new QGraphicsScene(this);
    ui.graphicsView->setScene(scene);

    scene->setSceneRect(-(fieldW / 2), -(fieldH / 2), fieldW, fieldH);
    scene->setBackgroundBrush(Qt::darkGreen);

    QLineF TopLine(scene->sceneRect().topLeft(), scene->sceneRect().topRight());
    QLineF LeftLine(scene->sceneRect().topLeft(), scene->sceneRect().bottomLeft());
    QLineF RightLine(scene->sceneRect().topRight(), scene->sceneRect().bottomRight());
    QLineF BottomLine(scene->sceneRect().bottomLeft(), scene->sceneRect().bottomRight());

    scene->addLine(TopLine, whitePen);
    scene->addLine(LeftLine, whitePen);
    scene->addLine(RightLine, whitePen);
    scene->addLine(BottomLine, whitePen);
    scene->addLine(QLine(0,-(fieldH / 2),0,(fieldH / 2)), whitePen);

    //    scene->addEllipse(scene->sceneRect().center().x() - 5,
    //                      scene->sceneRect().center().y() - 5,
    //                      10, 10, whitePen);
    /** center circle */
    scene->addEllipse(scene->sceneRect().center().x() - (fieldH / 8),
                      scene->sceneRect().center().y() - (fieldH / 8),
                      (fieldH / 4), (fieldH / 4), whitePen);
    /** left penalty mark */
    scene->addEllipse(scene->sceneRect().center().x() - (fieldW * 8 / 30) - 3,
                      scene->sceneRect().center().y() - 3,
                      6, 6, whitePen);
    /** right penalty mark */
    scene->addEllipse(scene->sceneRect().center().x() + (fieldW * 8 / 30) - 3,
                      scene->sceneRect().center().y() - 3,
                      6, 6, whitePen);

    /** left goal post */
    scene->addRect(scene->sceneRect().center().x() - (fieldW / 2) - (fieldH / 10),
                   scene->sceneRect().center().y() - 130,
                   (fieldH / 10), 260, yellowPen);
    /** right goal post */
    scene->addRect(scene->sceneRect().center().x() + (fieldW / 2),
                   scene->sceneRect().center().y() - 130,
                   (fieldH / 10), 260, yellowPen);
    /** left goal area */
    scene->addRect(scene->sceneRect().center().x() - (fieldW / 2),
                   scene->sceneRect().center().y() - (fieldH - (fieldW / 9)) / 2,
                   (fieldW / 9), fieldH - (fieldW / 9), whitePen);
    /** right goal area */
    scene->addRect(scene->sceneRect().center().x() + (fieldW / 2) - (fieldW / 9),
                   scene->sceneRect().center().y() - (fieldH - (fieldW / 9)) / 2,
                   (fieldW / 9), fieldH - (fieldW / 9), whitePen);

    //Change Origin//
    Origin.x = scene->sceneRect().center().x() - (fieldW / 2);
    Origin.y = scene->sceneRect().center().y() - (fieldH / 2);

    /** area */
    scene->addLine(Origin.x, Origin.y + (fieldH-260)/2,
                   Origin.x + fieldW, Origin.y + (fieldH-260)/2, blackPen);
    scene->addLine(Origin.x, Origin.y + (fieldH-260)/2 + 260,
                   Origin.x + fieldW, Origin.y + (fieldH-260)/2 + 260, blackPen);


    scene->addLine(Origin.x + (fieldW / 9), Origin.y,
                   Origin.x + (fieldW / 9), Origin.y + fieldH, blackPen);
    scene->addLine(Origin.x + (fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4, Origin.y,
                   Origin.x + (fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4, Origin.y + fieldH, blackPen);
    scene->addLine(Origin.x + (fieldW / 9) + 2*(fieldW - ((fieldW / 9)*2))/4, Origin.y,
                   Origin.x + (fieldW / 9) + 2*(fieldW - ((fieldW / 9)*2))/4, Origin.y + fieldH, blackPen);
    scene->addLine(Origin.x + (fieldW / 9) + 3*(fieldW - ((fieldW / 9)*2))/4, Origin.y,
                   Origin.x + (fieldW / 9) + 3*(fieldW - ((fieldW / 9)*2))/4, Origin.y + fieldH, blackPen);
    scene->addLine(Origin.x + (fieldW / 9) + 4*(fieldW - ((fieldW / 9)*2))/4, Origin.y,
                   Origin.x + (fieldW / 9) + 4*(fieldW - ((fieldW / 9)*2))/4, Origin.y + fieldH, blackPen);


    //--Set Field Number
    int org_x = ui.graphicsView->geometry().center().x()-(fieldW/2);
    int org_y = ui.graphicsView->geometry().center().y()-(fieldH/2);
    ui.label_num1->setGeometry(org_x,org_y,31,31);
    ui.label_num2->setGeometry(org_x + (fieldW / 9),org_y,31,31);
    ui.label_num3->setGeometry(org_x + (fieldW / 9) + 1*(fieldW - ((fieldW / 9)*2))/4,org_y,31,31);
    ui.label_num4->setGeometry(org_x + (fieldW / 9) + 2*(fieldW - ((fieldW / 9)*2))/4,org_y,31,31);
    ui.label_num5->setGeometry(org_x + (fieldW / 9) + 3*(fieldW - ((fieldW / 9)*2))/4,org_y,31,31);
    ui.label_num6->setGeometry(org_x + (fieldW / 9) + 4*(fieldW - ((fieldW / 9)*2))/4,org_y,31,31);
    ui.label_num7->setGeometry(org_x,org_y + (fieldH-260)/2,31,31);
    ui.label_num8->setGeometry(org_x + (fieldW / 9),org_y + (fieldH-260)/2,31,31);
    ui.label_num9->setGeometry(org_x + (fieldW / 9) + 1*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2,31,31);
    ui.label_num10->setGeometry(org_x + (fieldW / 9) + 2*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2,51,41);
    ui.label_num11->setGeometry(org_x + (fieldW / 9) + 3*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2,51,41);
    ui.label_num12->setGeometry(org_x + (fieldW / 9) + 4*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2,51,41);
    ui.label_num13->setGeometry(org_x,org_y + (fieldH-260)/2 + 260,51,41);
    ui.label_num14->setGeometry(org_x + (fieldW / 9),org_y + (fieldH-260)/2 + 260,51,41);
    ui.label_num15->setGeometry(org_x + (fieldW / 9) + 1*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2 + 260,51,41);
    ui.label_num16->setGeometry(org_x + (fieldW / 9) + 2*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2 + 260,51,41);
    ui.label_num17->setGeometry(org_x + (fieldW / 9) + 3*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2 + 260,51,41);
    ui.label_num18->setGeometry(org_x + (fieldW / 9) + 4*(fieldW - ((fieldW / 9)*2))/4,org_y + (fieldH-260)/2 + 260,51,41);


}

void MainWindow::initField()
{
    Field_Area = new int*[fieldH+1];
    for(int i = 0; i < fieldH+1; ++i)
    {
        Field_Area[i] = new int[fieldW+1];
        memset(Field_Area[i],0,sizeof(int)*(fieldW+1));
    }


    for(int y = 0; y < (fieldH-260)/2; y++)
    {
        for(int x = 0; x < (fieldW+1); x++)
        {
            if(0 <= x && x < (fieldW / 9))
                Field_Area[y][x] = 1;
            else if((fieldW / 9) <= x && x < ((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4))
                Field_Area[y][x] = 2;
            else if(((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4) <= x && x < (fieldW/2))
                Field_Area[y][x] = 3;
            else if((fieldW/2) <= x && x < ((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)))
                Field_Area[y][x] = 4;
            else if(((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)) <= x && x < ((fieldW/2) + (2*((fieldW - ((fieldW / 9)*2))/4))))
                Field_Area[y][x] = 5;
            else
                Field_Area[y][x] = 6;
        }
    }
    for(int y = (fieldH-260)/2; y < ((fieldH-260)/2 + 260); y++)
    {
        for(int x = 0; x < (fieldW+1); x++)
        {
            if(0 <= x && x < (fieldW / 9))
                Field_Area[y][x] = 7;
            else if((fieldW / 9) <= x && x < ((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4))
                Field_Area[y][x] = 8;
            else if(((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4) <= x && x < (fieldW/2))
                Field_Area[y][x] = 9;
            else if((fieldW/2) <= x && x < ((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)))
                Field_Area[y][x] = 10;
            else if(((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)) <= x && x < ((fieldW/2) + (2*((fieldW - ((fieldW / 9)*2))/4))))
                Field_Area[y][x] = 11;
            else
                Field_Area[y][x] = 12;
        }
    }
    for(int y = ((fieldH-260)/2 + 260); y < (fieldH+1) ; y++)
    {
        for(int x = 0; x < (fieldW+1); x++)
        {
            if(0 <= x && x < (fieldW / 9))
                Field_Area[y][x] = 13;
            else if((fieldW / 9) <= x && x < ((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4))
                Field_Area[y][x] = 14;
            else if(((fieldW / 9) + (fieldW - ((fieldW / 9)*2))/4) <= x && x < (fieldW/2))
                Field_Area[y][x] = 15;
            else if((fieldW/2) <= x && x < ((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)))
                Field_Area[y][x] = 16;
            else if(((fieldW/2) + ((fieldW - ((fieldW / 9)*2))/4)) <= x && x < ((fieldW/2) + (2*((fieldW - ((fieldW / 9)*2))/4))))
                Field_Area[y][x] = 17;
            else
                Field_Area[y][x] = 18;
        }
    }

//    for(int y = 0; y < fieldH+1; y++)
//    {
//        for(int x = 0; x < fieldW+1; x++)
//            cout<<"["<<Field_Area[y][x]<<"] ";
//        cout<<endl;
//    }
}

int MainWindow::Where_is_Robot(int x, int y)
{
//    cout << "x = " << x << ", y = " << y << endl;
    if((0 <= x && x <= fieldW) && (0 <= y && y <= fieldH))
        return Field_Area[y][x];
    else
        return 0;
}

void MainWindow::transform(int &x, int &y)
{
    x -= (fieldW / 2);
    y -= (fieldH / 2);
}

MainWindow::~MainWindow()
{
    for(int i = 0; i < (fieldH+1); i++)
        delete[]Field_Area[i];

    delete[] Field_Area;
}


}  // namespace localization

