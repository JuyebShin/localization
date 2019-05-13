/**
 * @file /include/localization/main_window.hpp
 *
 * @brief Qt based gui for localization.
 *
 * @date November 2010
 **/
#ifndef localization_MAIN_WINDOW_H
#define localization_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <opencv2/opencv.hpp>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "myitem.h"

/*****************************************************************************
** Namespace
*****************************************************************************/



namespace localization {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

protected:
        void mousePressEvent(QMouseEvent *event);

public Q_SLOTS:
        void on_pushButton_set_clicked();
        void on_pushButton_reset_clicked();

        void updateYaw();
        void updateOdometry();
        void visionCallback();
        void update();

private:
        void drawField();
        void initField();
        int Where_is_Robot(int x, int y);
        void transform(int &x, int &y);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        QTimer* mTimer;
        QGraphicsScene *scene;
        MyItem *robot;
        MyBall *ball;

        cv::Mat cameraMatrix;
        cv::Mat distortionCoeff;

        QPen whitePen;
        QPen yellowPen;
        QPen blackPen;
        int fieldW;
        int fieldH;

        int datumYaw;
        int currentYaw;

        bool isSet;
        bool isUpdate;
        bool step;

        int avgX = 0, avgY = 0;
        int nowX = 0, nowY = 0;
        int pastX = 0, pastY = 0;
        int targetX = 0, targetY = 0;
        int dX = 0, dY = 0;
        int ballX = 0, ballY = 0;

        int **Field_Area;

        cv::Point ball_pos;

        volatile unsigned int t;

};

}  // namespace localization

#endif // localization_MAIN_WINDOW_H
