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
#include "../include/localization/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace localization {

/*****************************************************************************
** Implementation
*****************************************************************************/

ros::Publisher localPub;

double yaw;
double panAngle = 0.0;
double tiltAngle = 0.0;

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
    ros::init(init_argc,init_argv,"localization");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    imuSub = n.subscribe("imu", 100, &QNode::imuCallback, this);
    coordinateSub = n.subscribe("ikcoordinate", 100, &QNode::coordinateCallback, this);
    ptSub = n.subscribe("pantilt", 100, &QNode::pantiltCallback, this);
    localSub = n.subscribe("robocup2019_local", 100, &QNode::localCallback, this);

    localPub = n.advertise<msg_generate::position_msg>("position", 100);

    start();

    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);

    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
    yaw = msg->yaw;

    if(yaw < 0) yaw = 360 + yaw;

    Q_EMIT recvImu();
}

void QNode::coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr &msg)
{
    Xmoved = msg->X / 8;
    Ymoved = (-1) * msg->Y / 8;

    Q_EMIT step();
}

void QNode::pantiltCallback(const msg_generate::pan_tilt_msg::ConstPtr &msg)
{
    panAngle = msg->Angle_Yaw;
    tiltAngle = msg->Angle_Pitch;
}

void QNode::localCallback(const msg_generate::localization_msg::ConstPtr &msg)
{
    localMsg.xcrossDist     = msg->xcrossDist;
    localMsg.xcrossTheta    = msg->xcrossTheta;

    localMsg.goalpostDist   = msg->goalpostDist;
    localMsg.goalpostTheta  = msg->goalpostTheta;

    localMsg.ballDist       = msg->ballDist;
    localMsg.ballTheta      = msg->ballTheta;

    Q_EMIT localUpdate();
}

}  // namespace localization
