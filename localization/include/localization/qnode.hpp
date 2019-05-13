/**
 * @file /include/localization/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef localization_QNODE_HPP_
#define localization_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <omp.h>

#include <msg_generate/ikcoordinate_msg.h>
#include <msg_generate/imu_msg.h>
#include <msg_generate/pan_tilt_msg.h>
#include <msg_generate/localization_msg.h>
#include <msg_generate/position_msg.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace localization {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

        double Xmoved;
        double Ymoved;

        msg_generate::localization_msg localMsg;



Q_SIGNALS:
        void recvImu();
        void step();
        void localUpdate();

        void rosShutdown();

private:
	int init_argc;
	char** init_argv;
        ros::Subscriber coordinateSub;
        ros::Subscriber imuSub;
        ros::Subscriber ptSub;
        ros::Subscriber localSub;

        void imuCallback(const msg_generate::imu_msg::ConstPtr&);
        void coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr&);
        void pantiltCallback(const msg_generate::pan_tilt_msg::ConstPtr&);
        void localCallback(const msg_generate::localization_msg::ConstPtr&);

};

}  // namespace localization

#endif /* localization_QNODE_HPP_ */
