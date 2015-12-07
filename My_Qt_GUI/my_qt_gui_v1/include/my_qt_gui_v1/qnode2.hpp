/**
 * @file /include/my_qt_gui_v1/qnode2.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef my_qt_gui_v1_QNODE2_HPP_
#define my_qt_gui_v1_QNODE2_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sstream>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_qt_gui_v1 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode2 : public QThread {
    Q_OBJECT
public:
	QNode2(int argc, char** argv );
	virtual ~QNode2();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Subscriber chatter_subscriber;
    QStringListModel logging_model;
};

}  // namespace my_qt_gui_v1

#endif /* my_qt_gui_v1_QNODE2_HPP_ */
