/**
 * @file /src/qnode2.cpp
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
#include <std_msgs/Float64.h> 
#include "../include/my_qt_gui_v1/qnode2.hpp"

/*****************************************************************************
** Global variables
*****************************************************************************/

double PI = 3.1415926;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_qt_gui_v1 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode2::QNode2(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode2::~QNode2() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode2::init() {
	ros::init(init_argc,init_argv,"my_qt_gui_v1");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::Float64>("vel_cmd", 1000);
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1000" argument says to use a buffer size of 1000; could make larger, if expect network backups 
	start();
	return true;
}

bool QNode2::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"my_qt_gui_v1");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::Float64>("vel_cmd", 1000);
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1000" argument says to use a buffer size of 1000; could make larger, if expect network backups
	start();
	return true;
}


// this the most relevant function we need to embed ros functions
void QNode2::run() {
	chatter_subscriber = n.subscribe("vel_cmd",1,myCallback);
	ros::spin(); //this is essentially a "while(1)" statement, except it 
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode2::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode2::myCallback(const std_msgs::Float64& message_holder) 
{ 
  ROS_INFO("received value is: %f",message_holder.data); 
  if (message_holder.data>0.5) 
      ROS_WARN("data is too big!!");
  //really could do something interesting here with the received data...but all we do is print it 
} 





}  // namespace my_qt_gui_v1
