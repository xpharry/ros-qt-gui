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
#include "../include/my_qt_gui_v1/qnode.hpp"

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

bool QNode::init(const std::string &master_url, const std::string &host_url) {
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
void QNode::run() {
    ros::Rate naptime(10.0); //create a ros object from the ros “Rate” class; 
    //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    std_msgs::Float64 velocity; //create a variable of type "Float64",
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission

    double amplitude = 1.0;
    double frequency = 5.0;
    int dt = 0;
	while ( ros::ok() ) {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        velocity.data = amplitude*sin(frequency*PI/180*dt); //phase increment by frequency*PI/180 radius each iteration
        dt++;
        ROS_INFO("Sending data %f", velocity.data);
        chatter_publisher.publish(velocity); // publish the value--of type Float64--
        std::ostringstream ss;
		ss << velocity.data;
		std::string s(ss.str()); 
		log(Info,std::string("The velocity now is: ")+s);
		ros::spinOnce();
        //the next line will cause the loop to sleep for the balance of the desired period
        // to achieve the specified loop frequency 
        naptime.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
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

}  // namespace my_qt_gui_v1
