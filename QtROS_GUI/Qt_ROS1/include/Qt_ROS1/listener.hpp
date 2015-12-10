/**
 * @file /qlistener/listener.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
//#include <string>
//#include <std_msgs/String.h>
#include "qnode.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
/*****************************************************************************
** Class
*****************************************************************************/

class Listener : public QNode {
	 Q_OBJECT
public:
	Listener(int argc, char** argv);
	virtual ~Listener() {}
	void run();
	void ros_comms_init();
Q_SIGNALS:
	void Update_Image(const QPixmap* image);
private:

	image_transport::Subscriber image_sub_;
//	void chatterCallback(const std_msgs::String::ConstPtr &msg);
	void chatterCallback(const sensor_msgs::ImageConstPtr& msg);
	QImage cvtCvMat2QImage(const cv::Mat & image);
	//ros::Subscriber chatter_subscriber;
};

#endif /* LISTENER_NODE_HPP_ */
