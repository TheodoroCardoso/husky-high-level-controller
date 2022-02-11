#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

namespace husky_high_level_controller {

// Class containing the Husky Highlevel Controller
class HuskyHighLevelController {
public:
	// Constructor
	HuskyHighLevelController(ros::NodeHandle& nodeHandle);

	// Destructor
	virtual ~HuskyHighLevelController();

private:
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void setTwist();
	void pubMarker();
	void markerSetUp();
	
	visualization_msgs::Marker _marker;

	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ros::Publisher _pub;
	ros::Publisher _markerPub;
	ros::Rate _loop {30};

	double _minDistance;
	double _obstacleAngle;
	double _paramCircleRadius;

	double _paramLinearX;
	double _paramAngularYawGain;
};

} /* namespace */
