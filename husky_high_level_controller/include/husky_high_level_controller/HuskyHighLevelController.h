#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
};

} /* namespace */
