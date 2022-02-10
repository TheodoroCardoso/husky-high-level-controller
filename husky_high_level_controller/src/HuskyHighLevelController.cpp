#include <husky_high_level_controller/HuskyHighLevelController.h>

namespace husky_high_level_controller {

HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : _nh (nodeHandle) {
  _sub = _nh.subscribe("/scan", 10, &HuskyHighLevelController::scanCallback, this);
}

HuskyHighLevelController::~HuskyHighLevelController(){}

void HuskyHighLevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
double min = msg->range_max;
	for (int i = 0; i < msg->ranges.size(); i++) {
		if (msg->ranges[i] < min) min = msg->ranges[i];
	}
	ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
}

} /* namespace */
