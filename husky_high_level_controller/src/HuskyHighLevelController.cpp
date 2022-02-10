#include <husky_high_level_controller/HuskyHighLevelController.h>

namespace husky_high_level_controller {

HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : _nh (nodeHandle) {
  if (!_nh.getParam("xVelocity", _paramLinearX) || !_nh.getParam("yawGain", _paramAngularYawGain)) { 
    ROS_ERROR("Could not read twist parameters!");
    ros::requestShutdown();
  }
  ROS_INFO_STREAM("Read twist parameters:" << std::endl << 
                  "xVelocity = " << _paramLinearX << std::endl << 
                  "yawGain" << _paramAngularYawGain);
  
  _sub = _nh.subscribe("/scan", 10, &HuskyHighLevelController::scanCallback, this);
  _pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  while (ros::ok()) {
    setTwist();
    ros::spinOnce();
    _loop.sleep();
  }
}

HuskyHighLevelController::~HuskyHighLevelController(){}

void HuskyHighLevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  double min = msg->range_max;
  int minIndex = 0;
	for (int i = 0; i < msg->ranges.size(); i++) {
		if (msg->ranges[i] < min) {
      min = msg->ranges[i];
      minIndex = i;
    }
	}
  _obstacleAngle = msg->angle_min + minIndex * msg->angle_increment;
  _minDistance = min;
  ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << _minDistance << " m | Obstacle Angle: " << _obstacleAngle << " rad");
}

void HuskyHighLevelController::setTwist() {
  geometry_msgs::Twist twist;
  twist.linear.x = _paramLinearX;
  twist.angular.z = _paramAngularYawGain * _obstacleAngle;
  _pub.publish(twist);
}

} /* namespace */
