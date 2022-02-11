#include <husky_high_level_controller/HuskyHighLevelController.h>
# define M_PI           3.14159265358979323846  /* pi */

namespace husky_high_level_controller {

HuskyHighLevelController::HuskyHighLevelController(ros::NodeHandle& nodeHandle) : _nh (nodeHandle) {
  if (!_nh.getParam("xVelocity", _paramLinearX) || !_nh.getParam("yawGain", _paramAngularYawGain) || !_nh.getParam("circleRadius", _paramCircleRadius)) { 
    ROS_ERROR("Could not read twist parameters!");
    ros::requestShutdown();
  }
  ROS_INFO_STREAM("Read twist parameters:" << std::endl << 
                  "xVelocity = " << _paramLinearX << std::endl << 
                  "yawGain = " << _paramAngularYawGain << std::endl <<
                  "circleRadius = " << _paramCircleRadius);
  
  _sub = _nh.subscribe("/scan", 10, &HuskyHighLevelController::scanCallback, this);
  _pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  markerSetUp();
  _markerPub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  while (ros::ok()) {
    setTwist();
    pubMarker();
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
  ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << _minDistance << " m   | Obstacle Angle: " << _obstacleAngle << " rad");
}

void HuskyHighLevelController::setTwist() {
  geometry_msgs::Twist twist;
  float angleReference =  M_PI / 2;
  float angularDiff = angleReference -_obstacleAngle;
  float distanceError = _paramCircleRadius - _minDistance;    

  // Normalize angle -pi~pi
  angularDiff = fmod(angularDiff + M_PI, 2 * M_PI);
  if (angularDiff < 0) angularDiff += 2 * M_PI;
  float angularError = angularDiff - M_PI;
  
  
  twist.linear.x = _paramLinearX;

  // Go towards pillar
  if (_minDistance > _paramCircleRadius) twist.angular.z = - _obstacleAngle * _paramAngularYawGain;
  // Rotate to start circulating behavior
  else if (abs(angularError) > (M_PI / 8)) { 
    twist.linear.x = 0;  
    twist.angular.z = 0.2;
  }
  // Keeps distance from pillar while going around it forever
  else {
    twist.linear.x = _paramLinearX;
    // Feedback proportional controller to correct trajectory
    float fbYawControl = _paramAngularYawGain * angularError + distanceError / 2;
    // Feedforward controller. 1.3 m is to account for the half of the robot + pillar radius
    float ffYawControl = (_paramLinearX / (_paramCircleRadius + 1.3));
    twist.angular.z = fbYawControl - ffYawControl;    
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "Distance Error: " << distanceError << " m | Angular Error: " << angularError << " rad");
  
  _pub.publish(twist);
}

void HuskyHighLevelController::pubMarker(){
	_marker.header.stamp = ros::Time();
	_marker.pose.position.x = _minDistance * cos(_obstacleAngle);
	_marker.pose.position.y = _minDistance * sin(_obstacleAngle);
	_markerPub.publish(_marker);
}


void HuskyHighLevelController::markerSetUp(){
  _marker.header.frame_id = "base_laser";
	_marker.type = visualization_msgs::Marker::SPHERE;
	_marker.action = visualization_msgs::Marker::ADD;
	_marker.pose.position.z = 1;
  _marker.pose.orientation.w = 0.075;
	_marker.scale.x = 0.15;
	_marker.scale.y = 0.15;
	_marker.scale.z = 0.15;
	_marker.color.a = 1.0;
	_marker.color.g = 1.0;	
}

} /* namespace */
