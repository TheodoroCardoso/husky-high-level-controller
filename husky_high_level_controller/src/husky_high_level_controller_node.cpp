#include <ros/ros.h>
#include "husky_high_level_controller/HuskyHighLevelController.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "husky_high_level_controller");
  ros::NodeHandle nodeHandle("~");

  husky_high_level_controller::HuskyHighLevelController huskyHighLevelController(nodeHandle);

  ros::spin();
  return 0;
}
