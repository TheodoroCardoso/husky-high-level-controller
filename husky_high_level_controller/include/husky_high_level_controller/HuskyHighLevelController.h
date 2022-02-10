#pragma once

#include <ros/ros.h>

namespace husky_high_level_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighLevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighLevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighLevelController();

private:
	ros::NodeHandle nodeHandle_;
};

} /* namespace */
