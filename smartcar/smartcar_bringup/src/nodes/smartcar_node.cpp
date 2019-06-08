#include "smartcar_bringup/base_controller.h"

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "smartcar_controller");
	smartcar::base_controller controller;
	controller.init();
	
	ROS_INFO("smartcar_controller is running ....... !");

	controller.run();
	
	return 0;
}  // end main()