#include "smartcar_bringup/base_controller2.h"

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "smartcar_controller2");
	smartcar::base_controller2 controller;
	controller.init();
	//controller.start();
	
	ROS_INFO("smartcar_controller is running ....... !");

	//ros::spin();
	ros::Rate r(50); // 10 hz
	while(ros::ok())
	{
		controller.odometry_publish();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}  // end main()
