#include<ros/ros.h>




int main(int argc, char **argv){
	ros::init(argc,argv,"ArmDrivr");	
	ros::NodeHandle n;
	//ros::ServiceServer service = n.advertiseService("Arm_setAngles",setAngles);
	ROS_INFO("Arm Driver Initialized");
	ros::spin();

	return 0;
}

