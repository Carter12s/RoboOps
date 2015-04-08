#include <ros/ros.h>
#include "RO_srv/Driver_setThrottle.h"
#include <sensor_msgs/Joy.h>

ros::ServiceClient driver_setThrottle;
ros::ServiceClient driver_setSteering;
ros::Subscriber joy_sub;

bool timed_out = false;
float steering = 0.0f;
float throttle = 0.0f;

ros::Timer timer;
ros::Time last_update;
ros::Duration timeout(2.0); //2 seconds

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	steering = joy->axes[0];
	throttle = joy->axes[1];
	last_update = ros::Time::now();
	if(timed_out){
		timer.start();
		timed_out = false;
	}
}

void timerCallback(const ros::TimerEvent&){
	ros::Duration delta_t = ros::Time::now()-last_update;
	if(delta_t > timeout){
		RO_srv::Driver_setThrottle srv;
		srv.request.throttle = 0.0f;
		driver_setThrottle.call(srv);
		srv.request.throttle = 0.0f;
		driver_setSteering.call(srv);
		ROS_WARN("RO_Joy: Timeout hit!");
		timed_out = true;
		timer.stop();
	}else{
		RO_srv::Driver_setThrottle srv;
		srv.request.throttle = throttle;
		driver_setThrottle.call(srv);
		srv.request.throttle = steering;
		driver_setSteering.call(srv);
		ROS_INFO("Throttle Set to %f", throttle);
		ROS_INFO("Steering Set to %f", steering);
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"RO_Joy");
	ros::NodeHandle n;
	driver_setThrottle = n.serviceClient<RO_srv::Driver_setThrottle>("Driver_setThrottle");
	driver_setSteering = n.serviceClient<RO_srv::Driver_setThrottle>("Driver_setSteering");
	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
	timer = n.createTimer(ros::Duration(0.1),timerCallback);
	ros::spin();
	return 0;
}
