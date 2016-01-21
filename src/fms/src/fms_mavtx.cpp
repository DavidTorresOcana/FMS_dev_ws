#include "mavlink_bridge_header.h"
#include "mavlink/common/mavlink.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


void sendMavMsg(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Sending mavlink message: %s", msg->data.c_str());
    
    mavlink_system.sysid = 100;
	mavlink_system.compid = 50;

	int system_id = 1;
	int component_id = 1;
	
	if (msg->data == "param list")
	{
		mavlink_msg_param_request_list_send(MAVLINK_COMM_0, system_id,component_id);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"fms_mavtx");
	
	ROS_INFO("%s", "Starting mavlink transmitter...");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("mavtx",1000,sendMavMsg);
	ros::spin();

	return 0;
}
