#include "ros/ros.h"
#include "std_msgs/String.h"

void MsgReceived(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received message: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"fms_kernel");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::String>("mavtx", 1000);
	ros::Subscriber sub = n.subscribe("mavrx",1000,MsgReceived);
	ros::Rate loop_rate(0.1);
	
	ROS_INFO("%s", "Starting FMS kernel...");
	
	while (ros::ok())
	{
		std_msgs::String pubmsg;
		std::stringstream ss;
		ss<<"param list";
		pubmsg.data = ss.str();
  
		ROS_INFO("Sending message: %s", pubmsg.data.c_str());
		pub.publish(pubmsg);
		
		ros::spinOnce();

		loop_rate.sleep();
	}
	
	return 0;
}
