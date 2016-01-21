#include "mavlink_bridge_header.h"
#include "mavlink/common/mavlink.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
	ros::init(argc,argv,"fms_mavrx");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::String>("mavrx", 1000);
	ros::Rate loop_rate(57600);
	
	serial::Serial sc("/dev/ttyAMA0",57600);

	mavlink_message_t msg;
	mavlink_status_t status;
	ROS_INFO("%s", "Starting mavlink receiver...");
	
	while (ros::ok())
	{	
		uint8_t c;
		sc.read(&c,1);

		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			std::stringstream ss;
			ss<<"Message received: "<<(int)msg.sysid<<" "<<(int)msg.compid<<" "<<(int)msg.msgid;
			ROS_INFO("%s",ss.str().c_str());
			
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					break;
				}
				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					mavlink_param_value_t tmp;
					mavlink_msg_param_value_decode(&msg,&tmp);
					std_msgs::String pubmsg;
					std::stringstream ssm;
					ssm<<"Parameter: "<<tmp.param_id<<" "<<tmp.param_value;
					pubmsg.data = ssm.str();
			  
					ROS_INFO("%s", pubmsg.data.c_str());
					pub.publish(pubmsg);
					break;
				}
				default:
				{
					break;
				}
			}
			
			msg = mavlink_message_t();
			status = mavlink_status_t();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	sc.close();
	return 0;
}
