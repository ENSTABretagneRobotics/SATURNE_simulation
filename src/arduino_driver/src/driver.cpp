#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher chatter_rc_left;
ros::Publisher chatter_rc_right;
ros::Publisher chatter_rc_enabled;
serial::Serial rc;

std::vector<std::string> split(const std::string& s, char delimiter)
{
   //https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

int checksumNMEA(std::string data)
{
	int checksum = 0;
	for(int i = 0; i < data.length(); i++)
	{
		if(data.at(i)!= '$')
		{
			if(data.at(i) == '*')
			{
				return checksum;
			}
			else
			{
				checksum ^= (char)data.at(i);
			}

		}

	}

	return checksum;
}

bool parseNMEA(std::string data)
{
	int found_comma1,found_comma2,found_comma3,found_star;

	found_comma1 = data.find(",");
	found_star = data.find("*");
	std::string type_of_data;
  	if (found_comma1 == std::string::npos or found_star == std::string::npos)
  	{
  		return false;
  	}
  	
  	int checksum = std::stoi(data.substr(found_star+1,2),0,16);
  	std::string p1 = data.substr(1,found_comma1-1);
  	if(checksum != checksumNMEA(data))
  	{
  		ROS_WARN("INVALID CHECKSUM");
  		return false;
  	}
  	if(p1 != "PARDN")
  	{
  		return false;
  	}
  	data = data.substr(found_comma1+1,data.length());
  	found_comma2 = data.find(",");
  		type_of_data = data.substr(0,found_comma2);
  		if(type_of_data == "CMD_RC")
  		{
  			std::vector<std::string> info_data = split(data.substr(found_comma2+1,found_star-(found_comma1+found_comma2+2)),',');
  			if(info_data.size() != 4)
  			{
  				return false;
  			}
  			else
  			{	
  			/*	
				ROS_INFO("SWITCH : %s",info_data[0].c_str());
				ROS_INFO("FORWARD REVERSE: %s",info_data[1].c_str());
				ROS_INFO("RIGHT LEFT : %s",info_data[2].c_str());
  				ROS_INFO("SPEED : %s",info_data[3].c_str());
			*/
  				double FORWARD_REVERSE = std::stod(info_data[1]);
  				double RIGHT_LEFT = std::stod(info_data[2]);
  				double SWITCH = std::stod(info_data[0]);
  				double SPEED = std::stod(info_data[3]);

  				double cmd_right = SWITCH*SPEED*(std::min(100.,std::max(-100.,FORWARD_REVERSE - RIGHT_LEFT)))/100.;
  				double cmd_left = SWITCH*SPEED*(std::min(100.,std::max(-100.,FORWARD_REVERSE + RIGHT_LEFT)))/100.;

  				std_msgs::Float64 msg_right, msg_left;
  				std_msgs::Bool msg_enabled;
  				msg_left.data = cmd_left;
  				msg_right.data = cmd_right;

  				if(SWITCH == 1)
  				{
  					msg_enabled.data = true;
  				}
  				else
  				{
  					msg_enabled.data = false;
  				}

  				chatter_rc_left.publish(msg_left);
  				chatter_rc_right.publish(msg_right);
  				chatter_rc_enabled.publish(msg_enabled);

  				return true;
  			}
  		}
  		else
  		{
  			return false;
  		}


}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Arduino_driver_node");
	//ros::NodeHandle nh("~");
    //std::string port =  nh.param<std::string>("port","/dev/POLOLU");
    std::string port = "/dev/ttyS0";
   
	rc.setPort(port);
	rc.setBaudrate(57600);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	rc.setTimeout(timeout);
	rc.open();
	if(rc.isOpen())
	{
		ros::NodeHandle n;

		chatter_rc_left= n.advertise<std_msgs::Float64>("/rc/left", 1000);
		chatter_rc_right = n.advertise<std_msgs::Float64>("/rc/right", 1000);
		chatter_rc_enabled = n.advertise<std_msgs::Bool>("/rc/enabled", 1000);
		//ROS_INFO("Ready to receive data...");
		std::string buffer = "";
		
		while (ros::ok())
		{
			if(rc.available())
			{
			    std::string ch;
				if(rc.read(ch, 1) == 1)
				{
					if(ch == "$")
					{
						parseNMEA(buffer);
						buffer = ch;				
					}
					else
					{
						buffer = buffer + ch;
					}

				}
				else
				{
					ROS_WARN("Failure to read");
				}
			}

			usleep(1);
		}
		rc.close();
	}
	else
	{
		ROS_ERROR("FAILURE TO OPEN %s",port.c_str());
	}
	return 0;

}