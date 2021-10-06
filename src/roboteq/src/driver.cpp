#include "serial/serial.h"
#include "ros/ros.h"
#include <time.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "roboteq/msg_cmd_esc.h"
#include <sstream>
#include <thread> 

int speed_left = 0;
int speed_right = 0;

serial::Serial esc;
ros::Time last_msg;

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

void readData()
{
	std::string buff;
	ros::NodeHandle n;
	ros::Publisher pub_volts = n.advertise<std_msgs::Float64>("batt_volts", 1000);
	ros::Publisher pub_int = n.advertise<std_msgs::Float64>("internal_volts", 1000);
	ros::Publisher pub_amp_left = n.advertise<std_msgs::Float64>("left_amps", 1000);
	ros::Publisher pub_amp_right = n.advertise<std_msgs::Float64>("right_amps", 1000);
	ros::Rate loop_rate(2000); // Attention au buffer si on ne lit pas assez vite
	std_msgs::Float64 msg_volts;
	std_msgs::Float64 msg_int;
	std_msgs::Float64 msg_left;
	std_msgs::Float64 msg_right;
	while (ros::ok())
	{
		if(esc.available())
		{
		    std::string ch;
			if(esc.read(ch, 1) == 0)
			{
				std::cout << "Echec de la lecture" << std::endl;
			}
			else
			{
				if(ch == "\r")
				{
					std::vector<std::string> buff_split = split(buff,'=');
					if(buff_split.at(0) == "V")
					{
						msg_volts.data = std::stod(split(buff_split.at(1),':')[1])/10.;
						msg_int.data = std::stod(split(buff_split.at(1),':')[0])/10.;
						pub_volts.publish(msg_volts);
						pub_int.publish(msg_int);
					}
					else if(buff_split.at(0) == "BA")
					{
						msg_left.data = std::stod(split(buff_split.at(1),':')[0])/10.;
						msg_right.data = std::stod(split(buff_split.at(1),':')[1])/10.;
					    pub_amp_left.publish(msg_left);
						pub_amp_right.publish(msg_right);
					}
					buff = "";
				}
				else
				{
					buff = buff+ch;
				}
			}
		}

			

			loop_rate.sleep();
	}
}

void cmdCallback(const roboteq::msg_cmd_esc& msg)
{
	speed_left = msg.cmd_left;
	speed_right = msg.cmd_right;
	last_msg = ros::Time::now();
}

void callbackCmd_vel(const geometry_msgs::Twist& msg)
{
	double coef = 5;
	double alpha = 3/200.;

	speed_left = int(std::min(std::max(coef*(msg.linear.x-msg.angular.z)/alpha, -1000.0), 1000.0));
	speed_right = int(std::min(std::max(coef*(msg.linear.x+msg.angular.z)/alpha, -1000.0), 1000.0));
	last_msg = ros::Time::now();
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Roboteq_node");
	ros::NodeHandle nh("~");
    std::string port =  nh.param<std::string>("port","/dev/ttyUSB0");
	esc.setPort(port);
	esc.setBaudrate(19200);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
	esc.setTimeout(timeout);
	esc.open();
	int count = 0;
	if(esc.isOpen())
	{
		std::cout << "Réussite de l'ouverture" << std::endl;
		std::thread thread_data(readData); 
		esc.write("^ECHOF 1\r");
		esc.flush();
		ros::NodeHandle n;
		ros::Subscriber sub_esc = n.subscribe("cmd_esc", 1000, cmdCallback);
		ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1000, callbackCmd_vel);
		ros::Rate loop_rate(10);
		while (ros::ok())
		{
			if((ros::Time::now()-last_msg).toSec() > 1.)
			{
				speed_left = 0;
				speed_right = 0;
			}
			std::stringstream speed_left_str;
			std::stringstream speed_right_str;
			speed_left_str << "!G 1 " << speed_left << "\r";
			speed_right_str << "!G 2 " << speed_right << "\r";

			esc.write(speed_left_str.str());
			esc.write(speed_right_str.str());
			esc.flush();
			if(count == 9)
			{
				esc.write("?A\r");
				esc.write("?V\r");
				esc.write("?BA\r");
				count = 0;
			}	
			count++;
 		//	catch (serial::IOException& e)
 		//	{
 			//	ROS_ERROR("CATCH IO");
 			//	sleep(1.);
 			//	esc.open();
 		//	}
			ros::spinOnce();
			loop_rate.sleep();
			}
		esc.write("!G 1 0");
		esc.write("!G 2 0");
		thread_data.join();
		esc.close();
	}
	else
	{
		std::cout << "Echec de l'ouverture" << std::endl;
	}

}